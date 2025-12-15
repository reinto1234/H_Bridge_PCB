// *************************************************************************
//  @file main.cpp
//  @brief Minimal main: just initialization + task creation + loop
// *************************************************************************

#include <Arduino.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
  #include "esp_log.h"
}
      // new: sampler/analyzer/web tasks + contexts
#include "mutexdefinitions.h" // your existing mutex externs/defs
#include "Input_meas.h"
#include "webserver.h"
#include "spi_sampler.h"
#include "Tasks.h"
#include "safety.h"

static const char* TAG = "HB-INV+AMC1306";

TaskHandle_t webSocketUpdateHandle = NULL;
TaskHandle_t webSocketTaskHandle   = NULL;
TaskHandle_t spiSampler1Handle     = NULL;
TaskHandle_t spiSampler2Handle     = NULL;
TaskHandle_t analyzer1Handle       = NULL;
TaskHandle_t analyzer2Handle       = NULL;
TaskHandle_t printerHandle         = NULL;
TaskHandle_t ControllerTaskHandle  = NULL;
TaskHandle_t estopTaskHandle       = NULL;

void setup() {
  Serial.begin(115200);
  delay(100);
  ESP_LOGI(TAG, "Start H-Bridge + AMC1306 dual capture (DMA triple-buffer + analyzer + Web)");

  // --- Your app init ---
  inverterMutex       = xSemaphoreCreateMutex();
  measurementinMutex  = xSemaphoreCreateMutex();
  measurementoutMutex = xSemaphoreCreateMutex();

  InputMeasurement::init();

  // --- WiFi / WebServer ---
  HBridgeWebServer::initWiFi();
  HBridgeWebServer::initServer();
  Serial.println("WebServer initialized!");

  // --- Init AMC1306 measurement engines ---
  if (!spiSamplerInitMeasurements()) {
    Serial.println("FATAL: OutputMeasurements init failed");
    abort();
  }

  // --- Start SPI engines (both) ---
  spiInitAndStart(&g_sampler1);
  spiInitAndStart(&g_sampler2);

  pinMode(ALERT_PIN, INPUT);  // external 3k3 pull-up present, don't use INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(ALERT_PIN), onAlertISR, RISING);
  pinMode(ESTOP_OUTPUT_PIN, OUTPUT);
  digitalWrite(ESTOP_OUTPUT_PIN, HIGH); // active HIGH
  delay(500);
  digitalWrite(ESTOP_OUTPUT_PIN, LOW);  // deactivate estop output after delay
  delay(500);
  digitalWrite(ESTOP_OUTPUT_PIN, HIGH); // active HIGH
  delay(500);
  digitalWrite(ESTOP_OUTPUT_PIN, LOW);  // deactivate estop output after delay


  Serial.println("Starting tasks...");

  // --- Create tasks ---
  // Samplers: high priority
  xTaskCreatePinnedToCore(spiSamplerTask, "spi_sampler_ch1",
                          4096, &g_pack1, configMAX_PRIORITIES - 2, &spiSampler1Handle, 1);
  xTaskCreatePinnedToCore(spiSamplerTask, "spi_sampler_ch2",
                          4096, &g_pack2, configMAX_PRIORITIES - 2, &spiSampler2Handle, 1);

  // Analyzers: medium priority
  xTaskCreatePinnedToCore(analyzerTask, "analyzer_ch1",
                          6144, &g_ch1_om, 2, &analyzer1Handle, 1);
  xTaskCreatePinnedToCore(analyzerTask, "analyzer_ch2",
                          6144, &g_ch2_om, 2, &analyzer2Handle, 1);

  // WebSocket loop + periodic updates (core 0 to keep WiFi stack happy)
  xTaskCreatePinnedToCore(webSocketTask, "WebSocketTask",
                          4096, NULL, 3, &webSocketTaskHandle, 0);
  xTaskCreatePinnedToCore(webSocketUpdate, "WebSocketUpdate",
                          4096, NULL, 2, &webSocketUpdateHandle, 0);

  //xTaskCreatePinnedToCore(printerTask, "printer",
  //                                4096, NULL, 1, &printerHandle, 0);

  xTaskCreatePinnedToCore(ControllerTask, "ControllerTask",
                          4096, NULL, 4, &ControllerTaskHandle, 0);

  xTaskCreatePinnedToCore(EmergencyStopTask, "EmergencyStop",
                            2048, nullptr, configMAX_PRIORITIES-1,
                            &estopTaskHandle, 0);

  
Serial.printf("heap free=%u  (8bit=%u  dma=%u  internal=%u)\n",
  esp_get_free_heap_size(),
  heap_caps_get_free_size(MALLOC_CAP_8BIT),
  heap_caps_get_free_size(MALLOC_CAP_DMA),
  heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void loop() {
  // nothing — everything runs in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
