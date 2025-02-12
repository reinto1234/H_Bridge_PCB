/************************************************************************
 * @file main.cpp
 * @brief Multithreaded main file for the H-Bridge Inverter System
 * 
 * This example creates two tasks:
 *   - inverterTask: Runs the inverter control loop (reads sensor, updates PWM)
 *   - webSocketTask: Runs the WebSocket loop for real-time communications
 *
 * The HTTP server (AsyncWebServer) is initialized in setup() and handles the
 * file serving and parameter updates.
 ************************************************************************/

#include <Arduino.h>
#include "PWM.h"         // Contains HBridgeInverter class and startInverter/stopInverter functions
#include "webserver.h"   // Contains HBridgeWebServer class
#include "mutexdefinitions.h"
#include "Input_meas.h"
#include "Output_meas.h"

// --- Task Handles ---
TaskHandle_t inverterTaskHandle = NULL;
TaskHandle_t webSocketTaskHandle  = NULL;
TaskHandle_t measurementTaskHandle = NULL;
TaskHandle_t webSocketUpdateHandle = NULL;



/**
 * @brief Task that updates the inverter control loop.
 * 
 * This task reads an analog value (e.g., from A0) as a simulated measurement
 * and updates the inverter's control loop using a placeholder setpoint value.
 */
void inverterTask(void *pvParameters) {
  (void)pvParameters; // Unused parameter
  const int setpoint = 512;  // Placeholder setpoint for the PI control
  while (true) {
    if (xSemaphoreTake(inverterMutex, portMAX_DELAY) == pdTRUE) {
    if (inverter != nullptr) {
      // Read the measurement (replace with actual sensor reading if needed)
      float vMeasured = analogRead(A0);
      // Update the inverter control loop
      inverter->loop(setpoint, vMeasured);
    }
    xSemaphoreGive(inverterMutex);
    }
    // Small delay to avoid hogging the CPU
    vTaskDelay(pdMS_TO_TICKS(CYCLETIME_PWM));
  }
}

/**
 * @brief Task for running the WebSocket loop.
 * 
 * This task periodically updates the WebSocketsServer via the public accessor.
 */
void webSocketTask(void *pvParameters) {
  (void)pvParameters; // Unused parameter
  while (true) {
    // Use the public function to call the private webSocket.loop()
    HBridgeWebServer::loopWebSocket();
    vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms delay between updates
  }
}

void webSocketUpdate(void *pvParameters) {
  (void)pvParameters; // Unused parameter
  while (true) {
    float* measurementin = InputMeasurement::measurementall();
    float measurementout[7]={1,0,0,0,0,0,0};
    //float* measurementout = OutputMeasurement::measurementall();
    HBridgeWebServer::updateMeasurements(measurementin, measurementout);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay between updates
  }
}

void measurementTask(void *pvParameters) {
  (void)pvParameters; // Unused parameter
    while (true) {
        float* measurementin = InputMeasurement::measurementall();
        float measurementout[7]={0,0,0,0,0,0,0};
        //float* measurementout = OutputMeasurement::measurementall();
        if (inverter != nullptr){
        inverter->getmeasurements(measurementin, measurementout);
        }
        vTaskDelay(pdMS_TO_TICKS(0.5)); // 500 µs
}
}

/**
 * @brief Setup function.
 *
 * Initializes serial communications, WiFi, the webserver, and creates
 * two FreeRTOS tasks: one for the inverter control loop and one for the WebSocket loop.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Starting H-Bridge Inverter System with Multithreading...");

  inverterMutex = xSemaphoreCreateMutex();
  measurementinMutex = xSemaphoreCreateMutex();
  measurementoutMutex = xSemaphoreCreateMutex();


  // Initialize WiFi and HTTP/WebSocket server
  HBridgeWebServer::initWiFi();
  HBridgeWebServer::initServer();

  InputMeasurement::init();

  // Note: The inverter is started/stopped via HTTP endpoints (/start, /stop)

  // Create the inverter control task on core 1 with a 4KB stack and low priority (1)
  xTaskCreatePinnedToCore(
    inverterTask,        // Task function
    "InverterTask",      // Name of task
    4096,                // Stack size (in bytes)
    NULL,                // Parameter to pass
    1,                   // Task priority
    &inverterTaskHandle, // Task handle
    1                    // Core where the task should run
  );

  // Create the WebSocket task on core 0 with a 4KB stack and low priority (1)
  xTaskCreatePinnedToCore(
    webSocketTask,       // Task function
    "WebSocketTask",     // Name of task
    4096,                // Stack size (in bytes)
    NULL,                // Parameter to pass
    1,                   // Task priority
    &webSocketTaskHandle,// Task handle
    0                    // Core where the task should run
  );

  // Create the WebSocket task on core 0 with a 4KB stack and low priority (1)
  xTaskCreatePinnedToCore(
    measurementTask,       // Task function
    "Measurementtask",     // Name of task
    4096,                // Stack size (in bytes)
    NULL,                // Parameter to pass
    1,                   // Task priority
    &measurementTaskHandle,  // Task handle
    0                    // Core where the task should run
  );



  // Create the WebSocket task on core 0 with a 4KB stack and low priority (1)
  xTaskCreatePinnedToCore(
    webSocketUpdate,       // Task function
    "WebSocketUpdate",     // Name of task
    4096,                // Stack size (in bytes)
    NULL,                // Parameter to pass
    2,                   // Task priority
    &webSocketUpdateHandle,  // Task handle
    0                    // Core where the task should run
  );
}

/**
 * @brief Main loop.
 *
 * The main loop can remain empty (or perform low-priority background tasks)
 * because the functionality is split among FreeRTOS tasks.
 */
void loop() {
  // Nothing to do here – tasks are running independently.
  delay(1000);
}
