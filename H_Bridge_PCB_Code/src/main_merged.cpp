
// /************************************************************************
//  * @file main_merged.cpp
//  * @brief Merged main: keeps multithreaded webserver/tasks from your last main.cpp
//  *        and integrates the new AMC1306-based OutputMeasurement pipeline from main_Outputconfig.cpp.
//  ************************************************************************/

// #include <Arduino.h>
// #include <SPI.h>

// // --- Inverter / server / mutexes ---
// #include "PWM.h"
// #include "webserver.h"
// #include "mutexdefinitions.h"

// // --- Input / Output measurement headers ---
// #include "Input_meas.h"
// #include "Output_meas_try.h"   // Provides AMC1306Measurement

// #if defined(ARDUINO_ARCH_ESP32)
// extern "C" {
//   #include "freertos/FreeRTOS.h"
//   #include "freertos/task.h"
//   #include "freertos/semphr.h"
// }
// #endif

// // ------------------- Config -------------------
// #define CYCLETIME_WEBSOCKET_UPDATE 500 // ms
// #define CYCLETIME_WEBSOCKET        10  // ms

// // -------- Pins for AMC1306 --------
// #define PIN_SCLK 17  // CLKIN -> AMC1306 CLKIN
// #define PIN_MISO 18  // DOUT  -> AMC1306 DOUT

// // -------- Clock & Decimation --------
// static const uint32_t F_CLK_HZ = 5000000UL; // 5 MHz
// static const uint16_t OSR      = 2000;      // sinc^3 -> fs=F_CLK/OSR





// // -------- SPI + Measurement instance --------
// SPIClass vspi(VSPI);
// AMC1306Measurement measV(
//   vspi, PIN_SCLK, PIN_MISO,
//   F_CLK_HZ, OSR,
//   /* d_seconds         */ 2,       // 2 seconds of decimated history
//   /* bytes_per_service */ 2048,    // smaller -> fresher decimated samples
//   /* fs_mV             */ 320.0f
// );


// // ------------------- Globals -------------------
// // Task handles
// static TaskHandle_t webSocketUpdateHandle = nullptr;
// static TaskHandle_t webSocketTaskHandle   = nullptr;
// static TaskHandle_t inverterTaskHandle    = nullptr;
// static TaskHandle_t measurementTaskHandle = nullptr;

// // Thread-safe shared outputs derived from AMC1306
// static String   g_outCSV;      // CSV of last full period samples
// static float    g_freqHz = NAN;
// static float    g_rmsmV  = NAN;

// // ------------------- Helpers -------------------
// static void ensureMutexes() {
//   if (inverterMutex == nullptr)        inverterMutex        = xSemaphoreCreateMutex();
//   if (measurementinMutex == nullptr)   measurementinMutex   = xSemaphoreCreateMutex();
//   if (measurementoutMutex == nullptr)  measurementoutMutex  = xSemaphoreCreateMutex();
// }

// static void startInverterTask(void*) {
//   ensureMutexes();
//   const float KP=0, KI=0, OUT_MIN=0, OUT_MAX=1023;
//   const ModulationType MOD = BIPOLAR;
//   const int PWM_FREQ = 20000;
//   startInverter(KP, KI, OUT_MIN, OUT_MAX, MOD, PWM_FREQ);
//   vTaskDelete(nullptr);
// }

// // Continuously clocks the AMC1306 and, when a full period is available,
// // computes frequency/RMS and builds a CSV string of that period's samples.
// // Continuously clocks the AMC1306 and publishes results.
// // In measurementTask():
// static void measurementTask(void*) {
//   static uint32_t lastStatMs = 0;
//   static uint64_t lastBytes  = 0;
//   Serial.println("Hello");
  

//   const TickType_t servicePeriod = pdMS_TO_TICKS(1);
//   TickType_t lastWake = xTaskGetTickCount();

//   uint32_t lastComputeMs = 0;

//   for (;;) {
//     // Drive ADC + decimate continuously
//     measV.service();

//     // Publish fresh numbers every 20 ms (or 10 ms if you want)
//     uint32_t now = millis();
//     const uint32_t COMPUTE_PERIOD_MS = 50;  // <-- tune to 10/20/50 as desired
//     if (now - lastStatMs >= 1000) {
//       Serial.println("Hello1");
//   lastStatMs = now;
//   uint64_t bTot  = measV.rxBytesTotal();
//   uint64_t bDiff = bTot - lastBytes;
//   lastBytes = bTot;

//   Serial.printf("[SPI] rx: +%llu B/s (total %llu), blocks=%llu, decim_fs=%.1f Hz, ring=%u\n",
//                 (unsigned long long)bDiff,
//                 (unsigned long long)bTot,
//                 (unsigned long long)measV.rxBlocksTotal(),
//                 measV.fsDecimatedHz(),
//                 (unsigned)measV.ringSamplesAvailable());
// }

//     if (now - lastComputeMs >= COMPUTE_PERIOD_MS) {
//       lastComputeMs = now;
//       Serial.println("Hello2");

//       // Ask compute for a short window (0=auto ~1.5 periods), or force 20 ms, etc.
//       // e.g., measV.compute(20);
//       if (measV.compute(200)) {
//         Serial.println("Hello3");
//         float fHz  = measV.frequencyHz();
//         float rRMS = measV.rmsLastPeriod();

//         if (measurementoutMutex) xSemaphoreTake(measurementoutMutex, portMAX_DELAY);
//         g_freqHz = fHz;
//         Serial.println(fHz);
//         g_rmsmV  = rRMS;
//         Serial.println(rRMS);
//         if (measurementoutMutex) xSemaphoreGive(measurementoutMutex);
//       }
//     }

//     vTaskDelayUntil(&lastWake, servicePeriod);
//   }
// }

// // Keep the websocket server pumping
// static void webSocketTask(void *pvParameters) {
//   (void)pvParameters;
//   for (;;) {
//     HBridgeWebServer::loopWebSocket();
//     vTaskDelay(pdMS_TO_TICKS(CYCLETIME_WEBSOCKET));
//   }
// }

// // Package up measurements and push to the web client periodically.
// // Input side remains your existing InputMeasurement; output side
// // now uses the AMC1306-derived values computed above.
// static void webSocketUpdate(void *pvParameters) {
//   (void)pvParameters;
//   for (;;) {
//     // Read input measurement array from your existing module
//     //float* measurementin = InputMeasurement::measurementall();
//     float measurementin[3] = {0}; // Fallback if InputMeasurement fails

//     // Prepare an output array for the webserver.
//     // NOTE: Adjust mapping/order to match what your frontend expects.
//     static float measurementout[7] = {0};

//     if (measurementoutMutex) xSemaphoreTake(measurementoutMutex, portMAX_DELAY);
//     float freq = g_freqHz;
//     float rms  = g_rmsmV;
//     //String csv = g_outCSV;
//     //g_outCSV.remove(0);  // clear CSV after taking a copy
//     if (measurementoutMutex) xSemaphoreGive(measurementoutMutex);

//     // Example mapping:
//     // [0]=Vrms(mV), [1]=Frequency(Hz), [2..6]=unused/reserved
//     measurementout[0] = rms;
//     measurementout[1] = freq;

//     // Push arrays to the web client (same API as before)
//     HBridgeWebServer::updateMeasurements(measurementin, measurementout);

//     // Also print CSV for logging/plotting, if any
//     //if (csv.length() > 0) {
//     //  Serial.println(csv);
//     //}
//     Serial.println(freq);
//     Serial.println(rms);
//     size_t n;
//   const float* p = measV.lastPeriod(n);
//   if (p && n > 0) {
//     Serial.print("Last period (");
//     Serial.print(n);
//     Serial.println(" samples):");
//     for (size_t i = 0; i < n; i++) {
//       Serial.print(p[i], 2);  // 2 decimal places
//       Serial.print(i == n-1 ? '\n' : ',');
//   }
// }


//     vTaskDelay(pdMS_TO_TICKS(CYCLETIME_WEBSOCKET_UPDATE));
//   }
// }

// // ------------------- Arduino entry points -------------------
// void setup() {
//   Serial.begin(115200);
//   ensureMutexes();

//   // Init legacy input measurement (unchanged)
//   //InputMeasurement::init();

//   // Init AMC1306 measurement (sets up SPI and warms the ring)
  
  

//   // Bring up WiFi + WebServer
//   HBridgeWebServer::initWiFi();
//   HBridgeWebServer::initServer();
//   Serial.println("WebServer initialized!");

//   vspi.end();
//   measV.begin();

  

  

//   // Inverter init (one-shot) on Core 0
//   //xTaskCreatePinnedToCore(startInverterTask, "SPWMInit", 4096, nullptr, 3, &inverterTaskHandle, 0);

//   // Measurement (continuous) on Core 1
//   xTaskCreatePinnedToCore(measurementTask, "Measure", 2*8192, nullptr, 2, &measurementTaskHandle, 1);

//   // WebSocket update & pump (Core 0)
//   xTaskCreatePinnedToCore(webSocketUpdate, "WebSocketUpdate", 4096, nullptr, 1, &webSocketUpdateHandle, 0);
//   xTaskCreatePinnedToCore(webSocketTask,   "WebSocketTask",   4096, nullptr, 1, &webSocketTaskHandle,   0);
// }

// void loop() {
//   vTaskDelay(1); // idle
// }
