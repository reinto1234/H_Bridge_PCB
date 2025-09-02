// // main.cpp

// #include <Arduino.h>
// #include <SPI.h>

// #include "PWM.h"
// #include "mutexdefinitions.h"
// #include "Output_meas.h"

// #if defined(ARDUINO_ARCH_ESP32)
// extern "C" {
//   #include "freertos/FreeRTOS.h"
//   #include "freertos/task.h"
// }
// #endif

// // ---- Cores ----
// #define CORE_0 0
// #define CORE_1 1

// // ---- Pins ----
// #define PIN_SCLK 17  // CLKIN -> AMC1306 CLKIN
// #define PIN_MISO 18  // DOUT  -> AMC1306 DOUT

// // ---- SPI instance ----
// SPIClass vspi(VSPI);

// // ---- Measurement class ----
// AMC1306Measurement measV(vspi, PIN_SCLK, PIN_MISO);

// // ---- Forward declares ----
// static void startInverterTask(void* pvParameters);
// static void measureTask(void* pvParameters);

// static void ensureMutexes() {
//   if (inverterMutex == nullptr)        inverterMutex       = xSemaphoreCreateMutex();
//   if (measurementinMutex == nullptr)   measurementinMutex  = xSemaphoreCreateMutex();
//   if (measurementoutMutex == nullptr)  measurementoutMutex = xSemaphoreCreateMutex();
//   if (measurementSpiMutex == nullptr)  measurementSpiMutex = xSemaphoreCreateMutex(); // dedicated for AMC1306 SPI
// }

// static void startInverterTask(void*) {
//   ensureMutexes();
//   const float KP=0, KI=0, OUT_MIN=0, OUT_MAX=1023;
//   const ModulationType MOD = UNIPOLAR;
//   const int PWM_FREQ = 20000;
//   startInverter(KP, KI, OUT_MIN, OUT_MAX, MOD, PWM_FREQ);
//   vTaskDelete(nullptr);
// }


// static void measureTask(void*) {
//   ensureMutexes();

//   // Keep AMC1306 SPI bit clock running continuously via service()
//   // Compute new results every 150 ms, but only print every 10 s.
//   const uint32_t COMPUTE_INTERVAL_MS = 120;    // 5 Hz measurement
//   const uint32_t PRINT_INTERVAL_MS   = 10000;  // print every 10 s

//   uint32_t lastCompute = 0;
//   uint32_t lastPrint   = 0;

//   for (;;) {
//     // 1) Drive SPI clock & fill the raw bit ring (continuous)
//     measV.service();

//     // 2) Periodic compute (fast)
//     uint32_t now = millis();
//     if ((uint32_t)(now - lastCompute) >= COMPUTE_INTERVAL_MS) {
//       lastCompute = now;
//       (void)measV.compute();  // updates freq, last full period, and RMS
//     }

//     // 3) Slow print (uses the latest computed results)
//     if ((uint32_t)(now - lastPrint) >= PRINT_INTERVAL_MS) {
//       lastPrint = now;

//       // Optionally refresh once more right before printing to ensure freshest buffer:
//       //(void)measV.compute();

//       // Print frequency
//       float fHz = measV.frequencyHz();
//       Serial.print("freq_hz=");
//       if (isnan(fHz)) Serial.println("nan");
//       else            Serial.println(fHz, 2);
//       //fHz= measV.calcFreqFast();
//       //Serial.println(fHz);
      

//       // Print the last full period samples as CSV (one line)
//       size_t n = 0;
//       const float* p = measV.lastPeriod(n);
//       if (p && n) {
//         for (size_t i = 0; i < n; ++i) {
//           Serial.print(p[i], 3);
//           Serial.print(",");
//         }
//         Serial.println();
//         // Print RMS
//         Serial.print("rms_mV=");
//         Serial.println(measV.rmsLastPeriod(), 3);
//       } else {
//         Serial.println(); // empty CSV line for consistency
//         Serial.println("rms_mV=nan");
//       }
//     }

//     taskYIELD();
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   ensureMutexes();

//   // Init measurement (sets up SPI and warms the ring/CIC)
//   measV.begin();

//   // Inverter on Core 0
//   xTaskCreatePinnedToCore(startInverterTask, "SPWMInit", 4096, nullptr, 3, nullptr, CORE_0);

//   // Measurement on Core 1
//   xTaskCreatePinnedToCore(measureTask, "Measure", 6144, nullptr, 2, nullptr, CORE_1);

  
// }

// void loop() {
//   vTaskDelay(1); // idle
// }
