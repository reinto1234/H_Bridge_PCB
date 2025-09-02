// /************************************************************************
//  * @file main.cpp
//  * @brief Multithreaded main file for the H-Bridge Inverter System
//  ************************************************************************/

// #include <Arduino.h>
// #include "PWM.h"
// #include "webserver.h"
// #include "mutexdefinitions.h"
// #include "Input_meas.h"
// #include "Output_meas.h"

// #define CYCLETIME_MEASUREMENT 1     // Measurement cycle time in ms
// #define CYCLETIME_WEBSOCKET_UPDATE 500 // WebSocket update cycle time in ms
// #define CYCLETIME_WEBSOCKET 10  // WebSocket cycle time in ms

// // --- Task Handles ---
// TaskHandle_t measurementTaskHandle = NULL;
// TaskHandle_t webSocketUpdateHandle = NULL;
// TaskHandle_t webSocketTaskHandle = NULL;



// /**
//  * @brief Task to periodically read sensor data into global buffers.
//  * This is the ONLY task that should call the measurement functions.
//  */
// void measurementTask(void *pvParameters) {
//     for (;;) {
//         if (OutputMeasurement::sampleNow) {
//             OutputMeasurement::sampleNow = false;
//             // Safe to call I2C here
//             float vInst = OutputMeasurement::getinstantaneous();
//             OutputMeasurement::vInstCSV += String(vInst, 6) + "\n"; // Append instantaneous voltage to CSV string
//         }

//         vTaskDelay(100); // Let lower-priority tasks run
//     }
// }

// void webSocketTask(void *pvParameters) {
//   (void)pvParameters; // Unused parameter
//   while (true) {
//     // Use the public function to call the private webSocket.loop()
//     HBridgeWebServer::loopWebSocket();
//     vTaskDelay(pdMS_TO_TICKS(CYCLETIME_WEBSOCKET)); // 10 ms delay between updates
//   }
// }

// /**
//  * @brief Task to periodically send data from global buffers via WebSocket.
//  * This task READS data that measurementTask WRITES.
//  */
// void webSocketUpdate(void *pvParameters) {
//   (void)pvParameters; // Unused parameter
//   while (true) {
//     //float measurementin[3]={1,0,0};
//     float* measurementin = InputMeasurement::measurementall();
//     //float measurementout[7]={1,0,0,0,0,0,0};
//     float* measurementout = OutputMeasurement::measurementall();
//     HBridgeWebServer::updateMeasurements(measurementin, measurementout);
//     Serial.println(OutputMeasurement::vInstCSV); // Print the CSV string for debugging
//     OutputMeasurement::vInstCSV = ""; // Clear the CSV string after sending
//     vTaskDelay(pdMS_TO_TICKS(CYCLETIME_WEBSOCKET_UPDATE)); // 500 ms delay between updates
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting H-Bridge Inverter System with Multithreading...");

//   inverterMutex = xSemaphoreCreateMutex();
//   measurementinMutex = xSemaphoreCreateMutex();
//   measurementoutMutex = xSemaphoreCreateMutex();

//   InputMeasurement::init();
//   OutputMeasurement::init();

//   HBridgeWebServer::initWiFi();
//   HBridgeWebServer::initServer();
//   Serial.println("WebServer initialized!");

//   // Create the measurement task on core 0
//   xTaskCreatePinnedToCore(
//     measurementTask,
//     "MeasurementTask",
//     4096,
//     NULL,
//     2, // Higher priority to ensure it gets data first
//     &measurementTaskHandle,
//     0
//   );

//   // Create the WebSocket update task on core 0
//   xTaskCreatePinnedToCore(
//     webSocketUpdate,
//     "WebSocketUpdate",
//     4096,
//     NULL,
//     1, // Lower priority
//     &webSocketUpdateHandle,
//     0
//   );

//    // Create the WebSocket task on core 0 with a 4KB stack and low priority (1)
//   xTaskCreatePinnedToCore(
//     webSocketTask,       // Task function
//     "WebSocketTask",     // Name of task
//     4096,                // Stack size (in bytes)
//     NULL,                // Parameter to pass
//     1,                   // Task priority
//     &webSocketTaskHandle,// Task handle
//     0                    // Core where the task should run
//   );
// }

// void loop() {
//   // Nothing to do here
// }

