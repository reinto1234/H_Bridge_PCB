// #include "Output_meas.h"
// #include <cmath>
// #include "mutexdefinitions.h"
// #include "I2C.h"

// // Sensor- und Puffer-Definitionen
// ACS37800 OutputMeasurement::acs37800;
// float OutputMeasurement::measurementBufferout[7] = {0.0f}; // Vrms, Irms, Power, Freq, Vinst, Iinst, Pinst

// // --- Konfigurationsparameter für Berechnungen ---
// #define ALPHA 0.1f
// #define HISTORY_SIZE 5


// void OutputMeasurement::init() {
//     I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);
//     Serial.println("I2CINA Output initialized");
    
//     while (!acs37800.begin(ACaddress, I2CINA)) {
//         Serial.println("ACS37800 not found. Retrying...");
//         delay(1000);
//     }
//     Serial.println("ACS37800 found!");
    
//     // Sensorkonfiguration
//     acs37800.setBypassNenable(false, true);
//     acs37800.setDividerRes(4000000);
//     acs37800.setSenseRes(20000);
//     acs37800.setCurrentRange(30.0);
// }

// // Diese Funktion wird vom Task zyklisch aufgerufen
// void OutputMeasurement::updateAll() {
//     static float vSumOfSquares = 0.0f, iSumOfSquares = 0.0f, powerSum = 0.0f;
//     static int sampleCount = 0;
//     static int dynamicWindowSize = 200;

//     static float filteredVoltage = 0.0f;
//     static bool isFirstMeasurement = true;
//     static float historyBuffer[5] = {0.0f};
//     static int measurementCount = 0;

//     static float currentFrequency = 50.0f;
//     static unsigned long lastUpdateTime = 0;
//     static unsigned long lastPeakTime = 0;

//     float vInst, iInst, pInst;
//     unsigned long now = micros();
//     unsigned long taskInterval = now - lastUpdateTime;
//     lastUpdateTime = now;

//     if (acs37800.readInstantaneous(&vInst, &iInst, &pInst) == ACS37800_SUCCESS) {
//         // --- Step 1: Filter voltage for peak detection ---
//         if (isFirstMeasurement) {
//             filteredVoltage = vInst;
//             isFirstMeasurement = false;
//         } else {
//             filteredVoltage = (ALPHA * vInst) + ((1.0f - ALPHA) * filteredVoltage);
//         }
        

//         // Shift history buffer
//         for (int i = 0; i < 4; i++) {
//             historyBuffer[i] = historyBuffer[i + 1];
//         }
//         historyBuffer[4] = filteredVoltage;
//         if (measurementCount < 5) measurementCount++;

//         // --- Step 2: Peak detection for frequency ---
//         if (measurementCount >= 5) {
//             bool isPeak =
//                 (historyBuffer[2] > historyBuffer[0] && historyBuffer[2] > historyBuffer[1] &&
//                  historyBuffer[2] > historyBuffer[3] && historyBuffer[2] > historyBuffer[4]);

//             if (isPeak) {
//                 unsigned long period = now - lastPeakTime;
//                 if (lastPeakTime > 0 && period > 0) {
//                     currentFrequency = 1000000.0f / period;
//                     for (int i = 0; i < 4; i++) {
                        
//                     }
                    

//                     // Update RMS window size based on frequency
//                     if (taskInterval > 0) {
//                         dynamicWindowSize = (int)(1000000.0f / (currentFrequency * taskInterval));
//                         dynamicWindowSize = max(20, min(dynamicWindowSize, 500)); // clamp
//                     }

//                     if (xSemaphoreTake(measurementoutMutex, (TickType_t)10) == pdTRUE) {
//                         measurementBufferout[3] = currentFrequency;
//                         xSemaphoreGive(measurementoutMutex);
//                     }
//                 }
//                 lastPeakTime = now;
//             }
//         }

//         // --- Step 3: Accumulate for RMS ---
//         vSumOfSquares += vInst * vInst;
//         iSumOfSquares += iInst * iInst;
//         powerSum -= pInst;
//         sampleCount++;

//         if (sampleCount >= dynamicWindowSize) {
//             float vrms = sqrt(vSumOfSquares / sampleCount);
//             float irms = sqrt(iSumOfSquares / sampleCount);
//             float activePower = powerSum / sampleCount;

//             if (xSemaphoreTake(measurementoutMutex, (TickType_t)10) == pdTRUE) {
//                 measurementBufferout[0] = vrms;
//                 measurementBufferout[1] = irms;
//                 measurementBufferout[2] = activePower;
//                 xSemaphoreGive(measurementoutMutex);
//             }

//             // Reset for next window
//             vSumOfSquares = 0.0f;
//             iSumOfSquares = 0.0f;
//             powerSum = 0.0f;
//             sampleCount = 0;
//         }

//         // --- Step 4: Save instantaneous values ---
//         if (xSemaphoreTake(measurementoutMutex, (TickType_t)10) == pdTRUE) {
//             measurementBufferout[4] = vInst;
//             measurementBufferout[5] = iInst;
//             measurementBufferout[6] = pInst;
//             xSemaphoreGive(measurementoutMutex);
//         }
//     }
// }
