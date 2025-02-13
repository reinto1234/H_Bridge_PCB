// #include <Arduino.h>
// #include "Output_meas.h"
// #include "Input_meas.h"
// #include "mutexdefinitions.h"

// // Create instances of measurement classes


// void setup() {
//     Serial.begin(115200);
    
//     // Initialize mutexes
//     measurementoutMutex = xSemaphoreCreateMutex();
//     measurementinMutex = xSemaphoreCreateMutex();
    
//     Serial.println("Initializing Output Measurement...");
//     OutputMeasurement::init();
//     Serial.println("Output Measurement Initialization complete.");
    
//     Serial.println("Initializing Input Measurement...");
//     InputMeasurement::init();
//     Serial.println("Input Measurement Initialization complete.");
// }

// void loop() {
//     Serial.println("\n--- Output Measurement Readings ---");
//     float* outputMeasurements = OutputMeasurement::measurementall();
//     Serial.print("Voltage: "); Serial.print(outputMeasurements[0]); Serial.println(" V");
//     Serial.print("Current: "); Serial.print(outputMeasurements[1]); Serial.println(" A");
//     Serial.print("Power: "); Serial.print(outputMeasurements[2]); Serial.println(" W");
//     Serial.print("Power Factor: "); Serial.print(outputMeasurements[3]); Serial.println();
//     Serial.print("Phase Angle: "); Serial.print(outputMeasurements[4]); Serial.println(" degrees");
//     Serial.print("Imaginary Power: "); Serial.print(outputMeasurements[5]); Serial.println(" VAR");
//     Serial.print("Frequency: "); Serial.print(outputMeasurements[6]); Serial.println(" Hz");
    
//     Serial.println("\n--- Input Measurement Readings ---");
//     float* inputMeasurements = InputMeasurement::measurementall();
//     Serial.print("Voltage: "); Serial.print(inputMeasurements[0]); Serial.println(" V");
//     Serial.print("Current: "); Serial.print(inputMeasurements[1]); Serial.println(" A");
//     Serial.print("Power: "); Serial.print(inputMeasurements[2]); Serial.println(" W");
    
//     Serial.println("------------------------------");
//     delay(2000); // Wait 2 seconds before the next measurement
// }