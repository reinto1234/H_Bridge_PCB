// /*************************************************************************
//  * @file Output_meas.cpp
//  * @brief Output measurements for the H-Bridge Inverter System
//  *
//  * This file contains the implementation of output measurements, including
//  * voltage, current, power, power factor, phase, imaginary power, and frequency.
//  ************************************************************************/

// #include "Output_meas.h"
// #include <cmath>
// #include "mutexdefinitions.h"
// #include "I2C.h"



// ACS37800 OutputMeasurement::acs37800;

// //Interrupt variables
// hw_timer_t* readTimer = nullptr;
// portMUX_TYPE readTimerMux = portMUX_INITIALIZER_UNLOCKED;


// #define DIO_0_PIN 25  // Zero-crossing detection
// #define DIO_1_PIN 26  // Event detection (e.g., overcurrent)

// // Interrupt variables
// volatile unsigned long lastZeroCrossTime = 0;
// volatile double frequency = 0;
// unsigned long debounceTime = 2000; // Minimum time between zero-crossing detections (in microseconds)
// String OutputMeasurement::vInstCSV;
// bool OutputMeasurement::sampleNow = false; // Flag to indicate if a sample should be taken

// portMUX_TYPE zeroCrossMux = portMUX_INITIALIZER_UNLOCKED;  // Mutex for ISR-safe critical section

// float OutputMeasurement::measurementBufferout[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [Voltage, Current, Power, Powerfactor, Phase, ImaginaryPower, Frequency]

// // Interrupt Service Routine for Zero-Cross Detection
// void IRAM_ATTR zeroCrossISR() {
//     unsigned long currentTime = micros();  // Use micros() for better precision

//     // Debouncing the zero-cross event to avoid noise or multiple triggers
//     if (currentTime - lastZeroCrossTime > debounceTime) {
//         // Calculate frequency (inverse of period in microseconds, converted to Hz)
//         unsigned long timeDifference = currentTime - lastZeroCrossTime;
//         portENTER_CRITICAL(&zeroCrossMux);  // Enter critical section to protect shared resource
//         frequency = double(1000000) / (double(timeDifference));  // Convert to frequency in Hz
//         portEXIT_CRITICAL(&zeroCrossMux);   // Exit critical section
//         lastZeroCrossTime = currentTime;
        
//     }
//     //Serial.println("Zero-cross detected");
// }

// void IRAM_ATTR readInstantaneousISR() {

//     OutputMeasurement::sampleNow = true;
// }

// void OutputMeasurement::init() {

//     // Configure GPIO for Zero-Crossing and Event Detection
//     pinMode(DIO_0_PIN, INPUT_PULLUP);
//     attachInterrupt(DIO_0_PIN, zeroCrossISR, FALLING); // Interrupt on falling edge for zero-cross

//     pinMode(DIO_1_PIN, INPUT_PULLUP);
    


//     I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed); // SDA on IO32, SCL on IO33
//     Serial.println("I2CINA Output initialized");
//     sleep(1);   // Wait for the sensor to initialize

//     //interrupt initialization
//     readTimer = timerBegin(1, 80, true); // 80 prescaler => 1us ticks (1MHz) → Core 1
//     timerAttachInterrupt(readTimer, &readInstantaneousISR, true);
//     timerAlarmWrite(readTimer, 1000, true); // 1ms = 1000us
//     timerAlarmEnable(readTimer);
//     vInstCSV = ""; // Initialize the CSV string for voltage data

    

//     bool sensorFound = false;
//     while (!sensorFound) {
//         // Attempt to initialize the ACS37800 sensor
//         if (acs37800.begin(ACaddress, I2CINA)) { // 0x66 is the I2C address for ACS37800
//             Serial.println("ACS37800 found!");
//             sensorFound = true; // Sensor found, exit the loop
//         } else {
//             // If the sensor is not found, print an error message and retry
//             Serial.println("ACS37800 not found. Retrying...");
//             delay(1000); // Wait before retrying (adjust as needed)
//         }
//     }
//     parametrizeSensor(); // Initialize the sensor parameters
//     //acs37800.enableDebugging(Serial); // Enable debugging on Serial port
    
   
// }

// float OutputMeasurement::getinstantaneous() {
//     float vInst, iInst, pInst;
//     if (acs37800.readInstantaneous(&vInst, &iInst, &pInst) == ACS37800_SUCCESS) {
//         return vInst; // Return instantaneous voltage
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getVoltage() {
//     float voltage, current;
//     if (acs37800.readRMS(&voltage, &current) == ACS37800_SUCCESS) {
//         return voltage;  //Voltage converted due to sensor calibration and voltage divider variation
        
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getCurrent() {
//     float voltage, current;
//     if (acs37800.readRMS(&voltage, &current) == ACS37800_SUCCESS) {
//         return current;
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getPower() {
//     float activePower, reactivePower;
//     if (acs37800.readPowerActiveReactive(&activePower, &reactivePower) == ACS37800_SUCCESS) {
//         return activePower* -1.0;; // Apparent power, Sensor is mounted the wrong way around, so we need to invert the sign
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getPowerfactor() {
//     float apparentPower, powerFactor;
//     bool posAngle, posPf;
//     if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
//         return powerFactor * 1.0; // Power factor is related to phase angle, Sensor is mounted the wrong way around, so we need to invert the sign
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getPhase() {
//     float apparentPower, powerFactor, Phase;
//     bool posAngle, posPf;
//     if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
//         // Calculate phase angle in degrees from power factor
//         Phase = acos(powerFactor * -1.0) * (180.0 / M_PI); // Convert radians to degrees Sensor is mounted the wrong way around, so we need to invert the sign
//         return Phase;
//     }
//     return -1.0; // Return error value
// }

// float OutputMeasurement::getImaginaryPower() {
//     float activePower, reactivePower;
//     if (acs37800.readPowerActiveReactive(&activePower, &reactivePower) == ACS37800_SUCCESS) {
//         return reactivePower * -1.0; // Imaginary power, Sensor is mounted the wrong way around, so we need to invert the sign
//     }
//     return -1.0; // Return error value
// }



// float* OutputMeasurement::measurementall() {
    
//     float voltage, current, power, powerfactor, phase, imaginaryPower, frequency;
//     if (micros() - lastZeroCrossTime > 100000) { // If no zero-crossing detected in the last 10ms
//         frequency = 0.0; // If no zero-crossing detected, set frequency to 0
//     }
//     else {
//         frequency = getFrequency(); // Get the frequency from the zero-crossing detection
//     }
    

//     if (frequency == 0.0) {
//         voltage = 0.0;
//         current = 0.0;
//         power = 0.0;
//         powerfactor = 0.0;
//         phase = 0.0;
//         imaginaryPower = 0.0;
//     }

//     else {
//         voltage = getVoltage();
//         current = getCurrent();
//         power = getPower();
//         powerfactor = getPowerfactor();
//         phase = getPhase();
//         imaginaryPower = getImaginaryPower();
//     }

//     if (xSemaphoreTake(measurementoutMutex, portMAX_DELAY) == pdTRUE) {
//         measurementBufferout[0] = voltage;
//         measurementBufferout[1] = current;
//         measurementBufferout[2] = power;
//         measurementBufferout[3] = powerfactor;
//         measurementBufferout[4] = phase;
//         measurementBufferout[5] = imaginaryPower;
//         measurementBufferout[6] = frequency;
//         xSemaphoreGive(measurementoutMutex);
//     }
//     return measurementBufferout;
// }

// float OutputMeasurement::getFrequency() {
//     // To safely access frequency from ISR, use a critical section
//     double currentFrequency;
//     portENTER_CRITICAL(&zeroCrossMux);  // Enter critical section to protect shared resource
//     currentFrequency = frequency; // Return the calculated frequency
//     portEXIT_CRITICAL(&zeroCrossMux);   // Exit critical section
//     return float(currentFrequency); // Return frequency in Hz, divided by 2 to account for the zero-crossing detection
// }

// void OutputMeasurement::parametrizeSensor(){
//     // ACS37800_REGISTER_0B_t reg1B;
//     // ACS37800_REGISTER_0C_t reg1C;
//     // ACS37800_REGISTER_0D_t reg1D;
//     // ACS37800_REGISTER_0E_t reg1E;
//     // ACS37800_REGISTER_0F_t reg1F;
//     //int newValue = 1; // Default value for the register
//     //newValue &= 0b1; // For safety, mask off any invalid bits
//     //reg1E.data.bits.delaycnt_sel = newValue;
//     //reg1E.data.bits.halfcycle_en = newValue; // Enable half-cycle mode
//     //newValue= 63;
//     //newValue &= 0b111111; // For safety, mask off any invalid bits
//     //reg1E.data.bits.overvreg = newValue;
//     //acs37800.writeRegister(reg1E.data.all, ACS37800_REGISTER_EEPROM_0E); delay(100); acs37800.writeRegister(reg1E.data.all, ACS37800_REGISTER_SHADOW_1E); delay(100);

//      // From the ACS37800 datasheet:
//     // CONFIGURING THE DEVICE FOR AC APPLICATIONS : DYNAMIC CALCULATION OF N
//     // Set bypass_n_en = 0 (default). This setting enables the device to
//     // dynamically calculate N based off the voltage zero crossings.
//     acs37800.setBypassNenable(false, true); // Disable bypass_n in shadow memory and eeprom


//     // We need to connect the LO pin to the 'low' side of the AC source.
//     // So we need to set the divider resistance to 4M Ohms (instead of 2M).
//     acs37800.setDividerRes(400000); // Comment this line if you are using GND to measure the 'low' side of the AC voltage
//     acs37800.setSenseRes(2000); // Set the sense resistor value to 1mOhm (default)
//     acs37800.setCurrentRange(30.0); // Set the current sensing range to 30A (default)
//     //acs37800.writeRegister(0x00000000, ACS37800_REGISTER_EEPROM_0E); // Clear the error flags register


// }
