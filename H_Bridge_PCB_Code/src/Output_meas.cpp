/*************************************************************************
 * @file Output_meas.cpp
 * @brief Output measurements for the H-Bridge Inverter System
 *
 * This file contains the implementation of output measurements, including
 * voltage, current, power, power factor, phase, imaginary power, and frequency.
 ************************************************************************/

#include "Output_meas.h"
#include <cmath>
#include "mutexdefinitions.h"
#include "I2C.h"



ACS37800 OutputMeasurement::acs37800;

#define DIO_0_PIN 25  // Zero-crossing detection
#define DIO_1_PIN 26  // Event detection (e.g., overcurrent)

// Interrupt variables
volatile unsigned long lastZeroCrossTime = 0;
volatile double frequency = 0;
unsigned long debounceTime = 800; // Minimum time between zero-crossing detections (in microseconds)

portMUX_TYPE zeroCrossMux = portMUX_INITIALIZER_UNLOCKED;  // Mutex for ISR-safe critical section

float OutputMeasurement::measurementBufferout[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [Voltage, Current, Power, Powerfactor, Phase, ImaginaryPower, Frequency]

// Interrupt Service Routine for Zero-Cross Detection
void IRAM_ATTR zeroCrossISR() {
    unsigned long currentTime = micros();  // Use micros() for better precision

    // Debouncing the zero-cross event to avoid noise or multiple triggers
    if (currentTime - lastZeroCrossTime > debounceTime) {
        // Calculate frequency (inverse of period in microseconds, converted to Hz)
        unsigned long timeDifference = currentTime - lastZeroCrossTime;
        portENTER_CRITICAL(&zeroCrossMux);  // Enter critical section to protect shared resource
        frequency = double(1000000) / (double(timeDifference));  // Convert to frequency in Hz
        portEXIT_CRITICAL(&zeroCrossMux);   // Exit critical section
        lastZeroCrossTime = currentTime;
        
    }
    //Serial.println("Zero-cross detected");
}

void OutputMeasurement::init() {

    // Configure GPIO for Zero-Crossing and Event Detection
    pinMode(DIO_0_PIN, INPUT_PULLUP);
    attachInterrupt(DIO_0_PIN, zeroCrossISR, FALLING); // Interrupt on falling edge for zero-cross

    pinMode(DIO_1_PIN, INPUT_PULLUP);
    


    I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed); // SDA on IO32, SCL on IO33
    Serial.println("I2CINA Output initialized");
    sleep(1);   // Wait for the sensor to initialize
    

    bool sensorFound = false;
    while (!sensorFound) {
        // Attempt to initialize the ACS37800 sensor
        if (acs37800.begin(ACaddress, I2CINA)) { // 0x66 is the I2C address for ACS37800
            Serial.println("ACS37800 found!");
            sensorFound = true; // Sensor found, exit the loop
        } else {
            // If the sensor is not found, print an error message and retry
            Serial.println("ACS37800 not found. Retrying...");
            delay(1000); // Wait before retrying (adjust as needed)
        }
    }
    parametrizeSensor(); // Initialize the sensor parameters
    //acs37800.enableDebugging(Serial); // Enable debugging on Serial port
    
   
}

float OutputMeasurement::getVoltage() {
    float voltage, current;
    if (acs37800.readRMS(&voltage, &current) == ACS37800_SUCCESS) {
        return voltage;
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getCurrent() {
    float voltage, current;
    if (acs37800.readRMS(&voltage, &current) == ACS37800_SUCCESS) {
        return current;
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getPower() {
    float activePower, reactivePower;
    if (acs37800.readPowerActiveReactive(&activePower, &reactivePower) == ACS37800_SUCCESS) {
        return activePower* -1.0;; // Apparent power
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getPowerfactor() {
    float apparentPower, powerFactor;
    bool posAngle, posPf;
    if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
        return fabs(powerFactor); // Power factor is related to phase angle
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getPhase() {
    float apparentPower, powerFactor, Phase;
    bool posAngle, posPf;
    if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
        // Calculate phase angle in degrees from power factor
        Phase = acos(fabs(powerFactor)) * (180.0 / M_PI);
        return Phase;
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getImaginaryPower() {
    float activePower, reactivePower;
    if (acs37800.readPowerActiveReactive(&activePower, &reactivePower) == ACS37800_SUCCESS) {
        return reactivePower * -1.0;
    }
    return -1.0; // Return error value
}

float* OutputMeasurement::measurementall() {
    float voltage = getVoltage();
    float current = getCurrent();
    float power = getPower();
    float powerfactor = getPowerfactor();
    float phase = getPhase();
    float imaginaryPower = getImaginaryPower();
    float frequency = getFrequency();

    if (xSemaphoreTake(measurementoutMutex, portMAX_DELAY) == pdTRUE) {
        measurementBufferout[0] = voltage;
        measurementBufferout[1] = current;
        measurementBufferout[2] = power;
        measurementBufferout[3] = powerfactor;
        measurementBufferout[4] = phase;
        measurementBufferout[5] = imaginaryPower;
        measurementBufferout[6] = frequency;
        xSemaphoreGive(measurementoutMutex);
    }
    return measurementBufferout;
}

float OutputMeasurement::getFrequency() {
    // To safely access frequency from ISR, use a critical section
    double currentFrequency;
    portENTER_CRITICAL(&zeroCrossMux);  // Enter critical section to protect shared resource
    currentFrequency = frequency; // Return the calculated frequency
    portEXIT_CRITICAL(&zeroCrossMux);   // Exit critical section
    return float(currentFrequency);
}

void OutputMeasurement::parametrizeSensor(){
    // ACS37800_REGISTER_0B_t reg1B;
    // ACS37800_REGISTER_0C_t reg1C;
    // ACS37800_REGISTER_0D_t reg1D;
    // ACS37800_REGISTER_0E_t reg1E;
    // ACS37800_REGISTER_0F_t reg1F;
    //int newValue = 1; // Default value for the register
    //newValue &= 0b1; // For safety, mask off any invalid bits
    //reg1E.data.bits.delaycnt_sel = newValue;
    //reg1E.data.bits.halfcycle_en = newValue; // Enable half-cycle mode
    //newValue= 63;
    //newValue &= 0b111111; // For safety, mask off any invalid bits
    //reg1E.data.bits.overvreg = newValue;
    //acs37800.writeRegister(reg1E.data.all, ACS37800_REGISTER_EEPROM_0E); delay(100); acs37800.writeRegister(reg1E.data.all, ACS37800_REGISTER_SHADOW_1E); delay(100);

     // From the ACS37800 datasheet:
    // CONFIGURING THE DEVICE FOR AC APPLICATIONS : DYNAMIC CALCULATION OF N
    // Set bypass_n_en = 0 (default). This setting enables the device to
    // dynamically calculate N based off the voltage zero crossings.
    acs37800.setBypassNenable(false, true); // Disable bypass_n in shadow memory and eeprom

    // We need to connect the LO pin to the 'low' side of the AC source.
    // So we need to set the divider resistance to 4M Ohms (instead of 2M).
    acs37800.setDividerRes(4000000); // Comment this line if you are using GND to measure the 'low' side of the AC voltage
    acs37800.setSenseRes(20000); // Set the sense resistor value to 1mOhm (default)
    //acs37800.setCurrentRange(30.0); // Set the current sensing range to 30A (default)
    //acs37800.writeRegister(0x00000000, ACS37800_REGISTER_EEPROM_0E); // Clear the error flags register


}
