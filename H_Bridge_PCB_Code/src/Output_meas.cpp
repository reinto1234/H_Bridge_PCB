/*************************************************************************
 * @file Output_meas.cpp
 * @date 2025/01/31
 *
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
volatile unsigned long frequency = 0;
unsigned long debounceTime = 1000; // Minimum time between zero-crossing detections (in microseconds)

portMUX_TYPE zeroCrossMux = portMUX_INITIALIZER_UNLOCKED;  // Mutex for ISR-safe critical section

float OutputMeasurement::measurementBufferout[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [Voltage, Current, Power, Powerfactor, Phase, ImaginaryPower, Frequency]

// Interrupt Service Routine for Zero-Cross Detection
void IRAM_ATTR zeroCrossISR() {
    unsigned long currentTime = micros();  // Use micros() for better precision

    // Debouncing the zero-cross event to avoid noise or multiple triggers
    if (currentTime - lastZeroCrossTime > debounceTime) {
        // Calculate frequency (inverse of period in microseconds, converted to Hz)
        unsigned long timeDifference = currentTime - lastZeroCrossTime;
        if (timeDifference > 0) {
            frequency = 1000000 / (timeDifference * 2);  // Convert to frequency in Hz
        }
        lastZeroCrossTime = currentTime;
    }
    //Serial.println("Zero-cross detected");
}

void OutputMeasurement::init() {
    I2CINA.begin(32, 33, 100000); // SDA on IO32, SCL on IO33
    Serial.println("I2CINA initialized");
    sleep(1);

    if (!acs37800.begin(0x62, I2CINA)) { // 0x60 is the default I2C address
        while (1); // Halt execution if sensor is not found
    }
    Serial.println("ACS37800 found!");

    // Configure GPIO for Zero-Crossing and Event Detection
    pinMode(DIO_0_PIN, INPUT_PULLUP);
    attachInterrupt(DIO_0_PIN, zeroCrossISR, FALLING); // Interrupt on falling edge for zero-cross

    pinMode(DIO_1_PIN, INPUT_PULLUP);
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
        return activePower;
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getPowerfactor() {
    float apparentPower, powerFactor;
    bool posAngle, posPf;
    if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
        return powerFactor; // Power factor is related to phase angle
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getPhase() {
    float apparentPower, powerFactor, Phase;
    bool posAngle, posPf;
    if (acs37800.readPowerFactor(&apparentPower, &powerFactor, &posAngle, &posPf) == ACS37800_SUCCESS) {
        // Calculate phase angle in degrees from power factor
        Phase = acos(powerFactor) * (180.0 / M_PI);
        return Phase;
    }
    return -1.0; // Return error value
}

float OutputMeasurement::getImaginaryPower() {
    float activePower, reactivePower;
    if (acs37800.readPowerActiveReactive(&activePower, &reactivePower) == ACS37800_SUCCESS) {
        return reactivePower;
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
    float currentFrequency;
    portENTER_CRITICAL(&zeroCrossMux);  // Enter critical section to protect shared resource
    currentFrequency = frequency; // Return the calculated frequency
    portEXIT_CRITICAL(&zeroCrossMux);   // Exit critical section
    return currentFrequency;
}
