/*************************************************************************
 * @file Input_meas.cpp
 * @brief Input measurements for the H-Bridge Inverter System
 *
 * This file contains the implementation of input measurements, including
 * voltage, current, and power.
 ************************************************************************/

#include "Input_meas.h"
#include "mutexdefinitions.h"
#include "I2C.h"


Adafruit_INA228 InputMeasurement::ina228;

#define ShuntResistor 0.015

float InputMeasurement::measurementBufferin[3] = {0.0, 0.0, 0.0}; // [Voltage, Current, Power]

void InputMeasurement::init() {

    I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);
    sleep(1);   // Wait for the sensor to initialize
    scanI2C();
    Serial.println("I2CINA initialized");

    if (!ina228.begin(0x40, &I2CINA)) {
        while (1); // Halt execution if sensor is not found
    }
    Serial.println("INA228 found!");

    // Set the shunt resistor value
    ina228.setShunt(ShuntResistor);
}

void InputMeasurement::init1(){
    I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);
}

float InputMeasurement::getVoltage() {
    return ((int)((ina228.readBusVoltage() / 1000000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getCurrent() {
    return ((int)((ina228.readCurrent() / 1000000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getPower() {
    return ina228.readPower();
}

float* InputMeasurement::measurementall() {
    
    float voltage = getVoltage();
    float current = getCurrent();
    float power = getPower();

    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        measurementBufferin[0] = voltage;
        measurementBufferin[1] = current;
        measurementBufferin[2] = power;
        xSemaphoreGive(measurementinMutex);
    }
    return measurementBufferin;
    
}

void InputMeasurement::scanI2C() {
    Serial.println("Scanning I2C devices...");
    byte deviceCount = 0;

    // Scan for devices on the I2C bus
    for (byte address = 1; address < 127; address++) {
        I2CINA.beginTransmission(address);
        if (I2CINA.endTransmission() == 0) {  // If no error, device found
            deviceCount++;
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("I2C scan complete.");

    // If fewer than two devices are found, restart the I2C bus
    // if (deviceCount < 2) {
    //     Serial.println("Less than two devices found, restarting I2C...");
    //     I2CINA.end();  // End the current I2C bus communication
    //     delay(1000);    // Small delay before restarting the bus
    //     I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);  // Restart the I2C bus
    //     delay(1000);
    //     scanI2C();  // Scan again after restarting
    // }
}
