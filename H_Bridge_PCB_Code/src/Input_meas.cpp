/*************************************************************************
 * @file Input_meas.cpp
 * @date 2025/01/31
 *
 ************************************************************************/

#include "Input_meas.h"
#include "mutexdefinitions.h"

TwoWire InputMeasurement::I2CINA = TwoWire(0);
Adafruit_INA228 InputMeasurement::ina228;

#define ShuntResistor 0.015

float InputMeasurement::measurementBufferin[3] = {0.0, 0.0, 0.0}; // [Voltage, Current, Power]

void InputMeasurement::init() {
    I2CINA.begin(33, 32, 100000);

    if (!ina228.begin(0x40, &I2CINA)) {
        while (1); // Halt execution if sensor is not found
    }

    // Set the shunt resistor value
    ina228.setShunt(ShuntResistor);
}

float InputMeasurement::getVoltage() {
    return ((int)((ina228.readBusVoltage() / 1000000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getCurrent() {
    return ((int)((ina228.readCurrent() / 1000000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getPower() {
    return ina228.readPower()+1.1;
}

float* InputMeasurement::measurementall() {
    
    float voltage = ((int)((ina228.readBusVoltage() / 1000000) * 100 + 0.5) / 100.0);
    float current = ((int)((ina228.readCurrent() / 1000000) * 100 + 0.5) / 100.0);
    float power = ina228.readPower();

    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        measurementBufferin[0] = voltage;
        measurementBufferin[1] = current;
        measurementBufferin[2] = power;
        xSemaphoreGive(measurementinMutex);
    }
    return measurementBufferin;
    
}
