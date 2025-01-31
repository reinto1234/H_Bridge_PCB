/*************************************************************************
 * @file Input_meas.cpp
 * @date 2025/01/31
 *
 ************************************************************************/

#include "Input_meas.h"

TwoWire InputMeasurement::I2CINA = TwoWire(0);
Adafruit_INA228 InputMeasurement::ina228;

#define ShuntResistor 0.015

void InputMeasurement::init() {
    I2CINA.begin(33, 32, 100000);

    if (!ina228.begin(0x40, &I2CINA)) {
        while (1); // Halt execution if sensor is not found
    }

    // Configure the shunt resistor valuelalalallalalalaa
    ina228.setShunt(ShuntResistor);
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
