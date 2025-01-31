/*************************************************************************
 * @file Input_meas.h
 * @date 2025/01/31
 *
 ************************************************************************/

#ifndef INPUT_MEAS_H_
#define INPUT_MEAS_H_

/*************************************************************************
 * Includes
 ************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA228.h>

/*************************************************************************
 * Class
 ************************************************************************/
class InputMeasurement {
public:
    /* Initialize the sensor */
    static void init();
    
    /* Get voltage measurement */
    static float getVoltage();
    
    /* Get current measurement */
    static float getCurrent();
    
    /* Get power measurement */
    static float getPower();

private:
    static Adafruit_INA228 ina228;
    static TwoWire I2CINA;
};

#endif /* INPUT_MEAS_H_ */
