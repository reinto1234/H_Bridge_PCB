/*************************************************************************
 * @file Output_meas.h
 * @date 2025/01/31
 *
 ************************************************************************/

#ifndef OUTPUT_MEAS_H_
#define OUTPUT_MEAS_H_

/*************************************************************************
 * Includes
 ************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_ACS37800_Arduino_Library.h>

/*************************************************************************
 * Class
 ************************************************************************/
class OutputMeasurement {
public:
    /* Initialize the sensor */
    static void init();
    
    /* Get voltage measurement */
    static float getVoltage();
    
    /* Get current measurement */
    static float getCurrent();
    
    /* Get power measurement */
    static float getPower();
    
    /* Get powerfactor measurement */
    static float getPowerfactor();

    /* Get phase measurement */
    static float getPhase();
    
    /* Get imaginary power measurement */
    static float getImaginaryPower();
    
    /* Get frequency measurement */
    static float getFrequency();

private:
    static ACS37800 acs37800;
    static TwoWire I2CACS;
};

#endif /* OUTPUT_MEAS_H_ */