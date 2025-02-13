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

#include <SparkFun_ACS37800_Arduino_Library.h>
#include "mutexdefinitions.h"

/*************************************************************************
 * Class
 ************************************************************************/
class OutputMeasurement {
public:
    static float* measurementall();
    /* Initialize the sensor */
    static void init();

private:
    static float measurementBufferout[7]; // [Voltage, Current, Power, Powerfactor, Phase, ImaginaryPower, Frequency]
    static ACS37800 acs37800;

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
    
};

#endif /* OUTPUT_MEAS_H_ */