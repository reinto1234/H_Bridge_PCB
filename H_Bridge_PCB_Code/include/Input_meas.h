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
#include <Adafruit_INA228.h>

/*************************************************************************
 * Class
 ************************************************************************/
class InputMeasurement {
public:
    /* Initialize the sensor */
    static void init();
    static float* measurementall();
    static void init1();
    static void clearflags(){
        ina228.setAlertLatch(INA228_ALERT_LATCH_TRANSPARENT);
        ina228.alertFunctionFlags();
    }


private:
    static Adafruit_INA228 ina228;
    
    static float measurementBufferin[3]; // [Voltage, Current, Power]
    /* Get voltage measurement */
    static float getVoltage();

    static void scanI2C();

    static void configure_overcurrent_alert_only(float I_limit_A, float R_shunt_ohm);

    /* Get current measurement */
    static float getCurrent();
    
    /* Get power measurement */
    static float getPower();
};

#endif /* INPUT_MEAS_H_ */
