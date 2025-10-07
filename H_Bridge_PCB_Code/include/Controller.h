/*************************************************************************
 * @file Controller.h
 * @date 2025/01/31
 *
 ************************************************************************/
#ifndef CONTROLLER_H
#define CONTROLLER_H

#define Controller_Timecycle 20  // Controller-Zykluszeit in ms

#include "mutexdefinitions.h"
#include <atomic>
#include "Output_meas.h"

class PIController {
public:
    PIController(float kp, float ki, float outputMin, float outputMax, float vrms);
    void ControlRMS(float* RMSvoltage);
    float factor = 1.0f; // Control factor, later multiplied with the PWM signal to adjust amplitude
    static int16_t g_amp_q10; // Declaration only; definition must be in .cpp file
    uint16_t stepindexPWM = 0;
    
    

private:
    float compute(float setpoint, float measurement);
    float kp;
    float ki;
    float integral;
    float prevError;
    float outputMin = 0.0f;
    float outputMax = 1.0f;
    uint32_t s_last_seq_ch1 = 0;  // consumer marker
    float vrms; // Desired RMS voltage
};



#endif // CONTROLLER_H