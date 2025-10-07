/************************************************************************
 * @file Controller.cpp
 * @brief Controller implementation for the controller module
 *
 * This file contains the implementation of the controller, including
 * the PI controller logic.
 ************************************************************************/
#include "Controller.h"
#include <math.h>
#include "mutexdefinitions.h" // Mutex-Definitionen einbinden

// Define the static variable with DRAM_ATTR at file scope
DRAM_ATTR int16_t PIController::g_amp_q10 = 307;   //initial value 0,3 maximum 1023: 1,0
// Place this at the top of the file, after includes, if not already present:
// DRAM_ATTR uint32_t Controller::g_amp_q10 = 1023;

// Constructor
PIController::PIController(float kp, float ki, float outputMin, float outputMax, float vrms)
    : kp(kp), ki(ki), outputMin(outputMin), outputMax(outputMax), integral(0.0f), prevError(0.0f), vrms(vrms) {
    // Initialization of g_amp_q10 should be done at declaration, not here
}

// Control RMS voltage
void PIController::ControlRMS(float* RMSvoltage) {
    // Mutex für Thread-Sicherheit verwenden
    if (RMSvoltage != nullptr && !std::isnan(*RMSvoltage)) {
        float factortemp = compute(vrms, *RMSvoltage); // Example usage
        //Serial.println(*RMSvoltage);
        //Serial.println(factortemp);
        int16_t temp = factortemp * 1023; // Store the computed factor in Q10 format
        __atomic_store_n(&g_amp_q10, temp, __ATOMIC_RELEASE); // Atomically update g_amp_q10
    }
    else{
        __atomic_store_n(&g_amp_q10, 1023*outputMin, __ATOMIC_RELEASE); // Reset to default factor
    }
   //g_amp_q10 = 1023*outputMax; // Always set to max for now
}

// Compute PI controller output
float PIController::compute(float setpoint, float measurement) {
    float error = setpoint - measurement;
    float inttemp = integral;
    integral += ki*error;

    float output = (kp * error) + (integral);
    //Serial.print("PI output: ");
    //Serial.println(output);
    if (output > outputMax) {
        output = outputMax;
        if (integral > inttemp) integral = inttemp; // Anti-windup
    }
    if (output < outputMin) {
        output = outputMin;
        if (integral < inttemp) integral = inttemp; // Anti-windup
    }
    //Serial.print("Adjusted PI output: ");
    //Serial.println(output);

    return output;
}



