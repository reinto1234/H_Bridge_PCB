/*************************************************************************
 * @file PWM.h
 * @date 2025/01/31
 *
 ************************************************************************/

#ifndef PWM_H
#define PWM_H

/*************************************************************************
 * Includes
 ************************************************************************/
#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "Controller.h"

/*************************************************************************
 * Defines
 ************************************************************************/
// GPIO-Pins für H-Brücke
#define HIN1 15
#define LIN1 16
#define HIN2 13
#define LIN2 14

// PWM-Konfiguration
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define RESOLUTION 10  // 10-Bit PWM
//#define CYCLETIME_PWM 0.5  //PWM-Zykluszeit in ms

// Sinus-Tabelle für SPWM
#define OUTPUT_FREQ 50   // Ausgangsfrequenz in Hz
#define SINE_STEPS 200 // Anzahl der Schritte in der Sinus-Tabelle


#define TIMER_INTERVAL_US (1000000 / (OUTPUT_FREQ * SINE_STEPS))
#define S_FREQ 35000    // Schaltfrequenz in Hz (35 kHz)




/*************************************************************************
 * Class
 ************************************************************************/
class HBridgeInverter {
public:
    HBridgeInverter(float vrms);
    void begin();
    void generateSPWM();
    PIController controller; // Example gains and output limits

private:
    
    int16_t sineTable[SINE_STEPS];
    float pwmDutyCycle;
    uint16_t stepIndex;
    float measurementBuffer[10];
    intr_handle_t spwmIntrHandle;
    
};

void startInverter(float vrms);
void stopInverter();

// Declare global inverter instance so other files can use it
extern HBridgeInverter* inverter;

#endif // PWM_H
