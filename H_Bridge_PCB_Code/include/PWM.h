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
#define CYCLETIME_PWM 0.2  //PWM-Zykluszeit in ms

// Sinus-Tabelle für SPWM
#define OUTPUT_FREQ 50   // Ausgangsfrequenz in Hz

/*************************************************************************
 * Enums and Structs
 ************************************************************************/
enum ModulationType {
    UNIPOLAR,
    BIPOLAR
};

struct PIController {
    float kp;
    float ki;
    float integral;
    float prevError;
    float outputMin;
    float outputMax;
};

/*************************************************************************
 * Class
 ************************************************************************/
class HBridgeInverter {
public:
    HBridgeInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq);
    void begin();
    float computePI(float setpoint, float measurement);
    void generateSPWM();
    void loop(float vRef, float vMeasured);
    void getmeasurements(float* measurementin, float* measurementout);
    

private:
    int SINE_STEPS;    // Anzahl der Werte für eine Periode
    int S_FREQ; // Startfrequenz in Hz
    std::vector<int> sineTable;
    PIController pi;
    float pwmDutyCycle;
    uint16_t stepIndex;
    ModulationType modulationType;
    void setModulationType(ModulationType type);      // Neu für Webserver
    void setSwitchingFrequency(int freq);            // Neu für Webserver
    float measurementBuffer[10];
};

void startInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq);
void stopInverter();

// Declare global inverter instance so other files can use it
extern HBridgeInverter* inverter;

#endif // PWM_H
