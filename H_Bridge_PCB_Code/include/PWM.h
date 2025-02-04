#ifndef PWM_H
#define PWM_H

#include <Arduino.h>
#include <math.h>

// GPIO-Pins für H-Brücke
#define HIN1 15
#define LIN1 16
#define HIN2 13
#define LIN2 14

// PWM-Konfiguration
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define RESOLUTION 10  // 10-Bit PWM
#define BASE_FREQ 30000 // Startfrequenz in Hz

// Sinus-Tabelle für SPWM

#define OUTPUT_FREQ 50   // Ausgangsfrequenz in Hz
#define SINE_STEPS int(BASE_FREQ/OUTPUT_FREQ)   // Anzahl der Werte für eine Periode

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

// H-Brücke Klasse
class HBridgeInverter {
public:
    HBridgeInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType);
    void begin();
    void setFrequency(float freq);
    float computePI(float setpoint, float measurement);
    void generateSPWM();
    void loop(float vRef, float vMeasured);
    

private:
    PIController pi;
    float pwmDutyCycle;
    uint16_t sineTable[SINE_STEPS];
    uint16_t stepIndex;
    ModulationType modulationType;
    void setModulationType(ModulationType type);      // Neu für Webserver
    void setSwitchingFrequency(int freq);            // Neu für Webserver
};

void startInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, float freq);
void stopInverter();

// Declare global inverter instance so other files can use it
extern HBridgeInverter* inverter;

#endif // PWM_H
