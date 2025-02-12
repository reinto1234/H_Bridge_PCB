#include "PWM.h"
#include "mutexdefinitions.h"

// Global variable for the inverter
HBridgeInverter* inverter = nullptr;


void startInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq) {
    if (inverter == nullptr) {
        if (xSemaphoreTake(inverterMutex, portMAX_DELAY) == pdTRUE) {
        inverter = new HBridgeInverter(kp, ki, outputMin, outputMax, modType, freq);
        inverter->begin();
        Serial.println("H-Bridge Inverter started!");
        xSemaphoreGive(inverterMutex);
        
    }
    } else {
        Serial.println("Inverter is already running!");
    }
}

void stopInverter() {
    
    if (inverter != nullptr) {
        // Stop PWM and reset GPIOs
        
    if (xSemaphoreTake(inverterMutex, portMAX_DELAY) == pdTRUE) {
        delete inverter;
        inverter = nullptr;
        xSemaphoreGive(inverterMutex);
    }
        ledcWrite(PWM_CHANNEL_1, 0);
        ledcWrite(PWM_CHANNEL_2, 0);
        digitalWrite(HIN1, LOW);
        digitalWrite(HIN2, LOW);
        digitalWrite(LIN1, LOW);
        digitalWrite(LIN2, LOW);
        Serial.println("H-Bridge Inverter stopped!");
    } else {
        Serial.println("No inverter is running!");
    }
}

HBridgeInverter::HBridgeInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq)  {
    pi.kp = kp;
    pi.ki = ki;
    pi.integral = 0;
    pi.prevError = 0;
    pi.outputMin = outputMin;
    pi.outputMax = outputMax;
    pwmDutyCycle = 0.5;
    stepIndex = 0;
    modulationType = modType; // Store modulation type correctly
    S_FREQ = freq;
    
    SINE_STEPS = 500.0f / float(OUTPUT_FREQ) / (float(CYCLETIME_PWM));

        
        // Initialisierung des Vektors
    sineTable.resize(SINE_STEPS);
    Serial.println("Sine Table Size: " + String(SINE_STEPS));

    

    // Precompute sine wave table
    for (int i = 0; i < SINE_STEPS; i++) {
        float sinValue = sin(2 * M_PI * i / SINE_STEPS);
        if (modulationType == BIPOLAR) {
            sineTable[i] = (uint16_t)((sinValue + 1) * 511);  // Bipolar: Full sinus range
        } else {
            sineTable[i] = (uint16_t)(abs(sinValue) * 1023); // Unipolar: Only positive part
        }
    }
    
}

void HBridgeInverter::begin() {
    for (int i = 0; i < 10; i++) {
        inverter->measurementBuffer[i] = 0.0f;
    }
    
    // PWM setup
    ledcSetup(PWM_CHANNEL_1, S_FREQ, RESOLUTION);
    ledcSetup(PWM_CHANNEL_2, S_FREQ, RESOLUTION);

    // Attach PWM to GPIOs
    ledcAttachPin(HIN1, PWM_CHANNEL_1);
    ledcAttachPin(HIN2, PWM_CHANNEL_2);

    // Set complementary outputs as digital
    pinMode(LIN1, OUTPUT);
    pinMode(LIN2, OUTPUT);
}

float HBridgeInverter::computePI(float setpoint, float measurement) {
    float error = setpoint - measurement;
    pi.integral += error;

    float output = (pi.kp * error) + (pi.ki * pi.integral);
    if (output > pi.outputMax) output = pi.outputMax;
    if (output < pi.outputMin) output = pi.outputMin;

    return setpoint;
}

void HBridgeInverter::getmeasurements(float* measurementin, float* measurementout) { //noch anpassbar je nachdem welche daten man braucht
    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        measurementBuffer[0] = measurementin[0];
        measurementBuffer[1] = measurementin[1];
        measurementBuffer[2] = measurementin[2];
        xSemaphoreGive(measurementinMutex);
    }
    if (xSemaphoreTake(measurementoutMutex, portMAX_DELAY) == pdTRUE) {
        measurementBuffer[3] = measurementout[0];
        measurementBuffer[4] = measurementout[1];
        measurementBuffer[5] = measurementout[2];
        measurementBuffer[6] = measurementout[3];
        measurementBuffer[7] = measurementout[4];
        measurementBuffer[8] = measurementout[5];
        measurementBuffer[9] = measurementout[6];
        xSemaphoreGive(measurementoutMutex);
    }
}

void HBridgeInverter::generateSPWM() {
    uint16_t pwmValue = sineTable[stepIndex];

    if (modulationType == BIPOLAR) {
        // Bipolar Modulation (both bridge arms switch simultaneously)
        ledcWrite(PWM_CHANNEL_1, pwmValue);
        ledcWrite(PWM_CHANNEL_2, 1023 - pwmValue);

        digitalWrite(LIN1, !digitalRead(HIN1));
        digitalWrite(LIN2, !digitalRead(HIN2));

    } else {  
        // Unipolar Modulation (one bridge arm stays constant)
        if (stepIndex < SINE_STEPS / 2) {
            ledcWrite(PWM_CHANNEL_1, pwmValue);
            ledcWrite(PWM_CHANNEL_2, 0);
            digitalWrite(LIN1, LOW);
            digitalWrite(LIN2, HIGH);
        } else {
            ledcWrite(PWM_CHANNEL_1, 0);
            ledcWrite(PWM_CHANNEL_2, pwmValue);
            digitalWrite(LIN1, HIGH);
            digitalWrite(LIN2, LOW);
        }
    }

    stepIndex = (stepIndex + 1) % SINE_STEPS;
}

void HBridgeInverter::loop(float vRef, float vMeasured) {
    float dutyCycle = computePI(vRef, vMeasured);
    pwmDutyCycle = dutyCycle;
    generateSPWM();
}
