#include "PWM.h"
#include "mutexdefinitions.h"

// Global variable for the inverter
// Ensures only one instance of the inverter is created
HBridgeInverter* inverter = nullptr;

// Function to start the inverter
void startInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq) {
    // Check if an inverter instance already exists
    if (inverter == nullptr) {
        // Acquire mutex before modifying the inverter instance
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

// Function to stop the inverter
void stopInverter() {
    // Check if an inverter instance exists
    if (inverter != nullptr) {
        // Acquire mutex before modifying the inverter instance
        if (xSemaphoreTake(inverterMutex, portMAX_DELAY) == pdTRUE) {
            delete inverter; // Free memory
            inverter = nullptr;
            xSemaphoreGive(inverterMutex);
        }
        // Stop PWM signals and reset GPIOs
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

// Constructor for HBridgeInverter class
HBridgeInverter::HBridgeInverter(float kp, float ki, float outputMin, float outputMax, ModulationType modType, int freq) {
    // Initialize PI controller parameters
    pi.kp = kp;
    pi.ki = ki;
    pi.integral = 0;
    pi.prevError = 0;
    pi.outputMin = outputMin;
    pi.outputMax = outputMax;
    pwmDutyCycle = 0.5;
    stepIndex = 0;
    modulationType = modType; // Store modulation type
    S_FREQ = freq;
    
    // Compute sine table size based on output frequency and cycle time
    SINE_STEPS = 500.0f / float(OUTPUT_FREQ) / (float(CYCLETIME_PWM));

    // Resize sine table
    sineTable.resize(SINE_STEPS);
    Serial.println("Sine Table Size: " + String(SINE_STEPS));

    // Precompute sine wave table based on modulation type
    for (int i = 0; i < SINE_STEPS; i++) {
        float sinValue = sin(2 * M_PI * i / SINE_STEPS);
        if (modulationType == BIPOLAR) {
            sineTable[i] = (uint16_t)((sinValue + 1) * 511);  // Bipolar: Full sinus range
        } else {
            sineTable[i] = (uint16_t)(abs(sinValue) * 1023); // Unipolar: Only positive part
        }
    }
}

// Initialize the inverter
void HBridgeInverter::begin() {
    // Initialize measurement buffer
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

// Compute PI controller output
float HBridgeInverter::computePI(float setpoint, float measurement) {
    float error = setpoint - measurement;
    pi.integral += error;

    float output = (pi.kp * error) + (pi.ki * pi.integral);
    if (output > pi.outputMax) output = pi.outputMax;
    if (output < pi.outputMin) output = pi.outputMin;

    return setpoint; // Doesnt return output because PI Controller is not implemented correctly yet
}

// Retrieve measurement data
void HBridgeInverter::getmeasurements(float* measurementin, float* measurementout) {
    // Store input measurements
    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        measurementBuffer[0] = measurementin[0];
        measurementBuffer[1] = measurementin[1];
        measurementBuffer[2] = measurementin[2];
        xSemaphoreGive(measurementinMutex);
    }
    // Store output measurements
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

// Generate sinusoidal PWM
void HBridgeInverter::generateSPWM() {
    uint16_t pwmValue = sineTable[stepIndex];

    if (modulationType == BIPOLAR) {
        // Bipolar modulation: both bridge arms switch simultaneously
        ledcWrite(PWM_CHANNEL_1, pwmValue);
        ledcWrite(PWM_CHANNEL_2, 1023 - pwmValue);

        digitalWrite(LIN1, !digitalRead(HIN1));
        digitalWrite(LIN2, !digitalRead(HIN2));
    } else {
        // Unipolar modulation: one bridge arm stays constant
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
    // Increment sine wave step
    stepIndex = (stepIndex + 1) % SINE_STEPS;
}

// Main control loop
void HBridgeInverter::loop(float vRef, float vMeasured) {
    float dutyCycle = computePI(vRef, vMeasured);
    pwmDutyCycle = dutyCycle;
    generateSPWM();
}