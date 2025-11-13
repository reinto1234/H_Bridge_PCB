#pragma once
#include <Arduino.h>

#define ESTOP_OUTPUT_PIN 19
#define I_Shutdown 8.0f // Shutdown current in Amperes

// Make it visible to all files that include safety.h
extern volatile bool g_emergency_stop;

// (optional) also declare the ISR and pin
void IRAM_ATTR onAlertISR();
constexpr int ALERT_PIN = 21;