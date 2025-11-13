#pragma once
#include <Arduino.h>

// Needs access to sampler context & OutputMeasurements packs/globals
#include "spi_sampler.h"

// Task entry points
void spiSamplerTask(void* arg);   // arg = &g_pack1 or &g_pack2
void analyzerTask(void* arg);     // arg = &g_ch1_om or &g_ch2_om
void printerTask(void* arg);      // optional diagnostics; arg ignored
void webSocketTask(void* arg);    // arg ignored
void webSocketUpdate(void* arg);  // arg ignored
void EmergencyStopTask(void* arg);
void ControllerTask(void* pvParameters);
