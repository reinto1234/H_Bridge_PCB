/************************************************************************
 * @file mutexdefinitions.cpp
 * @brief Mutex definitions for the H-Bridge Inverter System
 *
 * This file contains the definitions of mutexes used for synchronizing
 * access to shared resources in the H-Bridge Inverter System.
 ************************************************************************/

#include "mutexdefinitions.h"

SemaphoreHandle_t inverterMutex = NULL;
SemaphoreHandle_t measurementinMutex = NULL;
SemaphoreHandle_t measurementoutMutex = NULL;
SemaphoreHandle_t measurementSpiMutex = NULL;
