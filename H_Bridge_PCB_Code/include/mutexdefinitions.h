/*************************************************************************
 * @file mutexdefinitions.h
 * @date 2025/01/31
 *
 ************************************************************************/

#ifndef MUTEX_DEFINITIONS_H
#define MUTEX_DEFINITIONS_H

/*************************************************************************
 * Includes
 ************************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Deklariere den Mutex als extern, damit er in allen Dateien verfügbar ist.
extern SemaphoreHandle_t inverterMutex;
extern SemaphoreHandle_t measurementinMutex;
extern SemaphoreHandle_t measurementoutMutex;

#endif // MUTEX_DEFINITIONS_H
