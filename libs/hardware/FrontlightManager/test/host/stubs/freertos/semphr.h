#pragma once
#include "FreeRTOS.h"
using SemaphoreHandle_t = void*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { static int mutex; return &mutex; }
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* storage) { return storage; }
inline int xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
inline int xSemaphoreTakeRecursive(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGiveRecursive(SemaphoreHandle_t) { return pdTRUE; }
