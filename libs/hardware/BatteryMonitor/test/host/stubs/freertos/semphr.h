#pragma once
#include <freertos/FreeRTOS.h>
struct StaticSemaphore_t {};
using SemaphoreHandle_t = StaticSemaphore_t*;
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* storage) { return storage; }
inline int xSemaphoreTakeRecursive(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGiveRecursive(SemaphoreHandle_t) { return pdTRUE; }
