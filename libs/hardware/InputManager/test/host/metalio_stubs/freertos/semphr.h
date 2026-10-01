#pragma once
#include "FreeRTOS.h"
// The SDK's shared ADC lock (SharedAdc.h): a recursive mutex, single-threaded here.
using TickType_t = unsigned;
using SemaphoreHandle_t = void*;
struct StaticSemaphore_t {};
constexpr TickType_t portMAX_DELAY = ~0u;
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* s) { return s; }
inline int xSemaphoreTakeRecursive(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGiveRecursive(SemaphoreHandle_t) { return pdTRUE; }
