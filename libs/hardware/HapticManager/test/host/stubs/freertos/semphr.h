#pragma once
#include <mutex>
#include <freertos/FreeRTOS.h>
using SemaphoreHandle_t = std::mutex*;
constexpr unsigned portMAX_DELAY = ~0u;
inline bool failMutex = false;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return failMutex ? nullptr : new std::mutex; }
inline int xSemaphoreTake(SemaphoreHandle_t mutex, unsigned) { mutex->lock(); return pdTRUE; }
inline void xSemaphoreGive(SemaphoreHandle_t mutex) { mutex->unlock(); }
// The SDK's shared ADC lock (SharedAdc.h): a recursive mutex, single-threaded here.
using TickType_t = unsigned;
struct StaticSemaphore_t {};
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t*) {
  static std::mutex m;
  return &m;
}
inline int xSemaphoreTakeRecursive(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGiveRecursive(SemaphoreHandle_t) { return pdTRUE; }
