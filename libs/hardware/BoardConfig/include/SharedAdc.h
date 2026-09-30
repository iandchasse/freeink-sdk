#pragma once

// FreeInk SDK — one lock for every SAR ADC reader.
//
// Arduino-ESP32 3.x analogRead()/analogReadMilliVolts() only TRY-lock the ADC
// unit (adc_oneshot_read), so a reader that loses a race gets a silent 0, and
// the lazy per-pin and calibration setup inside them is not thread-safe. The
// SDK's ADC readers (button ladders, battery divider, charge-status ladder,
// frontlight LED monitor) run on different tasks and cores, so every
// conversion goes through this recursive mutex. A consumer that reads the ADC
// itself should use these helpers too.

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace freeink {
namespace adc {

inline SemaphoreHandle_t mutex() {
  // A function-local static is initialised once, thread-safely, and the static
  // buffer keeps the mutex off the heap.
  static StaticSemaphore_t storage;
  static const SemaphoreHandle_t handle = xSemaphoreCreateRecursiveMutexStatic(&storage);
  return handle;
}

// Holds the ADC for its scope. Recursive, so a helper below may run inside a
// held Lock. With a bounded `wait`, check held() before reading.
class Lock {
 public:
  explicit Lock(TickType_t wait = portMAX_DELAY) : held_(xSemaphoreTakeRecursive(mutex(), wait) == pdTRUE) {}
  ~Lock() {
    if (held_) xSemaphoreGiveRecursive(mutex());
  }
  Lock(const Lock&) = delete;
  Lock& operator=(const Lock&) = delete;
  bool held() const { return held_; }

 private:
  bool held_;
};

inline uint32_t readMilliVolts(uint8_t pin) {
  const Lock lock;
  return analogReadMilliVolts(pin);
}

inline uint16_t readRaw(uint8_t pin) {
  const Lock lock;
  return analogRead(pin);
}

inline void setAttenuation(adc_attenuation_t attenuation) {
  const Lock lock;
  analogSetAttenuation(attenuation);
}

}  // namespace adc
}  // namespace freeink
