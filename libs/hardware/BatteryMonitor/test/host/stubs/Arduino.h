#pragma once
#include <cstdint>
constexpr int HIGH = 1, LOW = 0, INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5, INPUT_PULLDOWN = 9;
inline unsigned long hostMillis = 0;  // the test's clock; delay() advances it
inline unsigned long millis() { return hostMillis; }
inline void delay(unsigned long ms) { hostMillis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int hostDigitalPin = 0;  // the test's detect-pin level
inline int digitalRead(int) { return hostDigitalPin; }
inline uint32_t hostAdcMv = 0;  // the test's battery pin reading
inline uint32_t analogReadMilliVolts(int) { return hostAdcMv; }
struct SerialStub {
  explicit operator bool() const { return false; }
  template <typename... Args>
  void printf(const char*, Args...) {}
};
inline SerialStub Serial;
// SharedAdc.h's raw read and attenuation helpers.
enum adc_attenuation_t { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };
inline uint16_t analogRead(int) { return 0; }
inline void analogSetAttenuation(adc_attenuation_t) {}
