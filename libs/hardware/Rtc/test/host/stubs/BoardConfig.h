#pragma once
#include <cstdint>
namespace BoardConfig {
enum class RtcType { None, Pcf8563, Pcf85063, Ds3231, Rx8130 };
struct Sensors {
  int i2cSda = 47, i2cScl = 48;
  uint32_t i2cHz = 100000;
  uint8_t rtcAddr = 0x32, i2cBus = 0;
  RtcType rtcType = RtcType::Rx8130;
  // What the shared line's Rtc.cpp also reads (a fallback RTC, the low-power flag) and SharedI2c.h (SensorsConfig).
  uint8_t rtcFallbackAddr = 0;
  RtcType rtcFallbackType = RtcType::None;
  bool rtcLowPower = false;
};
using SensorsConfig = Sensors;
struct Profile {
  Sensors sensors;
};
inline Profile ACTIVE;
inline bool rtcIdentityCheck() { return false; }
}  // namespace BoardConfig
