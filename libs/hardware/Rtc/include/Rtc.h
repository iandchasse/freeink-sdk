#pragma once

// FreeInk real-time clock (PCF8563).
//
// Reads/sets wall-clock time on the I2C RTC described by
// BoardConfig::ACTIVE.sensors (rtcAddr / sensor bus pins). Dependency-free
// Wire access, mirroring BatteryMonitor. Boards without an RTC (FREEINK_CAP_RTC
// off, or rtcAddr == 0) link stub bodies and present() returns false.

#include <Arduino.h>
#include <BoardConfig.h>

#include <cstdint>

namespace freeink {

class Rtc {
 public:
  // Wall-clock value. `year` is the full year (e.g. 2026); `weekday` is 0=Sunday.
  struct DateTime {
    uint16_t year = 2000;
    uint8_t month = 1;  // 1-12
    uint8_t day = 1;    // 1-31
    uint8_t hour = 0;   // 0-23
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t weekday = 0;  // 0=Sunday .. 6=Saturday
  };

  // Brings up the I2C bus and disables the RTC's CLKOUT. Tries rtcAddr/rtcType,
  // then the profile's rtcFallbackAddr/rtcFallbackType when the first does not
  // answer. Returns false when the active board has no RTC or neither ACKs.
  bool begin();
  bool present() const { return begun_; }
  // The RTC begin() found, for a bring-up screen: its address (0 before begin()
  // or when none answered) and the profile's RtcType for that address.
  uint8_t address() const { return addr_; }
  BoardConfig::RtcType type() const { return static_cast<BoardConfig::RtcType>(type_); }
  // SensorsConfig::rtcIdentityCheck: the address that answered but did not show
  // its RTC's register pattern, so begin() wrote nothing to it (0 = none).
#if FREEINK_CAP_PROBE_IDENTITY
  uint8_t rejectedAddress() const { return rejected_; }
#else
  uint8_t rejectedAddress() const { return 0; }
#endif

  // Reads the current time. Returns false on I2C error or if the RTC reports its
  // oscillator stopped (low voltage / never set) — the time is then unreliable.
  bool now(DateTime& out);

  // Sets the time. Returns false on I2C error.
  bool set(const DateTime& dt);

  // Shifts the running clock by a signed number of seconds, calendar-correct
  // across midnight/month/year boundaries (e.g. a time-zone change from
  // settings: adjust(deltaMinutes * 60)). Reads, shifts, writes back; `out`,
  // when given, receives the new time so callers can refresh UI state in one
  // call. Returns false when the RTC is absent or the read/write fails.
  bool adjust(int32_t seconds, DateTime* out = nullptr);

 private:
  bool begun_ = false;
  uint8_t addr_ = 0;  // resolved by begin(): primary or fallback
  uint8_t type_ = 0;  // BoardConfig::RtcType of addr_
#if FREEINK_CAP_PROBE_IDENTITY
  uint8_t rejected_ = 0;
#endif
};

}  // namespace freeink

using Rtc = freeink::Rtc;
