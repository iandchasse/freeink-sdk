#pragma once

// FreeInk SDK — the pure rules behind the boot-time hardware checks, free of
// Arduino so a host test can run them:
//   - key-ladder presence (PowerConfig::ladderPresenceProbe, InputManager);
//   - the FocalTech touch identity (TouchConfig::requireFocalTechId);
//   - the DS3231 register pattern (SensorsConfig::rtcIdentityCheck, Rtc);
//   - the power pin's idle level (PowerConfig::powerPinIdleCheck, PowerManager).
// Header-only; nothing is emitted unless a caller uses it.

#include <cstdint>

namespace freeink {
namespace probe {

// --- Key-ladder presence -----------------------------------------------------
// The ladder pin is read once with its internal pull-down on, then once with
// the pull-up on (about 45 kOhm each). A fitted ladder pull-up (10 kOhm on
// Silkscreen) holds the idle node near the rail against the pull-down, and a
// held key pulls both readings toward its own level. Only a node with no
// pull-up follows each internal pull from rail to rail.
constexpr uint16_t LADDER_FLOAT_LOW_MAX_MV = 400;
constexpr uint16_t LADDER_FLOAT_HIGH_MIN_MV = 2600;
constexpr bool ladderFloats(const uint16_t pullDownMv, const uint16_t pullUpMv) {
  return pullDownMv <= LADDER_FLOAT_LOW_MAX_MV && pullUpMv >= LADDER_FLOAT_HIGH_MIN_MV;
}

// --- FocalTech touch identity ------------------------------------------------
// ID_G_CIPHER, the chip id at register 0xA3, of the self-capacitance FocalTech
// controllers the FT5x06 path drives: 0x06 FT6206/FT6x06, 0x36 FT6236/FT6336,
// 0x64 FT6336U/FT6236U. The panel id at 0xA8 varies with the module maker, so it
// is not checked.
constexpr uint8_t FT_REG_CIPHER = 0xA3;
constexpr bool focalTechChipId(const uint8_t cipher) { return cipher == 0x06 || cipher == 0x36 || cipher == 0x64; }

// --- DS3231 register pattern -------------------------------------------------
// Registers 0x00-0x12, read in one transfer before anything is written. The
// DS3231 and DS3231M (Maxim 19-5312 Rev 5, Table 2) read these bits as 0 and
// keep the day of week in 1-7, and the time registers hold BCD. Each failed
// check sets one bit of the returned mask; 0 means the device looks like a
// DS3231.
constexpr uint8_t DS3231_REGISTER_COUNT = 0x13;
constexpr uint16_t DS_SECONDS_BITS = 1u << 0;   // 00h bit 7
constexpr uint16_t DS_SECONDS_BCD = 1u << 1;    // 00h not 00-59 BCD
constexpr uint16_t DS_MINUTES_BITS = 1u << 2;   // 01h bit 7
constexpr uint16_t DS_MINUTES_BCD = 1u << 3;    // 01h not 00-59 BCD
constexpr uint16_t DS_HOURS_BITS = 1u << 4;     // 02h bit 7
constexpr uint16_t DS_DAY = 1u << 5;            // 03h bits 7:3, or a day of week outside 1-7
constexpr uint16_t DS_DATE_BITS = 1u << 6;      // 04h bits 7:6
constexpr uint16_t DS_MONTH_BITS = 1u << 7;     // 05h bits 6:5
constexpr uint16_t DS_STATUS_BITS = 1u << 8;    // 0Fh bits 6:4
constexpr uint16_t DS_TEMP_LSB_BITS = 1u << 9;  // 12h bits 5:0

constexpr bool bcdBelow60(const uint8_t v) { return (v & 0x0F) <= 9 && (v >> 4) <= 5; }

constexpr uint16_t ds3231Mismatch(const uint8_t (&regs)[DS3231_REGISTER_COUNT]) {
  uint16_t mismatch = 0;
  if ((regs[0x00] & 0x80) != 0) mismatch |= DS_SECONDS_BITS;
  if (!bcdBelow60(regs[0x00] & 0x7F)) mismatch |= DS_SECONDS_BCD;
  if ((regs[0x01] & 0x80) != 0) mismatch |= DS_MINUTES_BITS;
  if (!bcdBelow60(regs[0x01] & 0x7F)) mismatch |= DS_MINUTES_BCD;
  if ((regs[0x02] & 0x80) != 0) mismatch |= DS_HOURS_BITS;
  if ((regs[0x03] & 0xF8) != 0 || (regs[0x03] & 0x07) == 0) mismatch |= DS_DAY;
  if ((regs[0x04] & 0xC0) != 0) mismatch |= DS_DATE_BITS;
  if ((regs[0x05] & 0x60) != 0) mismatch |= DS_MONTH_BITS;
  if ((regs[0x0F] & 0x70) != 0) mismatch |= DS_STATUS_BITS;
  if ((regs[0x12] & 0x3F) != 0) mismatch |= DS_TEMP_LSB_BITS;
  return mismatch;
}

// --- Power pin idle level ----------------------------------------------------
// ESP32-S3 inputs: a valid low is at most 0.25 x VDD and a valid high at least
// 0.75 x VDD (datasheet v2.2, Table 5-4); in between the input is undefined.
// For a 3.3 V pad:
constexpr uint16_t PIN_LOW_MAX_MV = 825;
constexpr uint16_t PIN_HIGH_MIN_MV = 2475;
enum class PinLevel : uint8_t { Low, Undefined, High };
constexpr PinLevel pinLevel(const uint16_t mv) {
  return mv <= PIN_LOW_MAX_MV ? PinLevel::Low : mv >= PIN_HIGH_MIN_MV ? PinLevel::High : PinLevel::Undefined;
}

}  // namespace probe
}  // namespace freeink
