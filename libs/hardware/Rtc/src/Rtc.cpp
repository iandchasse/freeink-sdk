#include "Rtc.h"

#include <time.h>

#include <BoardConfig.h>

#if FREEINK_CAP_RTC

#include <SharedI2c.h>
#include <Wire.h>
#include <soc/soc_caps.h>
#if FREEINK_CAP_PROBE_IDENTITY
#include <ProbeChecks.h>
#endif

namespace freeink {
namespace {

// PCF8563 register map (confirmed against the vendor peripheral demo).
constexpr uint8_t PCF8563_REG_CONTROL_STATUS1 = 0x00;
constexpr uint8_t PCF8563_REG_TIME = 0x02;  // seconds, minutes, hours, days, weekdays, months, years
constexpr uint8_t PCF8563_REG_CLKOUT = 0x0D;
constexpr uint8_t PCF8563_CLKOUT_DISABLED = 0x00;
constexpr uint8_t PCF8563_VL_FLAG = 0x80;  // seconds reg bit7: oscillator stopped / voltage-low

// PCF85063A register map. Same BCD time layout as the PCF8563 but shifted:
// Control_1/2 at 0x00/0x01, time from 0x04, seconds bit7 is the OS (oscillator
// stopped) flag, and Months carries no century bit. Micro Crystal's RV-8263-C7
// shares the map. Control_2 bits 2:0 select the CLKOUT frequency; 111 = off.
constexpr uint8_t PCF85063_REG_CONTROL1 = 0x00;
constexpr uint8_t PCF85063_REG_CONTROL2 = 0x01;
constexpr uint8_t PCF85063_CLKOUT_MASK = 0x07;
constexpr uint8_t PCF85063_REG_TIME = 0x04;
constexpr uint8_t PCF85063_OS_FLAG = 0x80;

// DS3231 register map.
constexpr uint8_t DS3231_REG_TIME = 0x00;  // seconds, minutes, hours, day, date, month, year
constexpr uint8_t DS3231_REG_CONTROL = 0x0E;
constexpr uint8_t DS3231_REG_STATUS = 0x0F;
constexpr uint8_t DS3231_CONTROL_INTCN = 0x04;
constexpr uint8_t DS3231_STATUS_OSF = 0x80;
constexpr uint8_t DS3231_STATUS_EN32KHZ = 0x08;

// RX8130CE register map (Paper Mono / PaperS3).
constexpr uint8_t RX8130_REG_TIME = 0x10;
constexpr uint8_t RX8130_REG_CONTROL0 = 0x1D;
constexpr uint8_t RX8130_POR_FLAG = 0x80;
constexpr uint8_t RX8130_STOP = 0x01;

bool g_wireReady[2] = {false, false};
TwoWire& sensorWire() {
  const auto& s = BoardConfig::ACTIVE.sensors;
#if SOC_I2C_NUM > 1
  return s.i2cBus == 1 ? Wire1 : Wire;
#else
  return Wire;
#endif
}

void ensureWire() {
  const auto& s = BoardConfig::ACTIVE.sensors;
  const uint8_t bus =
#if SOC_I2C_NUM > 1
      s.i2cBus == 1 ? 1 : 0;
#else
      0;
#endif
  if (g_wireReady[bus]) return;
  auto& wire = sensorWire();
  freeink::i2c::beginBus(wire, s.i2cSda, s.i2cScl, s.i2cHz);
  g_wireReady[bus] = true;
}

uint8_t bcdToDec(uint8_t v) { return static_cast<uint8_t>((v >> 4) * 10U + (v & 0x0FU)); }
uint8_t decToBcd(uint8_t v) { return static_cast<uint8_t>((v / 10U) << 4 | (v % 10U)); }

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* dst, uint8_t len) {
  ensureWire();
  auto& wire = sensorWire();
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom(addr, len, static_cast<uint8_t>(true)) < len) return false;
  for (uint8_t i = 0; i < len; ++i) dst[i] = wire.read();
  return true;
}

#if FREEINK_CAP_PROBE_IDENTITY
// SensorsConfig::rtcIdentityCheck: the address this begin() refused (0 = none).
uint8_t g_rejectedAddr = 0;
// The first access after a DS3231's first power-up resets its time registers,
// so a mismatch is read once more after this long.
constexpr uint32_t DS3231_RECHECK_MS = 20;

// The DS3231 status register. With rtcIdentityCheck, the whole register file is
// read first, and a device without the DS3231 pattern (ProbeChecks.h) is refused
// before anything is written to it.
bool readDs3231Status(const uint8_t addr, uint8_t& status) {
  if (!BoardConfig::rtcIdentityCheck()) return readRegs(addr, DS3231_REG_STATUS, &status, 1);
  uint8_t regs[probe::DS3231_REGISTER_COUNT] = {};
  if (!readRegs(addr, 0x00, regs, sizeof(regs))) return false;
  if (probe::ds3231Mismatch(regs) != 0) {
    delay(DS3231_RECHECK_MS);
    if (!readRegs(addr, 0x00, regs, sizeof(regs))) return false;
    if (probe::ds3231Mismatch(regs) != 0) {
      g_rejectedAddr = addr;
      return false;
    }
  }
  status = regs[DS3231_REG_STATUS];
  return true;
}
#endif

// Probe and initialise one RTC. Returns false when it does not ACK (or, with
// SensorsConfig::rtcIdentityCheck, is not the part named).
bool beginDevice(const uint8_t addr, const BoardConfig::RtcType type) {
  if (addr == 0) return false;
  const bool lowPower = BoardConfig::ACTIVE.sensors.rtcLowPower;
  uint8_t status = 0;
  switch (type) {
    case BoardConfig::RtcType::Pcf8563:
      if (!readRegs(addr, PCF8563_REG_CONTROL_STATUS1, &status, 1)) return false;
      writeReg(addr, PCF8563_REG_CLKOUT, PCF8563_CLKOUT_DISABLED);  // we don't use the 32 kHz CLKOUT
      return true;
    case BoardConfig::RtcType::Pcf85063:
      // CLKOUT lives in Control_2 alongside alarm/timer bits, so it is only
      // switched off (read-modify-write) on boards that ask for it: a PCF85063A
      // or RV-8263-C7 with its CLKOUT enabled runs 32.768 kHz out of the pin.
      if (!readRegs(addr, PCF85063_REG_CONTROL1, &status, 1)) return false;
      if (lowPower) {
        uint8_t control2 = 0;
        if (readRegs(addr, PCF85063_REG_CONTROL2, &control2, 1)) {
          writeReg(addr, PCF85063_REG_CONTROL2, static_cast<uint8_t>(control2 | PCF85063_CLKOUT_MASK));
        }
      }
      return true;
    case BoardConfig::RtcType::Ds3231:
#if FREEINK_CAP_PROBE_IDENTITY
      if (!readDs3231Status(addr, status)) return false;
#else
      if (!readRegs(addr, DS3231_REG_STATUS, &status, 1)) return false;
#endif
      writeReg(addr, DS3231_REG_CONTROL, DS3231_CONTROL_INTCN);  // disable square-wave output
      if (lowPower && (status & DS3231_STATUS_EN32KHZ)) {
        // EN32KHZ powers up set; the VBAT timekeeping current assumes it clear.
        // OSF is written back as read: writing 1 leaves it unchanged, and
        // clearing it here would hide a lost time.
        writeReg(addr, DS3231_REG_STATUS, static_cast<uint8_t>(status & ~DS3231_STATUS_EN32KHZ));
      }
      return true;
    case BoardConfig::RtcType::Rx8130:
      return readRegs(addr, RX8130_REG_CONTROL0, &status, 1);
    case BoardConfig::RtcType::None:
      return false;
  }
  return false;
}

}  // namespace

bool Rtc::begin() {
  const auto& s = BoardConfig::ACTIVE.sensors;
  if (s.rtcAddr == 0) return false;
  if (s.i2cSda < 0 || s.i2cScl < 0 || s.i2cHz == 0) return false;
  ensureWire();
#if FREEINK_CAP_PROBE_IDENTITY
  // rejectedAddress() is this begin()'s, on every return below.
  g_rejectedAddr = 0;
  struct NoteRejected {
    Rtc& rtc;
    ~NoteRejected() { rtc.rejected_ = g_rejectedAddr; }
  } noteRejected{*this};
#endif
  if (beginDevice(s.rtcAddr, s.rtcType)) {
    addr_ = s.rtcAddr;
    type_ = static_cast<uint8_t>(s.rtcType);
  } else if (s.rtcFallbackType != BoardConfig::RtcType::None && beginDevice(s.rtcFallbackAddr, s.rtcFallbackType)) {
    addr_ = s.rtcFallbackAddr;
    type_ = static_cast<uint8_t>(s.rtcFallbackType);
  } else {
    return false;
  }
  begun_ = true;
  return true;
}

bool Rtc::now(DateTime& out) {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  const auto rtcType = static_cast<BoardConfig::RtcType>(type_);
  uint8_t raw[7] = {};
  switch (rtcType) {
    case BoardConfig::RtcType::Pcf8563: {
      if (!readRegs(addr, PCF8563_REG_TIME, raw, sizeof(raw))) return false;
      if (raw[0] & PCF8563_VL_FLAG) return false;  // oscillator stopped -> time not trustworthy
      out.second = bcdToDec(raw[0] & 0x7FU);
      out.minute = bcdToDec(raw[1] & 0x7FU);
      out.hour = bcdToDec(raw[2] & 0x3FU);
      out.day = bcdToDec(raw[3] & 0x3FU);
      out.weekday = bcdToDec(raw[4] & 0x07U);
      out.month = bcdToDec(raw[5] & 0x1FU);
      const uint8_t yy = bcdToDec(raw[6]);
      out.year = (raw[5] & 0x80U) ? static_cast<uint16_t>(1900 + yy) : static_cast<uint16_t>(2000 + yy);
      return true;
    }
    case BoardConfig::RtcType::Pcf85063: {
      if (!readRegs(addr, PCF85063_REG_TIME, raw, sizeof(raw))) return false;
      if (raw[0] & PCF85063_OS_FLAG) return false;  // oscillator stopped -> time not trustworthy
      out.second = bcdToDec(raw[0] & 0x7FU);
      out.minute = bcdToDec(raw[1] & 0x7FU);
      out.hour = bcdToDec(raw[2] & 0x3FU);
      out.day = bcdToDec(raw[3] & 0x3FU);
      out.weekday = bcdToDec(raw[4] & 0x07U);
      out.month = bcdToDec(raw[5] & 0x1FU);
      out.year = static_cast<uint16_t>(2000 + bcdToDec(raw[6]));
      return true;
    }
    case BoardConfig::RtcType::Ds3231: {
      uint8_t status = 0;
      if (!readRegs(addr, DS3231_REG_STATUS, &status, 1) || (status & DS3231_STATUS_OSF)) return false;
      if (!readRegs(addr, DS3231_REG_TIME, raw, sizeof(raw))) return false;
      out.second = bcdToDec(raw[0] & 0x7FU);
      out.minute = bcdToDec(raw[1] & 0x7FU);
      if (raw[2] & 0x40U) {
        uint8_t h12 = bcdToDec(raw[2] & 0x1FU);
        if (h12 == 12) h12 = 0;
        out.hour = static_cast<uint8_t>((raw[2] & 0x20U) ? h12 + 12 : h12);
      } else {
        out.hour = bcdToDec(raw[2] & 0x3FU);
      }
      out.weekday = bcdToDec(raw[3] & 0x07U) % 7U;
      out.day = bcdToDec(raw[4] & 0x3FU);
      out.month = bcdToDec(raw[5] & 0x1FU);
      out.year = static_cast<uint16_t>(2000 + bcdToDec(raw[6]));
      return true;
    }
    case BoardConfig::RtcType::Rx8130: {
      if (!readRegs(addr, RX8130_REG_TIME, raw, sizeof(raw))) return false;
      if (raw[0] & RX8130_POR_FLAG) return false;
      out.second = bcdToDec(raw[0] & 0x7FU);
      out.minute = bcdToDec(raw[1] & 0x7FU);
      out.hour = bcdToDec(raw[2] & 0x3FU);
      out.weekday = 0;
      for (uint8_t i = 0; i < 7; ++i) {
        if (raw[3] & (1u << i)) {
          out.weekday = i;
          break;
        }
      }
      out.day = bcdToDec(raw[4] & 0x3FU);
      out.month = bcdToDec(raw[5] & 0x1FU);
      out.year = static_cast<uint16_t>(2000 + bcdToDec(raw[6]));
      return true;
    }
    case BoardConfig::RtcType::None:
      return false;
  }
  return false;
}

bool Rtc::set(const DateTime& dt) {
  const uint8_t addr = addr_;
  if (!begun_ || addr == 0) return false;
  const auto rtcType = static_cast<BoardConfig::RtcType>(type_);
  const uint8_t centuryBit = dt.year < 2000 ? 0x80U : 0x00U;
  ensureWire();
  auto& wire = sensorWire();
  if (rtcType == BoardConfig::RtcType::None) return false;
  if (rtcType == BoardConfig::RtcType::Rx8130) {
    uint8_t control = 0;
    if (!readRegs(addr, RX8130_REG_CONTROL0, &control, 1) ||
        !writeReg(addr, RX8130_REG_CONTROL0, static_cast<uint8_t>(control | RX8130_STOP))) {
      return false;
    }
    wire.beginTransmission(addr);
    wire.write(RX8130_REG_TIME);
    wire.write(decToBcd(dt.second));
    wire.write(decToBcd(dt.minute));
    wire.write(decToBcd(dt.hour));
    wire.write(static_cast<uint8_t>(1u << (dt.weekday % 7u)));
    wire.write(decToBcd(dt.day));
    wire.write(decToBcd(dt.month));
    wire.write(decToBcd(static_cast<uint8_t>(dt.year % 100)));
    const bool written = wire.endTransmission() == 0;
    const bool restarted = writeReg(addr, RX8130_REG_CONTROL0, static_cast<uint8_t>(control & ~RX8130_STOP));
    return written && restarted;
  }
  if (rtcType == BoardConfig::RtcType::Pcf85063) {
    wire.beginTransmission(addr);
    wire.write(PCF85063_REG_TIME);
    wire.write(decToBcd(dt.second));  // also clears OS once a valid time is written
    wire.write(decToBcd(dt.minute));
    wire.write(decToBcd(dt.hour));
    wire.write(decToBcd(dt.day));
    wire.write(decToBcd(dt.weekday));
    wire.write(decToBcd(dt.month));
    wire.write(decToBcd(static_cast<uint8_t>(dt.year % 100)));
    return wire.endTransmission() == 0;
  }
  wire.beginTransmission(addr);
  wire.write(rtcType == BoardConfig::RtcType::Pcf8563 ? PCF8563_REG_TIME : DS3231_REG_TIME);
  wire.write(decToBcd(dt.second));  // also clears VL once a valid time is written
  wire.write(decToBcd(dt.minute));
  wire.write(decToBcd(dt.hour));
  if (rtcType == BoardConfig::RtcType::Pcf8563) {
    wire.write(decToBcd(dt.day));
    wire.write(decToBcd(dt.weekday));
    wire.write(static_cast<uint8_t>(decToBcd(dt.month) | centuryBit));
  } else {
    wire.write(decToBcd(dt.weekday == 0 ? 7 : dt.weekday));
    wire.write(decToBcd(dt.day));
    wire.write(decToBcd(dt.month));
  }
  wire.write(decToBcd(static_cast<uint8_t>(dt.year % 100)));
  if (wire.endTransmission() != 0) return false;
  if (rtcType == BoardConfig::RtcType::Ds3231) {
    uint8_t status = 0;
    if (readRegs(addr, DS3231_REG_STATUS, &status, 1)) {
      uint8_t cleared = static_cast<uint8_t>(status & ~DS3231_STATUS_OSF);
      if (BoardConfig::ACTIVE.sensors.rtcLowPower) cleared = static_cast<uint8_t>(cleared & ~DS3231_STATUS_EN32KHZ);
      writeReg(addr, DS3231_REG_STATUS, cleared);
    }
  }
  return true;
}

bool Rtc::adjust(const int32_t seconds, DateTime* out) {
  DateTime now{};
  if (!this->now(now)) return false;
  // Round-trip through mktime so day/month/year carries are calendar-correct
  // (mktime normalizes out-of-range fields; the fixed offset cancels).
  struct tm t{};
  t.tm_year = now.year - 1900;
  t.tm_mon = now.month - 1;
  t.tm_mday = now.day;
  t.tm_hour = now.hour;
  t.tm_min = now.minute;
  t.tm_sec = now.second;
  time_t epoch = mktime(&t) + seconds;
  localtime_r(&epoch, &t);
  const DateTime dt{static_cast<uint16_t>(t.tm_year + 1900), static_cast<uint8_t>(t.tm_mon + 1),
                    static_cast<uint8_t>(t.tm_mday),         static_cast<uint8_t>(t.tm_hour),
                    static_cast<uint8_t>(t.tm_min),          static_cast<uint8_t>(t.tm_sec),
                    static_cast<uint8_t>(t.tm_wday)};
  if (!set(dt)) return false;
  if (out) *out = dt;
  return true;
}

}  // namespace freeink

#else  // FREEINK_CAP_RTC — no RTC on this board.

namespace freeink {
bool Rtc::begin() { return false; }
bool Rtc::now(DateTime&) { return false; }
bool Rtc::set(const DateTime&) { return false; }
bool Rtc::adjust(int32_t, DateTime*) { return false; }
}  // namespace freeink

#endif  // FREEINK_CAP_RTC
