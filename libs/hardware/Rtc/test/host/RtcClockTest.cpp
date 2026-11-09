#include <Arduino.h>
#include <BoardConfig.h>
#include <Rtc.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <vector>

namespace {
std::array<std::array<uint8_t, 256>, 128> registers{};
bool onlyPcf = false, dropRestart = false, stoppedOscillator = false;
int operations = 0, failAt = 0;
constexpr uint8_t PCF = 0x51;
constexpr uint8_t STOP = 0x20, TEST = 0x80;

bool failed(uint8_t addr) { return ++operations == failAt || (onlyPcf && addr != PCF); }
uint8_t bcd(unsigned value) { return static_cast<uint8_t>((value / 10) * 16 + value % 10); }
unsigned decimal(uint8_t value) { return (value >> 4) * 10 + (value & 15); }

void reset(bool fallback = false) {
  registers = {};
  onlyPcf = fallback;
  dropRestart = stoppedOscillator = false;
  operations = failAt = 0;
  auto& sensors = BoardConfig::ACTIVE.sensors;
  sensors = BoardConfig::Sensors{};
  sensors.rtcType = fallback ? BoardConfig::RtcType::Ds3231 : BoardConfig::RtcType::Pcf85063;
  sensors.rtcAddr = fallback ? 0x68 : PCF;
  sensors.rtcFallbackType = fallback ? BoardConfig::RtcType::Pcf85063 : BoardConfig::RtcType::None;
  sensors.rtcFallbackAddr = fallback ? PCF : 0;
  sensors.rtcLowPower = fallback;
}

// RTC time advances without the MCU clock, unless its divider/crystal is stopped.
void elapse(unsigned seconds) {
  if (stoppedOscillator || (registers[PCF][0] & (STOP | TEST))) return;
  auto& r = registers[PCF];
  const unsigned elapsed = decimal(r[6]) * 3600 + decimal(r[5]) * 60 + decimal(r[4] & 0x7f) + seconds;
  assert(elapsed < 86400);  // These tests stay within one calendar day.
  r[4] = bcd(elapsed % 60);
  r[5] = bcd(elapsed / 60 % 60);
  r[6] = bcd(elapsed / 3600);
}
}  // namespace

bool hostI2cWrite(uint8_t addr, const std::vector<uint8_t>& bytes) {
  if (failed(addr)) return false;
  if (bytes.empty()) return true;
  for (size_t i = 1; i < bytes.size(); ++i) {
    const uint8_t reg = static_cast<uint8_t>(bytes[0] + i - 1);
    const uint8_t value = bytes[i];
    if (addr == PCF && reg == 0 && dropRestart && !(value & STOP)) continue;
    registers[addr][reg] = value;
    if (addr == PCF && reg == 4 && stoppedOscillator) registers[addr][reg] |= 0x80;
  }
  return true;
}

bool hostI2cRead(uint8_t addr, uint8_t reg, uint8_t* out, size_t length) {
  if (failed(addr)) return false;
  for (size_t i = 0; i < length; ++i) out[i] = registers[addr][reg + i];
  return true;
}

int main() {
  const Rtc::DateTime expected{2026, 10, 31, 12, 34, 56, 6};
  Rtc::DateTime actual;
  for (bool fallback : {false, true}) {
    reset(fallback);
    registers[PCF][0] = STOP | TEST | 0x06;  // Stopped, external test, 12-hour mode, CIE.
    registers[PCF][1] = 0xa8;
    registers[PCF][2] = 0x37;  // Existing offset calibration and RAM must survive.
    registers[PCF][3] = 0x59;
    Rtc rtc;
    assert(rtc.begin() && rtc.address() == PCF && rtc.type() == BoardConfig::RtcType::Pcf85063);
    assert(rtc.set(expected));
    assert(registers[PCF][0] == 0x04);
    assert(registers[PCF][1] == (fallback ? 0xaf : 0xa8));
    assert(registers[PCF][2] == 0x37 && registers[PCF][3] == 0x59);
    assert(rtc.now(actual) && actual.year == 2026 && actual.month == 10 && actual.day == 31);
    assert(actual.hour == 12 && actual.minute == 34 && actual.second == 56 && actual.weekday == 6);
    elapse(5);
    assert(rtc.now(actual) && actual.minute == 35 && actual.second == 1);
    elapse(600);  // No MCU timer or polling during this simulated sleep.
    Rtc afterWake;
    assert(afterWake.begin() && afterWake.now(actual));
    assert(actual.hour == 12 && actual.minute == 45 && actual.second == 1);
    assert(registers[PCF][2] == 0x37 && registers[PCF][3] == 0x59);
    for (uint8_t invalid : {STOP, TEST, uint8_t(0x02)}) {
      registers[PCF][0] = invalid;
      assert(!rtc.now(actual));
    }
    registers[PCF][0] = 0;
    registers[PCF][4] |= 0x80;
    assert(!rtc.now(actual));
  }

  reset();
  registers[PCF][0] = 0x05;  // Genuine PCF85063 load capacitance and CIE are preserved.
  Rtc rtc;
  assert(rtc.begin() && rtc.set(expected) && registers[PCF][0] == 0x05);
  operations = 0;
  assert(rtc.set(expected));
  const int setOperations = operations;
  for (int failure = 1; failure <= setOperations; ++failure) {
    reset();
    operations = 0;
    failAt = failure;
    assert(!rtc.set(expected));
    failAt = 0;
    assert(rtc.set(expected) && rtc.now(actual));
  }
  reset();
  dropRestart = true;
  assert(!rtc.set(expected) && !rtc.now(actual));
  dropRestart = false;
  assert(rtc.set(expected));
  stoppedOscillator = true;
  assert(!rtc.set(expected) && !rtc.now(actual));
  stoppedOscillator = false;
  assert(rtc.set(expected));

  reset();
  BoardConfig::ACTIVE.sensors.rtcType = BoardConfig::RtcType::Ds3231;
  BoardConfig::ACTIVE.sensors.rtcAddr = 0x68;
  Rtc ds;
  assert(ds.begin() && ds.set(expected) && ds.now(actual));
  assert(actual.hour == expected.hour && actual.minute == expected.minute && actual.second == expected.second);
  printf("RTC clock tests PASS: fallback, tick/sleep retention, mode flags, %d I2C faults, DS3231\n", setOperations);
}
