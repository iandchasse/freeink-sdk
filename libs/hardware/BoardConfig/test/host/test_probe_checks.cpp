// Host test for the boot-time hardware checks (ProbeChecks.h): the key-ladder
// dual-pull classifier, the FocalTech chip ids, the DS3231 register pattern and
// the power pin's idle level, with the readings Silkscreen's parts give
// (HARDWARE.md §8, §9.1, §9.2, §10).
#include <ProbeChecks.h>

#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace probe = freeink::probe;

namespace {

int failures = 0;

void check(const bool ok, const char* what) {
  if (ok) return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}

// A node with `upOhms` to 3.3 V (0 = none) and `downOhms` to GND (0 = none),
// in mV. Both 0 is a floating node; the caller adds the internal pull.
int nodeMv(const double upOhms, const double downOhms) {
  if (upOhms <= 0) return downOhms > 0 ? 0 : -1;
  if (downOhms <= 0) return 3300;
  return static_cast<int>(3300.0 * downOhms / (upOhms + downOhms));
}

double parallel(const double a, const double b) { return a <= 0 ? b : b <= 0 ? a : a * b / (a + b); }

// The two readings of a ladder node: `pullUp` the external pull-up (0 = not
// fitted), `key` the pressed key's resistor (0 = none), `internal` the pad pull.
// The ADC tops out around 3.1 V at 11 dB.
void ladderReadings(const double pullUp, const double key, const double internal, uint16_t& down, uint16_t& up) {
  const auto clampAdc = [](int mv) { return static_cast<uint16_t>(mv > 3100 ? 3100 : mv < 0 ? 0 : mv); };
  down = clampAdc(nodeMv(pullUp, parallel(key, internal)));
  up = clampAdc(nodeMv(parallel(pullUp, internal), key));
}

void ladders() {
  constexpr double R_PULL_UP = 10000;  // R4 / R28
  constexpr double KEYS[] = {0, 100, 5600, 20000, 56000, 12000, 33000, 68000};
  // A fitted ladder is never taken for a floating one, released or with any key
  // held, across the internal pull's plausible range.
  for (const double internal : {20000.0, 45000.0, 100000.0}) {
    for (const double key : KEYS) {
      uint16_t down = 0;
      uint16_t up = 0;
      ladderReadings(R_PULL_UP, key, internal, down, up);
      check(!probe::ladderFloats(down, up), "a fitted ladder is present");
    }
  }
  // No pull-up fitted, nothing pressed: it follows each pull from rail to rail.
  for (const double internal : {20000.0, 45000.0, 100000.0}) {
    uint16_t down = 0;
    uint16_t up = 0;
    ladderReadings(0, 0, internal, down, up);
    check(probe::ladderFloats(down, up), "a ladder with no pull-up floats");
  }
  // No pull-up, but a key held through the test: kept (the old behaviour), never
  // a fitted ladder switched off.
  for (const double key : KEYS) {
    if (key == 0) continue;
    uint16_t down = 0;
    uint16_t up = 0;
    ladderReadings(0, key, 45000, down, up);
    check(!probe::ladderFloats(down, up), "a key held on a floating ladder keeps it on");
  }
  check(probe::ladderFloats(probe::LADDER_FLOAT_LOW_MAX_MV, probe::LADDER_FLOAT_HIGH_MIN_MV),
        "edges count as floating");
  check(!probe::ladderFloats(probe::LADDER_FLOAT_LOW_MAX_MV + 1, 3100), "pulled down too high: present");
  check(!probe::ladderFloats(0, probe::LADDER_FLOAT_HIGH_MIN_MV - 1), "pulled up too low: present");
}

void focalTech() {
  check(probe::focalTechChipId(0x64), "FT6336U");
  check(probe::focalTechChipId(0x36), "FT6236/FT6336");
  check(probe::focalTechChipId(0x06), "FT6206");
  check(!probe::focalTechChipId(0x00), "an all-zero window (Paper Mono) is not accepted");
  check(!probe::focalTechChipId(0xFF), "a floating bus is not accepted");
  check(!probe::focalTechChipId(0x18) && !probe::focalTechChipId(0x1C), "an AHT20's status byte");
  check(!probe::focalTechChipId(0xA3), "a PCF8574A echoing the register pointer");
  check(!probe::focalTechChipId(0x55), "an FT5x06 is a different register map");
}

using Regs = uint8_t[probe::DS3231_REGISTER_COUNT];

void ds3231() {
  // The power-on state: 01/01/00, day 1, 00:00:00, control 0x1C, OSF and EN32KHZ set.
  Regs por = {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0, 0, 0, 0, 0, 0, 0, 0x1C, 0x88, 0x00, 0x00, 0x00};
  check(probe::ds3231Mismatch(por) == 0, "a DS3231 at power-on");
  // A running clock: 23:59:59 on Saturday 31/12/99, EN32KHZ cleared, 25.25 C.
  Regs set = {0x59, 0x59, 0x23, 0x07, 0x31, 0x12, 0x99, 0x80, 0x80, 0x80,
              0x80, 0x80, 0x80, 0x80, 0x04, 0x00, 0xF3, 0x19, 0x40};
  check(probe::ds3231Mismatch(set) == 0, "a set DS3231 (alarm masks and a negative aging offset)");
  set[0x02] = 0x72;  // 12-hour mode, PM, 12 o'clock
  check(probe::ds3231Mismatch(set) == 0, "12-hour mode");
  set[0x05] = 0x92;  // century bit
  check(probe::ds3231Mismatch(set) == 0, "the century bit");

  Regs zero = {};
  check(probe::ds3231Mismatch(zero) == probe::DS_DAY, "all zero: the day of week is never 0");
  Regs ones;
  std::memset(ones, 0xFF, sizeof(ones));
  check(probe::ds3231Mismatch(ones) != 0, "a floating bus");
  // A PCF8523 at 0x68 after power-on: Control_3 0xE0, seconds with OS set.
  Regs pcf8523 = {0x00, 0x00, 0xE0, 0x80, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01,
                  0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x07, 0x00, 0x07};
  check(probe::ds3231Mismatch(pcf8523) != 0, "a PCF8523");
  // An AHT20-style device: every read starts again at its status byte.
  Regs aht = {0x1C, 0x80, 0x00, 0x05, 0x6A, 0x8F, 0x1C, 0x1C, 0x80, 0x00,
              0x05, 0x6A, 0x8F, 0x1C, 0x1C, 0x80, 0x00, 0x05, 0x6A};
  check(probe::ds3231Mismatch(aht) != 0, "an AHT20");

  const struct {
    uint8_t reg;
    uint8_t value;
    uint16_t bit;
    const char* what;
  } single[] = {
      {0x00, 0x80, probe::DS_SECONDS_BITS, "seconds bit 7"},
      {0x00, 0x5A, probe::DS_SECONDS_BCD, "seconds BCD"},
      {0x01, 0x60, probe::DS_MINUTES_BCD, "minutes 60"},
      {0x01, 0x80, probe::DS_MINUTES_BITS, "minutes bit 7"},
      {0x02, 0x80, probe::DS_HOURS_BITS, "hours bit 7"},
      {0x03, 0x08, probe::DS_DAY, "day bit 3"},
      {0x03, 0x00, probe::DS_DAY, "day 0"},
      {0x04, 0x41, probe::DS_DATE_BITS, "date bit 6"},
      {0x05, 0x21, probe::DS_MONTH_BITS, "month bit 5"},
      {0x0F, 0x10, probe::DS_STATUS_BITS, "status bit 4"},
      {0x12, 0x01, probe::DS_TEMP_LSB_BITS, "temperature LSB bit 0"},
  };
  for (const auto& s : single) {
    Regs regs;
    std::memcpy(regs, por, sizeof(regs));
    regs[s.reg] = s.value;
    if (s.bit == probe::DS_SECONDS_BITS || s.bit == probe::DS_MINUTES_BITS) {
      check((probe::ds3231Mismatch(regs) & s.bit) != 0, s.what);
    } else {
      check(probe::ds3231Mismatch(regs) == s.bit, s.what);
    }
  }
}

void powerPin() {
  using L = probe::PinLevel;
  check(probe::pinLevel(0) == L::Low, "released: R76 holds IO18 at 0 V");
  check(probe::pinLevel(probe::PIN_LOW_MAX_MV) == L::Low, "0.25 x VDD is low");
  check(probe::pinLevel(probe::PIN_LOW_MAX_MV + 1) == L::Undefined, "just above 0.25 x VDD");
  // R36 and R72 fitted: R28 + R36 + R72 to 3V3 against R76.
  check(probe::pinLevel(static_cast<uint16_t>(nodeMv(10000 + 68000 + 10000, 100000))) == L::Undefined,
        "R36 and R72 together: about 1.76 V");
  check(probe::pinLevel(probe::PIN_HIGH_MIN_MV - 1) == L::Undefined, "just below 0.75 x VDD");
  check(probe::pinLevel(probe::PIN_HIGH_MIN_MV) == L::High, "0.75 x VDD is high");
  check(probe::pinLevel(static_cast<uint16_t>(nodeMv(10000, 100000))) == L::High, "held: R62 against R76, 3.0 V");
}

}  // namespace

int main() {
  ladders();
  focalTech();
  ds3231();
  powerPin();
  if (failures != 0) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("probe checks: all passed\n");
  return 0;
}
