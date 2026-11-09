#include <FrontlightBrightness.h>
#include <FrontlightStartup.h>

#include <cassert>
#include <cstdint>
#include <initializer_list>

using namespace freeink::frontlight;

int main() {
  constexpr uint8_t percents[] = {1, 2, 10, 25, 50, 75, 100};
  constexpr uint32_t duties[] = {1, 1, 8, 34, 105, 203, 325};
  for (unsigned i = 0; i < 7; ++i) assert(mappedDuty(percents[i], 1023, 50) == duties[i]);
  assert(mappedDuty(0, 1023, 50) == 0);
  uint32_t previous = 0;
  for (unsigned raw = 0; raw <= 255; ++raw) {
    const auto percent = static_cast<uint8_t>(raw);
    const auto normalized = normalizeBrightness(percent);
    assert(normalized == (raw > 100 ? 100 : raw));
    assert(normalizeBrightness(normalized) == normalized);
    const auto duty = mappedDuty(percent, 1023, 50);
    assert(duty >= previous && duty <= 325);
    previous = duty;
    assert(mappedDuty(percent, 1023, 0) == legacyDuty(percent, 1023));
    for (bool stepped : {false, true}) {
      const auto oldDuty = stepped ? legacySteppedDuty(percent) : legacyDuty(percent, 1023);
      const auto migrated = migrateBrightnessDown(percent, 1023, 50, stepped);
      const auto selected = mappedDuty(migrated, 1023, 50);
      assert(selected <= oldDuty);
      for (uint8_t candidate = 1; candidate <= 100; ++candidate) {
        const auto alternative = mappedDuty(candidate, 1023, 50);
        assert(alternative <= selected || alternative > oldDuty);
      }
    }
    assert(migrateBrightnessDown(percent, 1023, 0) == normalized);
  }
  assert(migrateBrightnessDown(0, 1023, 50) == 0);
  assert(migrateBrightnessDown(1, 1023, 50) == 1);
  assert(migrateBrightnessDown(35, 1023, 50) == 69);
  assert(migrateBrightnessDown(50, 1023, 50) == 100);
  assert(migrateBrightnessDown(100, 1023, 50, true) == 69);
  constexpr uint32_t oldDuties[] = {1, 2, 3, 6, 10, 18, 32, 57, 101, 180};
  for (unsigned i = 0; i < 10; ++i) assert(legacySteppedDuty((i + 1) * 10) == oldDuties[i]);
  for (uint8_t endpoint : {25, 50, 100}) {
    for (uint32_t full : {255u, 1023u, 65535u}) {
      assert(mappedDuty(1, full, endpoint) == legacyDuty(1, full));
      assert(mappedDuty(100, full, endpoint) == legacyDuty(endpoint, full));
    }
  }
  assert(idleRailVerdict(false, 3000, 1000, 8000) == StartupFault::NoSamples);
  assert(idleRailVerdict(true, 0, 1000, 8000) == StartupFault::IdleRail);
  assert(idleRailVerdict(true, 999, 1000, 8000) == StartupFault::IdleRail);
  assert(idleRailVerdict(true, 1000, 1000, 8000) == StartupFault::None);
  assert(idleRailVerdict(true, 8000, 1000, 8000) == StartupFault::None);
  assert(idleRailVerdict(true, 8001, 1000, 8000) == StartupFault::IdleRail);
  assert(riseVerdict(0, 16000, 8000) == StartupFault::NoSamples);
  assert(riseVerdict(25, 7999, 8000) == StartupFault::NoRise);
  assert(riseVerdict(25, 8000, 8000) == StartupFault::None);
  for (unsigned bits = 0; bits < 8; ++bits)
    assert(restoreEligible(bits & 1, bits & 2, bits & 4) == ((bits & 1) || ((bits & 2) && (bits & 4))));
  assert(!restoreDue(99999, 0, 250));
  assert(!restoreDue(1249, 1000, 250));
  assert(restoreDue(1250, 1000, 250));
  assert(!restoreDue(148, UINT32_MAX - 100, 250));
  assert(restoreDue(149, UINT32_MAX - 100, 250));
}
