#pragma once

#include <cstdint>

namespace freeink::frontlight {

enum class StartupFault : uint8_t { None, IdleRail, NoSamples, NoRise };

constexpr StartupFault idleRailVerdict(const bool sampled, const uint32_t railMv,
                                     const uint16_t minMv, const uint16_t maxMv) {
  if (!sampled) return StartupFault::NoSamples;
  return railMv >= minMv && railMv <= maxMv ? StartupFault::None : StartupFault::IdleRail;
}

constexpr StartupFault riseVerdict(const uint32_t samples, const uint32_t peakMv, const uint16_t minMv) {
  if (samples == 0) return StartupFault::NoSamples;
  return peakMv >= minMv ? StartupFault::None : StartupFault::NoRise;
}

constexpr bool restoreEligible(const bool deepSleep, const bool softwareReset, const bool validSilentRestart) {
  return deepSleep || (softwareReset && validSilentRestart);
}

constexpr bool restoreDue(const uint32_t now, const uint32_t firstPaint, const uint16_t delayMs) {
  return firstPaint != 0 && static_cast<uint32_t>(now - firstPaint) >= delayMs;
}

}  // namespace freeink::frontlight
