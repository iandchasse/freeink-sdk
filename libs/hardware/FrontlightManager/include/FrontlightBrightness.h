#pragma once

#include <cstdint>

namespace freeink::frontlight {

// Legacy percent curve, retained for unrestricted boards and settings migration.
inline constexpr uint16_t GAMMA_TABLE[101] = {
    0,     32,    101,   197,   318,   460,   622,   803,   1001,  1217,  1449,  1697,  1960,  2237,  2529,
    2835,  3155,  3488,  3834,  4193,  4565,  4949,  5345,  5753,  6173,  6604,  7047,  7502,  7967,  8444,
    8931,  9429,  9938,  10457, 10987, 11527, 12077, 12638, 13208, 13789, 14379, 14979, 15588, 16208, 16836,
    17474, 18122, 18779, 19445, 20120, 20804, 21497, 22200, 22911, 23631, 24360, 25097, 25843, 26598, 27362,
    28134, 28914, 29703, 30500, 31306, 32120, 32942, 33772, 34611, 35457, 36312, 37175, 38045, 38924, 39811,
    40705, 41608, 42518, 43436, 44361, 45295, 46236, 47185, 48141, 49105, 50076, 51055, 52042, 53036, 54037,
    55046, 56062, 57086, 58117, 59155, 60200, 61253, 62313, 63380, 64454, 65535};

constexpr uint32_t legacyDuty(uint32_t percent, const uint32_t full) {
  if (percent == 0) return 0;
  if (percent > 100) percent = 100;
  const uint32_t duty = (full * GAMMA_TABLE[percent] + 32767u) / 65535u;
  return duty != 0 ? duty : 1u;
}

constexpr uint8_t normalizeBrightness(uint32_t percent) {
  if (percent > 100) percent = 100;
  return static_cast<uint8_t>(percent);
}

// Retain all 100 percent selections over the original curve's 1..maximum range.
// Interpolate the table before rounding once to hardware duty; 1% remains the
// original 1%, and 100% becomes the profile's maximumLegacyPercent.
constexpr uint32_t mappedDuty(uint32_t percent, const uint32_t full, uint32_t maximumLegacyPercent) {
  if (maximumLegacyPercent == 0) return legacyDuty(percent, full);
  if (percent == 0) return 0;
  percent = normalizeBrightness(percent);
  if (maximumLegacyPercent > 100) maximumLegacyPercent = 100;
  const uint32_t position = 99u + (percent - 1u) * (maximumLegacyPercent - 1u);
  const uint32_t index = position / 99u;
  const uint32_t fraction = position % 99u;
  uint32_t gamma = static_cast<uint32_t>(GAMMA_TABLE[index]) * 99u;
  if (fraction != 0) gamma += (GAMMA_TABLE[index + 1u] - GAMMA_TABLE[index]) * fraction;
  constexpr uint32_t denominator = 65535u * 99u;
  const uint32_t duty = static_cast<uint32_t>((static_cast<uint64_t>(full) * gamma + denominator / 2u) / denominator);
  return duty != 0 ? duty : 1u;
}

// Historical scale-1 output, only for migrating the short-lived ten-level build.
constexpr uint32_t legacySteppedDuty(uint32_t percent) {
  if (percent == 0) return 0;
  percent = normalizeBrightness(percent);
  const uint32_t level = (percent + 5u) / 10u;
  constexpr uint16_t duties[] = {1, 2, 3, 6, 10, 18, 32, 57, 101, 180};
  return duties[(level != 0 ? level : 1u) - 1u];
}

constexpr uint8_t migrateBrightnessDown(const uint8_t percent, const uint32_t full,
                                       const uint8_t maximumLegacyPercent, const bool tenLevelSource = false) {
  if (maximumLegacyPercent == 0) return normalizeBrightness(percent);
  const uint32_t oldDuty = tenLevelSource ? legacySteppedDuty(percent) : legacyDuty(percent, full);
  uint8_t result = 0;
  uint32_t bestDuty = 0;
  for (uint8_t candidate = 1; candidate <= 100; ++candidate) {
    const uint32_t duty = mappedDuty(candidate, full, maximumLegacyPercent);
    if (duty > oldDuty) break;
    // Prefer the lower percentage on equal-duty ties, preserving the 1% night setting.
    if (duty > bestDuty) {
      result = candidate;
      bestDuty = duty;
    }
  }
  return result;
}

}  // namespace freeink::frontlight
