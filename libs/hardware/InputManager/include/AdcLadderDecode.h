#pragma once

#include <cstdint>

// Pure decode of one ADC resistor-ladder reading (InputStyle::AdcLadders). No
// Arduino dependencies, so the host tests in test/host compile it directly.

namespace freeink::input_detail {

constexpr int ADC_LADDER_UNCLEAN = -1;   // not near any clean level: ignore the sample
constexpr int ADC_LADDER_RELEASED = -2;  // idle: no key on this ladder
constexpr int ADC_LADDER_RAW_SATURATED = 4090;

// `Key` has minMv/maxMv (inclusive) and button members (BoardConfig::AdcLadderKey).
// Returns the button of the first window containing `mv`, ADC_LADDER_RELEASED for
// a saturated raw reading or mv >= releasedMinMv (when releasedMinMv is non-zero),
// else ADC_LADDER_UNCLEAN. `mv` is signed so a zero-offset reading at the bottom
// window still matches.
template <typename Key>
int decodeAdcLadder(const Key* keys, uint8_t count, int32_t mv, int32_t raw, uint16_t releasedMinMv) {
  if (raw >= ADC_LADDER_RAW_SATURATED) return ADC_LADDER_RELEASED;
  if (releasedMinMv != 0 && mv >= releasedMinMv) return ADC_LADDER_RELEASED;
  for (uint8_t i = 0; i < count; ++i) {
    const int32_t lo = keys[i].minMv == 0 ? INT32_MIN : static_cast<int32_t>(keys[i].minMv);
    if (mv >= lo && mv <= static_cast<int32_t>(keys[i].maxMv)) return keys[i].button;
  }
  return ADC_LADDER_UNCLEAN;
}

// Debounce for one ladder: a decode commits only after `stableSamples`
// identical clean decodes in a row. An unclean decode breaks the run and keeps
// the committed state, so a sample averaged across a press edge cannot commit.
struct AdcLadderDebounce {
  int8_t candidate = ADC_LADDER_RELEASED;
  int8_t committed = ADC_LADDER_RELEASED;
  uint8_t run = 0;

  // Returns the committed decode: a button index or ADC_LADDER_RELEASED.
  int update(int decoded, uint8_t stableSamples) {
    if (decoded == ADC_LADDER_UNCLEAN) {
      run = 0;
      return committed;
    }
    if (decoded == candidate) {
      if (run < UINT8_MAX) ++run;
    } else {
      candidate = static_cast<int8_t>(decoded);
      run = 1;
    }
    if (run >= (stableSamples == 0 ? 1 : stableSamples)) committed = candidate;
    return committed;
  }
};

}  // namespace freeink::input_detail
