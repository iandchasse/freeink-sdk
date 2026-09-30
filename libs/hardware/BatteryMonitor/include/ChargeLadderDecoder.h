#pragma once

#include <cstdint>

// Pure state machine behind BoardConfig::ChargeStatusLadder decoding. No Arduino
// dependencies, so test/host compiles it directly. The caller owns the timing
// (settle after boot, one sample per minIntervalMs) and feeds settled readings.

namespace freeink::battery_detail {

// `Window` has minMv/maxMv (inclusive) and a `state` member whose enum has the
// enumerators Unknown, OnBattery, Charging, Complete, ExternalIdle and NoBattery
// (BoardConfig::ChargeLadderWindow / BoardConfig::ChargeState).
template <typename Window>
class ChargeLadderDecoder {
 public:
  using State = decltype(Window{}.state);

  struct Params {
    const Window* windows;
    uint8_t count;
    uint8_t debounceSamples;
    uint8_t completeStableSamples;
    uint16_t blinkDeltaMv;
    uint8_t unknownHoldSamples = 0;  // 0 = an out-of-window reading keeps the state indefinitely
  };

  static State classify(const Params& p, int32_t mv) {
    for (uint8_t i = 0; i < p.count; ++i) {
      if (mv >= static_cast<int32_t>(p.windows[i].minMv) && mv <= static_cast<int32_t>(p.windows[i].maxMv)) {
        return p.windows[i].state;
      }
    }
    return State::Unknown;
  }

  // Feed one reading; returns the reported state.
  //  * Outside every window: keep the last state, and restart the runs. After
  //    unknownHoldSamples such readings in a row (when set), report Unknown.
  //  * Complete window: only time separates "charged" from a no-battery blink
  //    (two levels that both sit in the window), so report Complete only after
  //    completeStableSamples consecutive in-window readings, and NoBattery as
  //    soon as the in-window readings have swung by blinkDeltaMv or more. The
  //    first in-window reading can sit on the node's settling tail after a
  //    state change, so the swing is measured from the second one.
  //  * Any state change needs debounceSamples agreeing readings.
  State feed(const Params& p, int32_t mv) {
    State target = classify(p, mv);
    if (target == State::Unknown) {
      completeRun_ = 0;
      candidateRun_ = 0;
      if (unknownRun_ < UINT8_MAX) ++unknownRun_;
      if (p.unknownHoldSamples != 0 && unknownRun_ >= p.unknownHoldSamples) {
        state_ = State::Unknown;
        candidate_ = State::Unknown;
      }
      return state_;
    }
    unknownRun_ = 0;
    if (target == State::Complete) {
      if (completeRun_ < UINT8_MAX) ++completeRun_;
      if (completeRun_ == 2) {
        completeMin_ = mv;
        completeMax_ = mv;
      } else if (completeRun_ > 2) {
        if (mv < completeMin_) completeMin_ = mv;
        if (mv > completeMax_) completeMax_ = mv;
      }
      if (completeRun_ >= 3 && completeMax_ - completeMin_ >= static_cast<int32_t>(p.blinkDeltaMv)) {
        target = State::NoBattery;
      } else if (completeRun_ < p.completeStableSamples) {
        candidateRun_ = 0;  // still a candidate: neither confirms nor changes the state
        return state_;
      }
    } else {
      completeRun_ = 0;
    }
    if (target == candidate_) {
      if (candidateRun_ < UINT8_MAX) ++candidateRun_;
    } else {
      candidate_ = target;
      candidateRun_ = 1;
    }
    if (candidateRun_ >= (p.debounceSamples == 0 ? 1 : p.debounceSamples)) state_ = candidate_;
    return state_;
  }

  State state() const { return state_; }

 private:
  State state_ = State::Unknown;
  State candidate_ = State::Unknown;
  uint8_t candidateRun_ = 0;
  uint8_t completeRun_ = 0;
  uint8_t unknownRun_ = 0;
  int32_t completeMin_ = 0;
  int32_t completeMax_ = 0;
};

}  // namespace freeink::battery_detail
