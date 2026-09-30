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
  //  * Outside every window: keep the last state, and restart the runs.
  //  * Complete window: only time separates "charged" from a no-battery blink
  //    (two levels that both sit in the window), so report Complete only after
  //    completeStableSamples consecutive in-window readings, and NoBattery as
  //    soon as the in-window readings have swung by blinkDeltaMv or more.
  //  * Any state change needs debounceSamples agreeing readings.
  State feed(const Params& p, int32_t mv) {
    State target = classify(p, mv);
    if (target == State::Unknown) {
      completeRun_ = 0;
      candidateRun_ = 0;
      return state_;
    }
    if (target == State::Complete) {
      if (completeRun_ == 0) {
        completeMin_ = mv;
        completeMax_ = mv;
      }
      if (completeRun_ < UINT8_MAX) ++completeRun_;
      if (mv < completeMin_) completeMin_ = mv;
      if (mv > completeMax_) completeMax_ = mv;
      if (completeMax_ - completeMin_ >= static_cast<int32_t>(p.blinkDeltaMv)) {
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
  int32_t completeMin_ = 0;
  int32_t completeMax_ = 0;
};

}  // namespace freeink::battery_detail
