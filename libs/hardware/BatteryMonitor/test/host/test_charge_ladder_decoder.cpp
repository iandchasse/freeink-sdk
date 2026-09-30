#include <cstdio>

#include "../../include/ChargeLadderDecoder.h"

namespace {

int checksRun = 0;
int checksFailed = 0;

#define CHECK(condition)                                               \
  do {                                                                 \
    ++checksRun;                                                       \
    if (!(condition)) {                                                \
      ++checksFailed;                                                  \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition); \
    }                                                                  \
  } while (0)

enum class State : uint8_t { Unknown, OnBattery, Charging, Complete, ExternalIdle, NoBattery };

struct Window {
  uint16_t minMv;
  uint16_t maxMv;
  State state;
};

using Decoder = freeink::battery_detail::ChargeLadderDecoder<Window>;

// Mirrors BoardConfig.h SILKSCREEN.chargeLadder (silkscreen-pcb HARDWARE.md 3.7, 13.1).
constexpr Window WINDOWS[] = {{2601, 0xFFFF, State::ExternalIdle},
                              {1800, 2600, State::OnBattery},
                              {800, 1350, State::Charging},
                              {350, 700, State::Complete}};
constexpr Decoder::Params PARAMS = {WINDOWS, 4, 2, 8, 70};

State feedN(Decoder& d, int mv, int n) {
  State s = State::Unknown;
  for (int i = 0; i < n; ++i) s = d.feed(PARAMS, mv);
  return s;
}

void testClassify() {
  CHECK(Decoder::classify(PARAMS, 3300) == State::ExternalIdle);
  CHECK(Decoder::classify(PARAMS, 2601) == State::ExternalIdle);
  CHECK(Decoder::classify(PARAMS, 2600) == State::OnBattery);
  CHECK(Decoder::classify(PARAMS, 2200) == State::OnBattery);
  CHECK(Decoder::classify(PARAMS, 1800) == State::OnBattery);
  CHECK(Decoder::classify(PARAMS, 1799) == State::Unknown);
  CHECK(Decoder::classify(PARAMS, 1500) == State::Unknown);
  CHECK(Decoder::classify(PARAMS, 1115) == State::Charging);
  CHECK(Decoder::classify(PARAMS, 954) == State::Charging);  // weak USB, ST + CHRG
  CHECK(Decoder::classify(PARAMS, 750) == State::Unknown);
  CHECK(Decoder::classify(PARAMS, 550) == State::Complete);
  CHECK(Decoder::classify(PARAMS, 508) == State::Complete);  // weak USB, ST + STDBY
  CHECK(Decoder::classify(PARAMS, 349) == State::Unknown);
}

void testDebounce() {
  Decoder d;
  CHECK(d.feed(PARAMS, 2200) == State::Unknown);  // one sample is not enough
  CHECK(d.feed(PARAMS, 2200) == State::OnBattery);
  CHECK(d.feed(PARAMS, 3300) == State::OnBattery);
  CHECK(d.feed(PARAMS, 3300) == State::ExternalIdle);
  // A single sample from a chattering switchover does not flip the state.
  CHECK(d.feed(PARAMS, 2200) == State::ExternalIdle);
  CHECK(d.feed(PARAMS, 3300) == State::ExternalIdle);
}

void testUnknownBandKeepsState() {
  Decoder d;
  CHECK(feedN(d, 1115, 2) == State::Charging);
  CHECK(d.feed(PARAMS, 1500) == State::Charging);
  CHECK(d.feed(PARAMS, 750) == State::Charging);
  CHECK(d.feed(PARAMS, 100) == State::Charging);
  // The unknown sample breaks a pending change.
  CHECK(d.feed(PARAMS, 2200) == State::Charging);
  CHECK(d.feed(PARAMS, 1500) == State::Charging);
  CHECK(d.feed(PARAMS, 2200) == State::Charging);
  CHECK(d.feed(PARAMS, 2200) == State::OnBattery);
}

void testCompleteNeedsStability() {
  Decoder d;
  CHECK(feedN(d, 1115, 2) == State::Charging);
  // Seven steady in-window samples are still only a candidate.
  CHECK(feedN(d, 550, 7) == State::Charging);
  // The eighth makes Complete the candidate; the debounce then needs one more.
  CHECK(d.feed(PARAMS, 552) == State::Charging);
  CHECK(d.feed(PARAMS, 548) == State::Complete);
  CHECK(feedN(d, 551, 5) == State::Complete);
  // Weak-USB complete (510 mV) is the same state.
  CHECK(feedN(d, 510, 3) == State::Complete);
}

void testNoBatteryBlink() {
  // Cell pulled while on USB: the charger alternates ~550 / ~410 mV every 1-4 s.
  Decoder d;
  CHECK(feedN(d, 1115, 2) == State::Charging);
  CHECK(d.feed(PARAMS, 550) == State::Charging);
  CHECK(d.feed(PARAMS, 550) == State::Charging);
  CHECK(d.feed(PARAMS, 415) == State::Charging);  // swing seen: NoBattery candidate
  CHECK(d.feed(PARAMS, 415) == State::NoBattery);
  for (int i = 0; i < 20; ++i) {
    CHECK(d.feed(PARAMS, (i / 3) % 2 == 0 ? 550 : 415) == State::NoBattery);  // never reports Complete
  }

  // With ST mixed in, the smallest swing is 508 <-> 415 (93 mV).
  Decoder e;
  CHECK(feedN(e, 1115, 2) == State::Charging);
  CHECK(e.feed(PARAMS, 508) == State::Charging);
  CHECK(e.feed(PARAMS, 415) == State::Charging);
  CHECK(e.feed(PARAMS, 508) == State::NoBattery);

  // The ST step alone (550 <-> 508, 42 mV) is not a blink.
  Decoder f;
  CHECK(feedN(f, 1115, 2) == State::Charging);
  for (int i = 0; i < 7; ++i) CHECK(f.feed(PARAMS, i % 2 == 0 ? 550 : 508) == State::Charging);
  CHECK(f.feed(PARAMS, 550) == State::Charging);
  CHECK(f.feed(PARAMS, 508) == State::Complete);
}

void testSparseSamplesCannotFakeComplete() {
  // Two samples that happen to land on the 550 mV blink phase are not enough.
  Decoder d;
  CHECK(feedN(d, 3300, 2) == State::ExternalIdle);
  CHECK(d.feed(PARAMS, 550) == State::ExternalIdle);
  CHECK(d.feed(PARAMS, 550) == State::ExternalIdle);
}

}  // namespace

int main() {
  testClassify();
  testDebounce();
  testUnknownBandKeepsState();
  testCompleteNeedsStability();
  testNoBatteryBlink();
  testSparseSamplesCannotFakeComplete();
  std::printf("%d checks, %d failed\n", checksRun, checksFailed);
  return checksFailed == 0 ? 0 : 1;
}
