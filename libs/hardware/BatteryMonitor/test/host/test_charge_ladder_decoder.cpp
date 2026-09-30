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
constexpr Decoder::Params PARAMS = {WINDOWS, 4, 2, 8, 70, 30};

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

  // With ST mixed in, the smallest swing is 508 <-> 415 (93 mV). The first
  // in-window sample only starts the run; the swing counts from the second.
  Decoder e;
  CHECK(feedN(e, 1115, 2) == State::Charging);
  CHECK(e.feed(PARAMS, 508) == State::Charging);
  CHECK(e.feed(PARAMS, 415) == State::Charging);
  CHECK(e.feed(PARAMS, 508) == State::Charging);  // swing seen: NoBattery candidate
  CHECK(e.feed(PARAMS, 415) == State::NoBattery);

  // The ST step alone (550 <-> 508, 42 mV) is not a blink.
  Decoder f;
  CHECK(feedN(f, 1115, 2) == State::Charging);
  for (int i = 0; i < 7; ++i) CHECK(f.feed(PARAMS, i % 2 == 0 ? 550 : 508) == State::Charging);
  CHECK(f.feed(PARAMS, 550) == State::Charging);
  CHECK(f.feed(PARAMS, 508) == State::Complete);
}

void testSettlingTailIsNotABlink() {
  // After STDBY asserts, the node relaxes toward ~550 mV (tau ~17 ms); a 1 Hz sample
  // can catch the tail inside the window on the way down.
  Decoder d;
  CHECK(feedN(d, 1115, 2) == State::Charging);
  CHECK(d.feed(PARAMS, 680) == State::Charging);
  CHECK(feedN(d, 550, 10) == State::Complete);

  Decoder e;
  CHECK(feedN(e, 2200, 3) == State::OnBattery);
  CHECK(e.feed(PARAMS, 690) == State::OnBattery);
  CHECK(feedN(e, 550, 10) == State::Complete);
}

void testUnknownHoldExpires() {
  // A reading stuck in a gap between windows drops the last state after the hold.
  Decoder d;
  CHECK(feedN(d, 1115, 2) == State::Charging);
  CHECK(feedN(d, 1500, 29) == State::Charging);
  CHECK(d.feed(PARAMS, 1500) == State::Unknown);
  CHECK(feedN(d, 1500, 10) == State::Unknown);
  // A later clean reading decodes again after the usual debounce.
  CHECK(d.feed(PARAMS, 2200) == State::Unknown);
  CHECK(d.feed(PARAMS, 2200) == State::OnBattery);

  // A short excursion keeps the state, and any in-window reading restarts the count.
  Decoder e;
  CHECK(feedN(e, 1115, 2) == State::Charging);
  CHECK(feedN(e, 1500, 5) == State::Charging);
  CHECK(e.feed(PARAMS, 1115) == State::Charging);
  CHECK(feedN(e, 1500, 29) == State::Charging);

  // The unplug transient (a sample or two in the gap before the on-battery level)
  // is unaffected.
  Decoder f;
  CHECK(feedN(f, 1115, 2) == State::Charging);
  CHECK(f.feed(PARAMS, 1500) == State::Charging);
  CHECK(f.feed(PARAMS, 2200) == State::Charging);
  CHECK(f.feed(PARAMS, 2200) == State::OnBattery);

  // unknownHoldSamples 0 keeps the state indefinitely.
  constexpr Decoder::Params keep = {WINDOWS, 4, 2, 8, 70, 0};
  Decoder g;
  for (int i = 0; i < 2; ++i) g.feed(keep, 1115);
  for (int i = 0; i < 100; ++i) g.feed(keep, 1500);
  CHECK(g.state() == State::Charging);
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
  testSettlingTailIsNotABlink();
  testUnknownHoldExpires();
  testSparseSamplesCannotFakeComplete();
  std::printf("%d checks, %d failed\n", checksRun, checksFailed);
  return checksFailed == 0 ? 0 : 1;
}
