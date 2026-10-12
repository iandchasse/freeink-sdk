// The X3 UC8279d driver (the Silkscreen 3.68" variant's) reports the refresh it fires to
// EpdBus::noteRefresh() under FREEINK_DIAGNOSTICS, as the SSD1677 and UC8279 X4 drivers do: the mode it
// actually ran after any FAST/HALF promotion (the GC bank is a FULL or the caller's HALF, only the DU bank
// is a FAST), the CDI byte in the sequence field, whether the caller waited, the turn-off request, and no
// custom LUT. The record is closed by the completion wait displayFinish() already does, so it is never left
// in flight by a blocking display(). The UC8253 driver reports nothing: no build of ours links it.
// Every check below is an assert(): a build with -DNDEBUG must not turn this into a pass that ran nothing.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <vector>

#include "driver/Uc8279Driver.h"

namespace {
using freeink::EpdBus;
using freeink::RefreshMode;

const uint8_t kFull = static_cast<uint8_t>(RefreshMode::Full);
const uint8_t kHalf = static_cast<uint8_t>(RefreshMode::Half);
const uint8_t kFast = static_cast<uint8_t>(RefreshMode::Fast);

// Runs `act` and returns the one report it made.
template <typename F>
freeink::RefreshReport once(F act) {
  const unsigned before = EpdBus::report.count;
  const unsigned closedBefore = EpdBus::report.closed;
  act();
  assert(EpdBus::report.count == before + 1);
  assert(EpdBus::report.closed == closedBefore + 1);  // closed by displayFinish()'s wait
  assert(!EpdBus::report.inFlight);
  return EpdBus::report;
}

void uc8279() {
  constexpr size_t n = 792 / 8 * 528;
  std::vector<uint8_t> fb(n, 0xAA);
  EpdBus bus;
  freeink::Uc8279Driver d;
  d.begin(bus);

  // The first paint runs the GC bank, with CDI 0x97 (the first refresh after init).
  auto r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Full, false); });
  assert(r.mode == kFull && r.sequence == 0x97 && !r.async && !r.turnOff && !r.customLut);

  // The boot budget keeps the next paint on GC even for a FAST: reported as the FULL it ran.
  r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Fast, false); });
  assert(r.mode == kFull && r.sequence == 0xD7);

  // Then a FAST with a baseline is the DU bank, a HALF is the GC bank reported as HALF.
  r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Fast, false); });
  assert(r.mode == kFast && r.sequence == 0xD7);
  r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Half, false); });
  assert(r.mode == kHalf);

  r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Fast, true); });
  assert(r.mode == kFast && r.turnOff);

  // A resync request forces GC for the next refresh.
  d.requestResync(0);
  r = once([&] { d.display(bus, fb.data(), nullptr, RefreshMode::Fast, false); });
  assert(r.mode == kFull);

  d.display(bus, fb.data(), nullptr, RefreshMode::Fast, false);
  const unsigned count = EpdBus::report.count;
  const bool started = d.displayStart(bus, fb.data(), nullptr, RefreshMode::Fast, false);
  assert(started);
  assert(EpdBus::report.count == count + 1 && EpdBus::report.async && EpdBus::report.inFlight);
  d.displayFinish(bus, fb.data());
  assert(!EpdBus::report.inFlight);
}
}  // namespace

int main() {
  uc8279();
  std::puts("PASS: the X3 UC8279d driver reports its refreshes to the diagnostics");
}
