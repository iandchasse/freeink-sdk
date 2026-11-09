#include <BatteryChargeEstimator.h>
#include "ProductionCurve.h"

#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <cstdio>

using Filter = freeink::battery::BatteryDisplayFilter;
using Supply = Filter::Supply;
static_assert(sizeof(Filter) <= 40, "display estimator stays small and allocation-free");

int main() {
  Filter f;
  unsigned reads = 0;
  auto poll = [&](uint32_t now, Supply source, bool load, bool refresh, uint16_t mv) {
    if (f.sampleDue(now, source, load, refresh)) {
      ++reads;
      f.accept(now, percentTenthsFromNotches(mv));
    }
  };
  poll(2499, Supply::Battery, false, false, 3850);
  assert(reads == 0 && f.shown() == Filter::UNKNOWN);
  poll(2500, Supply::Battery, false, false, 3850);
  assert(reads == 1 && f.shown() == 55);
  poll(2501, Supply::Battery, false, false, 3740);
  assert(reads == 1);  // a render query cannot accelerate the filter
  for (uint32_t t = 4000; t < 34000; t += 1500) poll(t, Supply::Battery, true, false, 3740);
  assert(reads == 1 && f.shown() == 55);  // frontlight/radio sag is never consumed
  poll(34000, Supply::Battery, false, false, 3740);
  assert(reads == 1);
  poll(34500, Supply::Battery, false, true, 3740);
  assert(reads == 1);  // completion timestamp catches a refresh between loop passes
  poll(35000, Supply::Battery, false, false, 3850);
  assert(reads == 2 && f.shown() == 55);

  for (uint32_t t = 37000; t < 637000; t += 1500) poll(t, Supply::External, false, false, 4200);
  assert(reads == 2 && f.shown() == 55);  // no delayed seed from charging voltage
  Filter wake;
  wake.restore(f.save());
  assert(wake.shown() == 55 && !wake.sampleDue(3000, Supply::External, false, false));
  poll(638000, Supply::Battery, false, false, 3850);
  poll(647999, Supply::Battery, false, false, 3850);
  assert(reads == 2 && f.shown() == 55);
  poll(648000, Supply::Battery, false, false, 3850);
  assert(reads == 3 && f.shown() == 55);  // same rested voltage, no USB jump
  poll(650000, Supply::Complete, true, true, 3500);
  assert(reads == 3 && f.shown() == 100);  // charger status, not a voltage guess
  poll(651000, Supply::External, false, false, 3700);
  assert(reads == 3 && f.shown() == 100);
  poll(652000, Supply::Battery, false, false, 4180);
  poll(662000, Supply::Battery, false, false, 4180);
  assert(reads == 4 && f.shown() == 100);

  Filter coldUsb;
  assert(!coldUsb.sampleDue(60000, Supply::External, false, false));
  assert(coldUsb.shown() == Filter::UNKNOWN);
  assert(!coldUsb.sampleDue(120000, Supply::Complete, false, false));
  assert(coldUsb.shown() == 100);
  Filter unknown;
  assert(!unknown.sampleDue(60000, Supply::Unknown, false, false));
  assert(unknown.shown() == Filter::UNKNOWN);

  for (int p : {510, 499, 481, 475, 500}) assert(Filter::stabilize(p, 50, false) == 50);
  assert(Filter::stabilize(474, 50, false) == 45);
  assert(Filter::stabilize(640, 50, true) == 50);
  assert(Filter::stabilize(650, 50, true) == 65);
  assert(Filter::stabilize(570, 50, false) == 55);
  Filter zero;
  assert(zero.sampleDue(3000, Supply::Battery, false, false));
  zero.accept(3000, 0);
  zero.accept(4500, 500);
  assert(zero.save().tenths == 50 && zero.shown() == 0);  // zero is valid history
  Filter fine(1);
  fine.accept(3000, 563);
  assert(fine.shown() == 56);  // keep the battery-log image's 1% resolution

  Filter wrap;
  assert(wrap.sampleDue(UINT32_MAX - 30000, Supply::Battery, false, false));
  wrap.accept(UINT32_MAX - 30000, 560);
  assert(!wrap.sampleDue(UINT32_MAX - 10000, Supply::External, false, false));
  assert(!wrap.sampleDue(UINT32_MAX - 5000, Supply::Battery, false, false));
  assert(!wrap.sampleDue(2000, Supply::Battery, false, false));
  assert(wrap.sampleDue(6000, Supply::Battery, false, false));
  for (int step : {1, 5}) {
    for (int previous = 0; previous <= 100; previous += step) {
      for (int p = 0; p <= 1000; ++p) {
        const auto shown = Filter::stabilize(p, static_cast<uint16_t>(previous), true, step);
        assert(shown <= 100 && shown % step == 0);
        assert(shown <= previous || p >= previous * 10 + Filter::LARGE_RISE_TENTHS);
      }
    }
  }
  puts("Battery display PASS: quiet sampling, charging hold/completion, USB settling, sleep carry, zero, hysteresis, rebound and timer wrap");
  // Report a sustained voltage change after the original and new EMA converge,
  // including integer rounding. Seed each case from the named resting voltage.
  const auto settledShown = [](int base, int mv, bool current) {
    using Est = freeink::battery::BatteryChargeEstimator;
    Est filter({500, 250, 150, 1000}, percentTenthsFromNotches);
    const Est::Context context{Est::Supply::Battery, 0, false, false, true};
    int cached = percentTenthsFromNotches(static_cast<uint16_t>(base));
    for (uint32_t t = 0; t <= 30000; t += 1000) filter.sample(t, static_cast<uint16_t>(base), context);
    const int sample = percentTenthsFromNotches(static_cast<uint16_t>(mv));
    for (uint32_t n = 1; n <= 300; ++n) {
      if (current) {
        filter.sample(30000 + n * 1000, static_cast<uint16_t>(mv), context);
      } else {
        const int before = cached;
        const int delta = sample - cached;
        cached = cached == 0 ? sample : cached + (delta + (delta >= 0 ? 5 : -5)) / 10;
        if (cached == before) break;
      }
    }
    return current ? filter.shown() : Filter::quantize((cached + 5) / 10);
  };
  puts("mV,seed,old_fall_mV,old_rise_mV,new_fall_mV,new_rise_mV (constant quiet voltage after EMA convergence)");
  for (int base : {3750, 3850, 4000}) {
    const auto seed = Filter::quantize((percentTenthsFromNotches(base) + 5) / 10);
    int edges[4] = {};
    for (int direction = -1; direction <= 1; direction += 2) {
      for (int delta = 1; delta <= 1000; ++delta) {
        const int index = direction < 0 ? 0 : 1;
        if (!edges[index] && settledShown(base, base + direction * delta, false) != seed) edges[index] = delta;
        if (!edges[index + 2] && settledShown(base, base + direction * delta, true) != seed) edges[index + 2] = delta;
      }
    }
    printf("%d,%u,%d,%d,%d,%d\n", base, seed, edges[0], edges[1], edges[2], edges[3]);
  }
}
