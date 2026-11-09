#include <BatteryChargeEstimator.h>
#include <SilkscreenBattery.h>
#include "ProductionCurve.h"

#include <cassert>
#include <cstdio>
#include <initializer_list>

using Est = freeink::battery::BatteryChargeEstimator;
using Supply = Est::Supply;
using Phase = Est::Phase;
constexpr Est::Config CONFIG{BoardConfig::silkscreen_battery::CAPACITY_MAH,
  BoardConfig::silkscreen_battery::CHARGE_CURRENT_MA, BoardConfig::silkscreen_battery::DEFAULT_STEP_MV,
  BoardConfig::silkscreen_battery::STEP_CREEP_PERMILLE};
static_assert(CONFIG.capacityMah == 500 && CONFIG.creepPermille == 1000, "approved Silkscreen defaults");
Est::Context context(Supply s = Supply::Battery, uint16_t light = 0, bool busy = false) {
  return {s, light, busy, false, true};
}
void feed(Est& e, uint32_t at, uint16_t mv, Supply s = Supply::Battery, uint16_t light = 0, bool busy = false) {
  auto c = context(s, light, busy);
  e.observe(at, c);
  if (busy) e.skip();
  else e.sample(at, mv, c);
}
uint16_t voltage(int tenths) {
  if (tenths >= 1000) return LIION_NOTCH_MV[10];
  for (int i = 1; i <= 10; ++i) if (tenths <= i * 100)
    return static_cast<uint16_t>(LIION_NOTCH_MV[i-1] + (tenths - (i-1)*100) *
                                (LIION_NOTCH_MV[i] - LIION_NOTCH_MV[i-1]) / 100);
  return LIION_NOTCH_MV[0];
}

int main() {
  // Real loop/ADC work makes the one-second sample interval slightly longer.
  for (const uint32_t interval : {1001u, 1017u, 1300u}) {
    Est jitter(CONFIG, percentTenthsFromNotches);
    for (uint32_t i = 0; i < 3; ++i) {
      const uint32_t now = 2500 + i * interval;
      const auto c = context();
      jitter.observe(now, c);
      assert(jitter.sampleDue(now) && jitter.canSample());
      assert(jitter.sample(now, 3850, c));
      if (i < 2) assert(jitter.shown() == Est::UNKNOWN);
    }
    assert(jitter.shown() == 55 && jitter.tenths() == 560);

    Est lightJitter(CONFIG, percentTenthsFromNotches);
    for (uint32_t i = 0; i < 10; ++i) feed(lightJitter, i * interval, 3850);
    for (uint32_t i = 10; i < 30; ++i)
      feed(lightJitter, i * interval, 3830, Supply::Battery, 1);
    assert(lightJitter.lightCorrectionMv() == 20);
    assert(!(lightJitter.flags() & Est::LightHold) && lightJitter.shown() == 55);

    Est chargeJitter(CONFIG, percentTenthsFromNotches);
    for (uint32_t i = 0; i < 30; ++i) feed(chargeJitter, i * interval, 3790);
    for (uint32_t i = 30; i < 46; ++i)
      feed(chargeJitter, i * interval, 3940, i < 33 ? Supply::Battery : Supply::Charging);
    Est::Calibration learned{};
    assert(chargeJitter.takeCalibration(learned) && learned.stepMv == 150 && learned.count == 1);
    assert(chargeJitter.shown() == 40);
  }

  Est e(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 30000; t += 1000) feed(e, t, 3790);
  assert(e.shown() == 40);
  // Raw rail moves before the ladder publishes Charging three seconds later.
  for (uint32_t t = 30000; t <= 40000; t += 1000)
    feed(e, t, 3940, t < 33000 ? Supply::Battery : Supply::Charging);
  assert(e.stepMv() == 150 && e.shown() == 40);
  assert(e.tenths() == 400); // no pre-publication plug lift in the seed
  Est::Calibration calibration{};
  assert(e.takeCalibration(calibration) && calibration.stepMv == 150 && calibration.count == 1);
  assert(!e.takeCalibration(calibration));
  // Rate, not elapsed gaps or a large voltage target, limits charging progress.
  const auto initial = e.tenths();
  for (uint32_t t = 41000; t <= 641000; t += 1000) feed(e, t, 4200, Supply::Charging);
  assert(e.tenths() >= initial && e.tenths() <= initial + 84);
  const auto held = e.tenths();
  feed(e, 900000, 4200, Supply::Charging, 0, true);
  feed(e, 1200000, 4200, Supply::Charging);
  assert(e.tenths() == held && e.shown() < 100);
  // Only the unchanged charger decoder's Complete state confirms full.
  e.observe(1201000, context(Supply::Complete, 0, true));
  assert(e.shown() == 100 && e.tenths() == 1000);
  assert(e.takeCompleteReference() && e.completeReferenceMv() == 4200 && e.completeReferenceAt() == 1200000);
  assert(!e.takeCompleteReference());
  feed(e, 1202000, 4180, Supply::Battery);
  const auto full = e.tenths();
  feed(e, 1211000, 3500);
  assert(e.tenths() == full); // electrical settling hold
  for (uint32_t t = 1212000; t < 1250000; t += 1000) feed(e, t, 3850);
  const auto lower = e.tenths();
  for (uint32_t t = 1250000; t < 1300000; t += 1000) {
    const auto before = e.tenths();
    feed(e, t, 4200);
    assert(e.tenths() <= before); // delayed data may still lower it, never raise it
  }
  assert(e.tenths() <= lower);
  for (Supply fallback : {Supply::Unknown, Supply::ExternalIdle, Supply::NoBattery}) {
    const auto before = e.tenths();
    feed(e, 1310000, 4200, fallback);
    assert(e.tenths() == before);
  }

  Est boot(CONFIG, percentTenthsFromNotches);
  boot.setCalibration({160, 8});
  for (uint32_t t = 0; t < 30UL * 60000; t += 1000) feed(boot, t, 3950, Supply::Charging);
  assert(boot.shown() == 40 && boot.phase() == Phase::Charging);
  assert(!boot.takeCalibration(calibration)); // a flat weak-USB rail is not proof of CV
  Est waking(CONFIG, percentTenthsFromNotches);
  waking.setCalibration(boot.calibration()); waking.restore(boot.save());
  feed(waking, 31UL * 60000, 4200, Supply::Charging);
  assert(waking.shown() == 40); // no catch-up on wake

  Est light(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 10000; t += 1000) feed(light, t, 3850);
  for (uint32_t t = 10000; t < 30000; t += 1000) feed(light, t, 3830, Supply::Battery, 1);
  assert(light.lightCorrectionMv() == 20 && !(light.flags() & Est::LightHold) && light.shown() == 55);
  for (uint32_t t = 30000; t < 50000; t += 1000) feed(light, t, 3820, Supply::Battery, 2);
  assert(light.lightCorrectionMv() == 30 && light.shown() == 55);
  for (uint32_t t = 50000; t < 100000; t += 1000) feed(light, t, 3770, Supply::Battery, 2);
  assert(light.shown() <= 45); // a long lit session continues tracking discharge

  Est noisy(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 10000; t += 1000) feed(noisy, t, 3850);
  for (uint32_t t = 10000; t < 30000; t += 1000) feed(noisy, t, t % 2000 ? 3790 : 3830, Supply::Battery, 7);
  assert((noisy.flags() & Est::LightHold) && noisy.shown() == 55);
  Est scrub(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 10000; t += 1000) feed(scrub, t, 3850);
  feed(scrub, 10000, 3840, Supply::Battery, 1);
  feed(scrub, 11000, 3830, Supply::Battery, 2);
  for (uint32_t t = 12000; t < 30000; t += 1000) feed(scrub, t, 3820, Supply::Battery, 3);
  assert(scrub.lightCorrectionMv() == 30 && scrub.shown() == 55);

  Est anchored(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 10000; t += 1000) feed(anchored, t, 3850);
  for (uint32_t t = 10000; t < 30000; t += 1000) feed(anchored, t, 3830, Supply::Battery, 1);
  for (uint32_t t = 30000; t < 45000; t += 1000)
    feed(anchored, t, 4000, t < 33000 ? Supply::Battery : Supply::Charging, 1);
  assert(anchored.stepMv() == 170 && anchored.compensatedMv() == 3850);
  for (uint32_t t = 45000; t < 55000; t += 1000) feed(anchored, t, 4000, Supply::Charging, 0);
  assert(anchored.compensatedMv() == 3850); // USB brightness changes do not double-compensate
  feed(anchored, 55000, 3850, Supply::Battery, 0);
  for (uint32_t t = 56000; t < 80000; t += 1000) feed(anchored, t, 3850);
  assert(anchored.compensatedMv() == 3850);

  Est changed(CONFIG, percentTenthsFromNotches);
  for (uint32_t t = 0; t < 30000; t += 1000) feed(changed, t, 3790);
  for (uint32_t t = 30000; t < 50000; t += 1000)
    feed(changed, t, 3940, t < 33000 ? Supply::Battery : Supply::Charging, t >= 34000 ? 1 : 0);
  assert(!changed.takeCalibration(calibration) && (changed.flags() & Est::StepRejected));

  Est taper(CONFIG, percentTenthsFromNotches);
  bool sawTaper = false, saw99 = false;
  for (uint32_t t = 0; t <= 180UL * 60000; t += 1000) {
    const uint16_t raw = t < 60000 ? 3790 : voltage(std::min<int>(900, 400 + static_cast<int>((t - 60000) / 7200))) + 150;
    feed(taper, t, raw, t < 63000 ? Supply::Battery : Supply::Charging);
    sawTaper |= taper.phase() == Phase::Taper;
    saw99 |= taper.shown() == 99;
    assert(taper.shown() != 100);
  }
  if (!sawTaper || !saw99) fprintf(stderr, "taper=%d 99=%d final phase=%u soc=%u shown=%u\n", sawTaper, saw99, static_cast<unsigned>(taper.phase()), taper.tenths(), taper.shown());
  assert(sawTaper && saw99);
  feed(taper, 181UL * 60000, 4200, Supply::Complete);
  assert(taper.shown() == 100);
  const auto priorCal = taper.calibration();
  for (uint32_t t = 181UL * 60000 + 1000; t < 182UL * 60000; t += 1000) feed(taper, t, 4150, Supply::Complete);
  for (uint32_t t = 182UL * 60000; t < 183UL * 60000; t += 1000) feed(taper, t, 4200, Supply::Charging);
  assert(taper.stepMv() == 50 && taper.shown() == 100);
  assert(taper.calibration().stepMv == priorCal.stepMv && taper.calibration().count == priorCal.count);

  Est wrap(CONFIG, percentTenthsFromNotches);
  for (uint32_t i = 0; i < 30; ++i) feed(wrap, UINT32_MAX - 20000 + i * 1000, 3850);
  assert(wrap.shown() == 55);
  puts("Charge estimator PASS: delayed edges, learned/default offsets, rate/gap caps, completion, relaxation, lit/noisy levels, USB light changes, taper, restart and wrap");

  // Planned hardware campaign, scripted at 1 Hz: 40%, 30 minutes charging,
  // then 30 minutes resting. Legacy is the original per-getter ADC EMA at the
  // same cadence, not a claim about the old UI's variable render frequency.
  puts("trace_min,raw_mv,legacy_shown,new_shown,new_tenths,phase,step_mv,comp_mv");
  Est trace(CONFIG, percentTenthsFromNotches);
  int legacy = 0;
  for (uint32_t t = 0; t <= 3630000; t += 1000) {
    const bool plugged = t >= 30000 && t < 1830000;
    const bool decoded = t >= 33000 && t < 1833000;
    int soc = t < 30000 ? 400 : std::min<int>(650, 400 + static_cast<int>((t - 30000) / 7200));
    int creep = t < 30000 ? 0 : t < 1830000 ? static_cast<int>((t - 30000) * 15 / 1800000) :
                std::max<int>(0, 15 - static_cast<int>((t - 1830000) * 15 / 1800000));
    const uint16_t raw = static_cast<uint16_t>(voltage(soc) + (plugged ? 150 : 0) + creep);
    feed(trace, t, raw, decoded ? Supply::Charging : Supply::Battery);
    const int target = percentTenthsFromNotches(raw), delta = target - legacy;
    legacy = legacy == 0 ? target : legacy + (delta + (delta >= 0 ? 5 : -5)) / 10;
    if (t == 0 || t == 30000 || t == 90000 || t == 330000 || t == 930000 || t == 1830000 ||
        t == 1890000 || t == 2730000 || t == 3630000)
      printf("%.1f,%u,%u,%u,%u,%u,%u,%u\n", (static_cast<int>(t) - 30000) / 60000.0, raw,
             freeink::battery::BatteryDisplayFilter::quantize((legacy + 5) / 10), trace.shown(), trace.tenths(),
             static_cast<unsigned>(trace.phase()), trace.stepMv(), trace.compensatedMv());
  }
}
