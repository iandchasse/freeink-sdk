#pragma once

#include <BatteryDisplayFilter.h>

#include <algorithm>
#include <cstdint>
#include <type_traits>

namespace freeink::battery {

// Main-loop owned display estimate. All timestamps and quiet voltage readings
// come from the caller; charger decoding and the cutoff remain independent.
class BatteryChargeEstimator {
 public:
  enum class Supply : uint8_t { Unknown, Battery, Charging, Complete, ExternalIdle, NoBattery };
  enum class Phase : uint8_t { Unknown, Battery, Step, Charging, Taper, Complete, Relax };
  enum Flag : uint16_t { LightHold = 1, Busy = 2, StepRejected = 4, StepLearned = 8, ConfirmedFull = 16 };
  struct Config {
    uint16_t capacityMah;
    uint16_t chargeCurrentMa;
    uint16_t defaultStepMv;
    uint16_t creepPermille;
  };
  struct Context {
    Supply supply;
    uint16_t light;  // 0 = off; otherwise brightness + warmth, supplied by HAL
    bool loadActive;
    bool recentRefresh;
    bool lightReady;
  };
  struct Calibration { uint16_t stepMv; uint8_t count; };
  // Trivial data only: this may live in RTC_NOINIT memory.
  struct Saved {
    uint32_t unplugAt;
    uint32_t chargeAt;
    uint16_t tenths;
    uint16_t shown;
    uint16_t lightKey;
    uint16_t lightSag;
    uint16_t stepMv;
    uint16_t anchorMv;
    uint8_t supply;
    uint8_t phase;
    uint8_t flags;  // bit0 estimate valid, bit1 full confirmed, bit2 light cache valid, bit3 relax active
    uint8_t reserved;
  };
  using Curve = uint16_t (*)(uint16_t);
  static constexpr uint16_t UNKNOWN = 101;
  static constexpr uint32_t SAMPLE_MS = 1000;
  // Three readings need two sample intervals plus room for loop/ADC scheduling.
  static constexpr uint32_t SAMPLE_WINDOW_MS = 3 * SAMPLE_MS;
  static constexpr uint32_t QUIET_MS = 2000;
  static constexpr uint32_t EDGE_SETTLE_MS = 3000;
  static constexpr uint32_t EDGE_TIMEOUT_MS = 9000;
  static constexpr uint32_t USB_SETTLE_MS = 10000;
  static constexpr uint32_t RELAX_MS = 30UL * 60 * 1000;
  static constexpr uint32_t PLATEAU_MS = 3UL * 60 * 1000;
  static constexpr uint32_t TAPER_TAU_MS = 20UL * 60 * 1000;
  static constexpr uint16_t MAX_NOISE_MV = 8;
  static constexpr uint16_t MIN_STEP_MV = 30;
  static constexpr uint16_t MAX_STEP_MV = 400;
  static constexpr uint16_t MAX_LIGHT_MV = 150;
  static constexpr uint16_t CC_RISE_MV = 10;
  static constexpr uint16_t PLATEAU_RISE_MV = 2;

  BatteryChargeEstimator(Config config, Curve curve, int step = 5)
      : config_(config), curve_(curve), step_(step == 1 ? 1 : 5) {
    if (!config_.capacityMah) config_.capacityMah = 500;
    if (!config_.chargeCurrentMa) config_.chargeCurrentMa = 250;
    if (config_.defaultStepMv < MIN_STEP_MV || config_.defaultStepMv > MAX_STEP_MV) config_.defaultStepMv = 150;
    if (config_.creepPermille < 500 || config_.creepPermille > 2000) config_.creepPermille = 1000;
    calibration_ = {config_.defaultStepMv, 0};
    saved_.shown = UNKNOWN;
    saved_.stepMv = calibration_.stepMv;
  }
  void setCalibration(Calibration c) {
    if (c.count && c.count <= 16 && c.stepMv >= MIN_STEP_MV && c.stepMv <= MAX_STEP_MV) calibration_ = c;
    saved_.stepMv = calibration_.stepMv;
  }
  Calibration calibration() const { return calibration_; }
  bool takeCalibration(Calibration& c) {
    if (!calibrationDirty_) return false;
    calibrationDirty_ = false;
    c = calibration_;
    return true;
  }
  Saved save() const { return saved_; }
  void restore(Saved s) {
    if (s.tenths > 1000 || (s.shown > 100 && s.shown != UNKNOWN) || s.supply > 5 || s.phase > 6 ||
        s.lightSag > MAX_LIGHT_MV || s.stepMv > MAX_STEP_MV || s.anchorMv > MAX_LIGHT_MV || (s.flags & ~15)) return;
    saved_ = s;
    context_.supply = static_cast<Supply>(s.supply);
    phase_ = static_cast<Phase>(s.phase);
    chargeStartedAt_ = s.chargeAt;
    if (!saved_.stepMv) saved_.stepMv = calibration_.stepMv;
  }
  uint16_t shown() const { return saved_.shown; }
  uint16_t tenths() const { return saved_.tenths; }
  Phase phase() const { return phase_; }
  uint16_t rawMv() const { return rawMv_; }
  uint16_t compensatedMv() const { return compensatedMv_; }
  uint16_t stepMv() const { return saved_.stepMv; }
  uint16_t lightCorrectionMv() const { return charging() ? saved_.anchorMv : lightSag_; }
  uint16_t flags() const {
    return (context_.supply == Supply::Battery && context_.light && !lightValid_ ? LightHold : 0) | (!quiet_ ? Busy : 0) |
           (stepRejected_ ? StepRejected : 0) | (stepLearned_ ? StepLearned : 0) |
           (saved_.flags & 2 ? ConfirmedFull : 0);
  }
  uint16_t completeReferenceMv() const { return completeMv_; }
  uint32_t completeReferenceAt() const { return completeAt_; }
  bool takeCompleteReference() { const bool value = completeDirty_; completeDirty_ = false; return value; }

  // Called even between ADC samples, so short light changes cannot hide inside
  // a charger calibration window. Unknown ladder states retain trusted history.
  void observe(uint32_t now, Context c) {
    const Supply previous = static_cast<Supply>(saved_.supply);
    if (c.light != context_.light) {
      const uint16_t oldTag = tag(context_.supply);
      const uint16_t oldLight = context_.light;
      const uint16_t oldSag = lightSag_;
      const bool oldValid = lightValid_;
      const bool carryPre = lightPending_ && lightPreValid_ && lightPreKnown_ &&
                            static_cast<uint32_t>(now - lightPreAt_) < 15000;
      epoch_ = static_cast<uint16_t>((epoch_ + 1) & 8191);
      edgeValid_ = false;
      lightPending_ = false;
      if (c.supply == Supply::Battery && previous == Supply::Battery) {
        if (!carryPre) {
          lightPreValid_ = mean(now, 0, 4000, oldTag, lightPreMv_);
          lightPreSag_ = oldSag;
          lightPreKnown_ = oldValid;
          lightPreviousKey_ = oldLight;
          lightPreAt_ = now;
        }
        lightChangedAt_ = now;
        lightPending_ = lightPreValid_;
      }
      lightSag_ = 0;
      lightValid_ = c.light == 0;
      if (c.light && (saved_.flags & 4) && saved_.lightKey == c.light) {
        lightSag_ = saved_.lightSag;
        lightValid_ = true;
      }
    }
    context_ = c;
    if (c.loadActive || !c.lightReady) { busyAt_ = now; busySeen_ = true; }
    quiet_ = !c.loadActive && !c.recentRefresh && c.lightReady &&
             (!busySeen_ || static_cast<uint32_t>(now - busyAt_) >= QUIET_MS);
    if (c.supply == Supply::Unknown || c.supply == Supply::ExternalIdle || c.supply == Supply::NoBattery) return;
    if (c.supply != previous) {
      rateRemainder_ = taperRemainder_ = 0;
      validSeen_ = false;
      if (c.supply == Supply::Charging) {
        chargeStartedAt_ = saved_.chargeAt = now;
        saved_.flags &= ~8;
        saved_.stepMv = calibration_.stepMv;
        saved_.anchorMv = previous == Supply::Battery && lightValid_ ? lightSag_ : 0;
        restart_ = previous == Supply::Complete;
        edgePending_ = previous == Supply::Battery || restart_;
        edgeAt_ = now;
        edgeEpoch_ = epoch_;
        edgeValid_ = edgePending_ && mean(now, 4000, 8000, tag(previous), edgePreMv_);
        stepRejected_ = edgePending_ && !edgeValid_;
        stepLearned_ = false;
        phase_ = edgePending_ ? Phase::Step : Phase::Charging;
        resetTrend();
      } else if (c.supply == Supply::Battery) {
        if (previous == Supply::Charging || previous == Supply::Complete) {
          saved_.unplugAt = now;
          saved_.flags |= 8;
          // USB light changes do not measure a battery sag. Reuse only the
          // known battery level; an unmeasured lit level holds the estimate.
          lightValid_ = !c.light || ((saved_.flags & 4) && saved_.lightKey == c.light);
          lightSag_ = c.light && lightValid_ ? saved_.lightSag : 0;
        }
        edgePending_ = false;
        phase_ = saved_.flags & 8 ? Phase::Relax : Phase::Battery;
      }
    }
    saved_.supply = static_cast<uint8_t>(c.supply);
    if (c.supply == Supply::Complete) {
      if (previous == Supply::Charging) {
        completeMv_ = rawMv_;
        completeAt_ = rawAt_;
        completeDirty_ = true;
      }
      saved_.tenths = 1000;
      saved_.shown = 100;
      saved_.flags = static_cast<uint8_t>((saved_.flags | 3) & ~8);
      edgePending_ = false;
      phase_ = Phase::Complete;
    } else if (c.supply == Supply::Battery && (saved_.flags & 8) &&
               static_cast<uint32_t>(now - saved_.unplugAt) >= RELAX_MS) {
      saved_.flags &= ~8;
      phase_ = Phase::Battery;
    }
    saved_.phase = static_cast<uint8_t>(phase_);
  }
  bool sampleDue(uint32_t now) {
    if (attempted_ && static_cast<uint32_t>(now - attemptedAt_) < SAMPLE_MS) return false;
    attempted_ = true; attemptedAt_ = now; return true;
  }
  bool canSample() const { return quiet_; }
  void skip() {
    validSeen_ = false;
    rateRemainder_ = taperRemainder_ = 0;
    trendCount_ = 0; trendSum_ = 0; trendMs_ = 0; plateauMs_ = 0;
  }
  bool sample(uint32_t now, uint16_t mv, Context c) {
    observe(now, c);
    if (!quiet_ || mv < 2500 || mv > 4600) { skip(); return false; }
    rawMv_ = mv; rawAt_ = now;
    history_[cursor_] = {now, mv, tag(c.supply)};
    cursor_ = static_cast<uint8_t>((cursor_ + 1) % HISTORY);
    if (count_ < HISTORY) ++count_;
    uint32_t dt = validSeen_ ? static_cast<uint32_t>(now - validAt_) : 0;
    if (dt > 3000) { skip(); dt = 0; }
    dt = std::min(dt, SAMPLE_MS);
    validSeen_ = true; validAt_ = now;
    learnLight(now);
    learnChargeStep(now);
    if (c.supply == Supply::Charging) {
      int offset = saved_.stepMv;
      const int creep = static_cast<int>(config_.creepPermille) - 1000;
      offset += static_cast<int>(static_cast<int64_t>(saved_.stepMv) * creep *
                    std::min(static_cast<uint32_t>(now - chargeStartedAt_), TAPER_TAU_MS) / (1000LL * TAPER_TAU_MS));
      compensatedMv_ = clampMv(static_cast<int>(mv) - offset + saved_.anchorMv);
      if (edgePending_) return true;
      trackPlateau(mv, dt);
      uint16_t target = std::min<uint16_t>(curve_(compensatedMv_), 990);
      if (phase_ == Phase::Taper) {
        taperRemainder_ += static_cast<uint64_t>(990 - std::min<uint16_t>(saved_.tenths, 990)) * dt;
        target = static_cast<uint16_t>(saved_.tenths + taperRemainder_ / TAPER_TAU_MS);
        target = std::min<uint16_t>(target, 990);
      }
      if (!(saved_.flags & 1)) seed(target);
      else {
        const uint64_t denominator = static_cast<uint64_t>(config_.capacityMah) * 3600000;
        rateRemainder_ += static_cast<uint64_t>(config_.chargeCurrentMa) * 1000 * dt;
        const uint16_t allowance = static_cast<uint16_t>(rateRemainder_ / denominator);
        rateRemainder_ %= denominator;
        if (target > saved_.tenths) {
          const auto increment = std::min<uint16_t>(target - saved_.tenths, allowance);
          saved_.tenths += increment;
          if (phase_ == Phase::Taper) taperRemainder_ -= static_cast<uint64_t>(increment) * TAPER_TAU_MS;
        }
      }
      show(false);
    } else if (c.supply == Supply::Battery) {
      compensatedMv_ = clampMv(static_cast<int>(mv) + lightSag_);
      if (lightPending_ || (c.light && !lightValid_) ||
          ((saved_.flags & 8) && static_cast<uint32_t>(now - saved_.unplugAt) < USB_SETTLE_MS)) return true;
      // Delay trusted battery updates beyond the ladder's 2-3 s publication
      // latency, so a plug-in lift cannot contaminate the charging seed.
      uint16_t trustedMv;
      if (!mean(now, saved_.flags & 1 ? 4000 : 0, saved_.flags & 1 ? 8000 : SAMPLE_WINDOW_MS,
                tag(Supply::Battery), trustedMv)) return true;
      const uint16_t target = curve_(clampMv(static_cast<int>(trustedMv) + lightSag_));
      if (!(saved_.flags & 1)) seed(target);
      else {
        const int delta = static_cast<int>(target) - saved_.tenths;
        const int next = saved_.tenths + (delta + (delta >= 0 ? 5 : -5)) / 10;
        if (!(saved_.flags & 8) || next < saved_.tenths) saved_.tenths = static_cast<uint16_t>(next);
      }
      show(true);
    }
    saved_.phase = static_cast<uint8_t>(phase_);
    return true;
  }

 private:
  static constexpr uint8_t HISTORY = 16;
  struct Reading { uint32_t at; uint16_t mv; uint16_t tag; };
  static uint16_t clampMv(int mv) { return static_cast<uint16_t>(std::max(0, std::min(mv, 5000))); }
  uint16_t tag(Supply supply) const { return static_cast<uint16_t>((epoch_ << 3) | static_cast<uint8_t>(supply)); }
  bool charging() const { return context_.supply == Supply::Charging || context_.supply == Supply::Complete; }
  bool mean(uint32_t now, uint32_t youngest, uint32_t oldest, uint16_t wantedTag, uint16_t& value) const {
    uint32_t sum = 0; uint16_t low = 65535, high = 0; uint8_t n = 0;
    for (uint8_t i = 0; i < count_; ++i) {
      const auto& r = history_[i]; const uint32_t age = now - r.at;
      if (r.tag != wantedTag || age < youngest || age > oldest) continue;
      sum += r.mv; low = std::min(low, r.mv); high = std::max(high, r.mv); ++n;
    }
    if (n < 3 || high - low > MAX_NOISE_MV) return false;
    value = static_cast<uint16_t>((sum + n / 2) / n); return true;
  }
  void learnLight(uint32_t now) {
    if (!lightPending_) return;
    if (context_.supply != Supply::Battery) { lightPending_ = false; return; }
    const uint32_t age = now - lightChangedAt_;
    if (age >= EDGE_TIMEOUT_MS) { lightPending_ = false; return; }
    if (age >= EDGE_SETTLE_MS + 2000) {
      uint16_t after;
      if (mean(now, 0, std::min<uint32_t>(SAMPLE_WINDOW_MS, age - EDGE_SETTLE_MS), tag(Supply::Battery), after)) {
        if (!context_.light) {
          const int sag = static_cast<int>(after) - lightPreMv_;
          if (lightPreviousKey_ && sag >= 0 && sag <= MAX_LIGHT_MV) {
            saved_.lightKey = lightPreviousKey_; saved_.lightSag = static_cast<uint16_t>(sag); saved_.flags |= 4;
          }
          lightSag_ = 0; lightValid_ = true; lightPending_ = false;
        } else if (lightPreKnown_) {
          const int sag = static_cast<int>(lightPreMv_) + lightPreSag_ - after;
          if (sag >= 0 && sag <= MAX_LIGHT_MV) {
            saved_.lightKey = context_.light; saved_.lightSag = static_cast<uint16_t>(sag); saved_.flags |= 4;
            lightSag_ = static_cast<uint16_t>(sag); lightValid_ = true; lightPending_ = false;
          }
        }
      }
    }
    if (age >= EDGE_TIMEOUT_MS) lightPending_ = false;
  }
  void learnChargeStep(uint32_t now) {
    if (!edgePending_) return;
    const uint32_t age = now - edgeAt_;
    if (age >= EDGE_TIMEOUT_MS) { edgePending_ = false; stepRejected_ = true; phase_ = Phase::Charging; resetTrend(); return; }
    if (age >= EDGE_SETTLE_MS && edgeValid_ && epoch_ == edgeEpoch_) {
      uint16_t after;
      if (mean(now, 0, std::min<uint32_t>(SAMPLE_WINDOW_MS, age - 1000), tag(Supply::Charging), after)) {
        const int measured = static_cast<int>(after) - edgePreMv_;
        if (measured >= (restart_ ? 1 : MIN_STEP_MV) && measured <= MAX_STEP_MV) {
          saved_.stepMv = static_cast<uint16_t>(measured);
          stepLearned_ = true; stepRejected_ = false; edgePending_ = false;
          if (!restart_) {
            const unsigned weight = std::min<unsigned>(calibration_.count, 15);
            calibration_.stepMv = static_cast<uint16_t>((calibration_.stepMv * weight + measured + (weight + 1) / 2) / (weight + 1));
            calibration_.count = static_cast<uint8_t>(std::min<unsigned>(calibration_.count + 1, 16));
            calibrationDirty_ = true;
          }
        }
      }
    }
    if (age >= EDGE_TIMEOUT_MS) { edgePending_ = false; stepRejected_ = true; }
    if (!edgePending_) { phase_ = Phase::Charging; resetTrend(); }
  }
  void resetTrend() { trendSum_ = 0; trendCount_ = 0; trendMs_ = plateauMs_ = 0; trendBase_ = trendPeak_ = 0; }
  void trackPlateau(uint16_t mv, uint32_t dt) {
    if (phase_ == Phase::Taper || !dt) return;
    trendSum_ += mv; ++trendCount_; trendMs_ += dt;
    if (trendMs_ < 60000) return;
    const uint16_t avg = static_cast<uint16_t>(trendSum_ / trendCount_);
    if (!trendBase_) { trendBase_ = trendPeak_ = plateauAnchor_ = avg; plateauMs_ = 0; }
    else {
      trendPeak_ = std::max(trendPeak_, avg);
      if (avg > plateauAnchor_ + PLATEAU_RISE_MV || avg + MAX_NOISE_MV < plateauAnchor_) {
        plateauAnchor_ = avg; plateauMs_ = 0;
      } else plateauMs_ += trendMs_;
      if (trendPeak_ >= trendBase_ + CC_RISE_MV && plateauMs_ >= PLATEAU_MS) phase_ = Phase::Taper;
    }
    trendSum_ = 0; trendCount_ = 0; trendMs_ = 0;
  }
  void seed(uint16_t target) { saved_.tenths = target; saved_.flags |= 1; }
  void show(bool battery) {
    if (!(saved_.flags & 1)) return;
    const uint16_t previous = saved_.shown;
    uint16_t next = BatteryDisplayFilter::stabilize(saved_.tenths, previous, battery, step_);
    if (context_.supply == Supply::Charging && previous <= 100) {
      next = std::max(previous, next);
      if (previous < 99 && saved_.tenths >= 985) next = 99;
    }
    if (next == 100 && !(saved_.flags & 2)) next = 99;
    if ((saved_.flags & 8) && previous <= 100) next = std::min(next, previous);
    saved_.shown = next;
    if (next < 100) saved_.flags &= ~2;
  }
  Config config_;
  Curve curve_;
  int step_;
  Saved saved_{};
  Calibration calibration_{};
  Context context_{Supply::Unknown, 0, false, false, true};
  Phase phase_ = Phase::Unknown;
  Reading history_[HISTORY]{};
  uint64_t rateRemainder_ = 0, taperRemainder_ = 0;
  uint32_t attemptedAt_ = 0, busyAt_ = 0, validAt_ = 0, rawAt_ = 0, edgeAt_ = 0, chargeStartedAt_ = 0;
  uint32_t lightChangedAt_ = 0, lightPreAt_ = 0, trendSum_ = 0, trendMs_ = 0, plateauMs_ = 0, completeAt_ = 0;
  uint16_t rawMv_ = 0, compensatedMv_ = 0, epoch_ = 0, edgeEpoch_ = 0, edgePreMv_ = 0;
  uint16_t lightPreMv_ = 0, lightPreSag_ = 0, lightPreviousKey_ = 0, lightSag_ = 0;
  uint16_t trendBase_ = 0, trendPeak_ = 0, plateauAnchor_ = 0, completeMv_ = 0, trendCount_ = 0;
  uint8_t cursor_ = 0, count_ = 0;
  bool attempted_ = false, busySeen_ = false, quiet_ = true, validSeen_ = false;
  bool edgePending_ = false, edgeValid_ = false, restart_ = false, stepLearned_ = false, stepRejected_ = false;
  bool lightPending_ = false, lightValid_ = true, lightPreValid_ = false, lightPreKnown_ = true;
  bool calibrationDirty_ = false, completeDirty_ = false;
};
static_assert(std::is_trivial<BatteryChargeEstimator::Saved>::value, "RTC state must have no startup constructor");
static_assert(sizeof(BatteryChargeEstimator::Saved) == 24, "RTC state has explicit padding");
static_assert(sizeof(BatteryChargeEstimator) <= 384, "one bounded, allocation-free estimator per Silkscreen HAL");

}  // namespace freeink::battery
