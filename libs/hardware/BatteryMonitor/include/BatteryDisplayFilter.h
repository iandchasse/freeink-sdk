#pragma once

#include <cstdint>

namespace freeink::battery {

// Display estimate only. The caller supplies quiet ADC samples and cached charger
// state; this never reads hardware or participates in the battery cutoff.
class BatteryDisplayFilter {
 public:
  enum class Supply : uint8_t { Unknown, Battery, External, Complete };
  static constexpr uint16_t UNKNOWN = 101;
  static constexpr uint32_t SAMPLE_MS = 1500;
  static constexpr uint32_t LOAD_SETTLE_MS = 2000;
  static constexpr uint32_t USB_SETTLE_MS = 10000;
  static constexpr uint32_t BOOT_SETTLE_MS = 2500;
  static constexpr int DEADBAND_TENTHS = 20;
  static constexpr int LARGE_RISE_TENTHS = 150;

  struct Saved {
    uint16_t tenths = 0;
    uint16_t shown = UNKNOWN;
    Supply supply = Supply::Unknown;
  };

  static constexpr uint16_t quantize(int percent, int step = 5) {
    percent = percent < 0 ? 0 : percent > 100 ? 100 : percent;
    return static_cast<uint16_t>(step == 5 && percent >= 98 ? 100 : percent / step * step);
  }

  static constexpr uint16_t stabilize(int tenths, uint16_t previous, bool onBattery, int step = 5) {
    const auto candidate = quantize((tenths + 5) / 10, step);
    if (previous > 100) return candidate;
    if (candidate < previous && quantize((tenths + DEADBAND_TENTHS + 5) / 10, step) < previous)
      return candidate;
    if (candidate > previous && (!onBattery || tenths >= previous * 10 + LARGE_RISE_TENTHS) &&
        quantize((tenths - DEADBAND_TENTHS + 5) / 10, step) > previous)
      return candidate;
    return previous;
  }

  explicit BatteryDisplayFilter(int step = 5) : step_(step == 1 ? 1 : 5) {}
  uint16_t shown() const { return saved_.shown; }
  Saved save() const { return saved_; }
  void restore(Saved saved) {
    if (saved.tenths > 1000 || saved.shown > 100 || saved.supply > Supply::Complete) return;
    saved_ = saved;
    saved_.shown = quantize(saved.shown, step_);
    reseed_ = true;
  }

  // Main-loop owner. External power never qualifies as a rest-voltage sample.
  // Unknown supply also holds: it must not seed from a cable still being decoded.
  bool sampleDue(uint32_t now, Supply supply, bool loadActive, bool recentRefresh) {
    if (loadActive) {
      lastLoad_ = now;
      loadSeen_ = true;
    }
    if (now >= BOOT_SETTLE_MS) bootSettled_ = true;
    if (supply == Supply::Unknown) return false;
    const bool wasExternal = saved_.supply == Supply::External || saved_.supply == Supply::Complete;
    const bool external = supply == Supply::External || supply == Supply::Complete;
    if (saved_.supply != Supply::Unknown && external != wasExternal) {
      usbChanged_ = now;
      usbSettling_ = true;
      reseed_ = true;
      charged_ = wasExternal;
    }
    saved_.supply = supply;
    if (supply == Supply::Complete) {
      saved_.tenths = 1000;
      saved_.shown = 100;
    }
    if (external || !bootSettled_ || loadActive || recentRefresh ||
        (loadSeen_ && static_cast<uint32_t>(now - lastLoad_) < LOAD_SETTLE_MS)) return false;
    if (usbSettling_ && static_cast<uint32_t>(now - usbChanged_) < USB_SETTLE_MS) return false;
    return !sampled_ || static_cast<uint32_t>(now - lastSample_) >= SAMPLE_MS;
  }

  void noteSampleAttempt(uint32_t now) {
    sampled_ = true;
    lastSample_ = now;
  }

  void accept(uint32_t now, uint16_t tenths) {
    if (tenths > 1000) return;
    if (reseed_ || saved_.shown == UNKNOWN) {
      saved_.tenths = tenths;
    } else {
      const int delta = static_cast<int>(tenths) - saved_.tenths;
      saved_.tenths = static_cast<uint16_t>(saved_.tenths + (delta + (delta >= 0 ? 5 : -5)) / 10);
    }
    // A confirmed charge cycle may add a small amount of real charge. Ordinary
    // load recovery on battery must clear the larger upward threshold instead.
    saved_.shown = stabilize(saved_.tenths, saved_.shown, !charged_, step_);
    charged_ = false;
    reseed_ = false;
    usbSettling_ = false;
    noteSampleAttempt(now);
  }

 private:
  Saved saved_{};
  uint32_t lastSample_ = 0;
  uint32_t lastLoad_ = 0;
  uint32_t usbChanged_ = 0;
  int step_;
  bool bootSettled_ = false;
  bool sampled_ = false;
  bool loadSeen_ = false;
  bool usbSettling_ = false;
  bool reseed_ = true;
  bool charged_ = false;
};

static_assert(BatteryDisplayFilter::stabilize(499, 50, false) == 50, "hold at a 5% boundary");
static_assert(BatteryDisplayFilter::stabilize(479, 50, false) == 50, "hold inside the lower deadband");
static_assert(BatteryDisplayFilter::stabilize(474, 50, false) == 45, "cross the lower deadband");
static_assert(BatteryDisplayFilter::stabilize(640, 50, true) == 50, "small recovery is not new charge");
static_assert(BatteryDisplayFilter::stabilize(650, 50, true) == 65, "accept a large recovery");

}  // namespace freeink::battery
