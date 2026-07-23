#pragma once

// FreeInk SDK — frontlight manager.
//
// Drives a PWM frontlight described by BoardConfig::ACTIVE.frontlight. Inert on
// boards without one (e.g. Xteink X4/X3), so it is always safe to construct.
//
// Two hardware shapes sit behind one API:
//
//   * Single channel — one PWM pin dims one LED string (the LilyGo backlight).
//     Uses the Arduino LEDC wrapper. setColorTemperature() records the request
//     but has nothing to drive.
//
//   * Two-temperature boost driver — a switched supply rail, separate warm and
//     cool sink channels, and a regulation-sense line (de-link). Brightness is
//     one PWM channel; colour temperature is a second timer alternating the two
//     sinks. This path drives the ESP-IDF LEDC API directly: it needs three
//     channels across two timers on a fixed clock source, which the Arduino
//     wrapper does not express. The two paths never share the peripheral — a
//     board is one shape or the other.
//
// The shape comes from the board profile: warm/cool pins assigned means the
// two-temperature path.

#include <Arduino.h>
#include <BoardConfig.h>

class FrontlightManager {
 public:
  // Bring up the PWM channel(s). No-op if the board has no frontlight.
  void begin();

  // Set brightness as a 0-100 percentage. 0 turns the light off.
  void setBrightness(uint8_t percent);

  // Convenience: fully off / restore last brightness.
  void off();
  void on();

  // Warm/cool mix, 0 = fully cool, 100 = fully warm. On single-channel boards
  // this only records the request; there is no second channel to drive.
  void setColorTemperature(uint8_t warmPercent);

  bool present() const {
#if FREEINK_CAP_FRONTLIGHT
    return BoardConfig::ACTIVE.frontlight.gpio != BoardConfig::PIN_UNASSIGNED;
#else
    return false;  // frontlight code not compiled in (FREEINK_CAP_FRONTLIGHT=0)
#endif
  }
  uint8_t brightness() const { return _brightness; }
  uint8_t colorTemperature() const { return _warmPercent; }

  // True when this board drives the two-temperature boost-driver frontlight
  // rather than a single dimmed channel. Hosts can use it to decide whether to
  // offer a colour-temperature control at all.
  static bool hasColorTemperature() {
#if FREEINK_CAP_FRONTLIGHT
    return BoardConfig::ACTIVE.frontlight.warmGpio != BoardConfig::PIN_UNASSIGNED &&
           BoardConfig::ACTIVE.frontlight.coolGpio != BoardConfig::PIN_UNASSIGNED;
#else
    return false;
#endif
  }

  // --- Boost-driver lifecycle -------------------------------------------------
  // On a boost-driven frontlight the supply rail must be raised and proven to
  // reach regulation before the sinks are driven — an open-circuit converter
  // output climbs to its over-voltage limit. enable() raises the rail, probes
  // for regulation, and only then applies brightness and colour.
  //
  // Returns false if the probe fails (no LED strip fitted, over-voltage, or a
  // wiring fault): the light stays off and isHardwareFault() reads true. While
  // running, a rising edge on the sense line trips the same fault state from an
  // ISR and zeroes the channels.
  //
  // On single-channel boards these map to on()/off() and always succeed, so
  // callers can use them unconditionally.
  bool enable();
  void disable();
  bool isEnabled() const { return _enabled; }
  bool isHardwareFault() const { return _hardwareFault; }
  // Re-probe after a fault. Returns false if the fault is still present.
  bool clearFault() { return enable(); }

 private:
#if FREEINK_CAP_FRONTLIGHT
  // Two-temperature path (ESP-IDF LEDC).
  void beginMultiChannel();
  void updateMultiChannel();
  void setRail(bool on);
  bool probeRegulation();
  void armSense();
  void disarmSense();
  static void senseIsr(void* arg);
  static void probeIsr(void* arg);

  // The boost converter needs a non-zero load to regulate, so brightness never
  // reaches 0 while enabled; disable() drops the rail instead.
  static constexpr uint8_t MIN_BRIGHTNESS = 1;
  static constexpr uint32_t PROBE_TIMEOUT_MS = 150;
  // The probe leaves the sense line transitional; let it settle before arming
  // the fault ISR so the arming edge isn't read as a fault.
  static constexpr uint32_t SENSE_SETTLE_MS = 50;
#endif

  bool _begun = false;
  uint8_t _brightness = 0;
  uint8_t _lastBrightness = 50;
  uint8_t _warmPercent = 50;
  // Written by the fault ISR as well as the main path.
  volatile bool _enabled = false;
  volatile bool _hardwareFault = false;
  volatile bool _senseArmed = false;
  volatile bool _probeTriggered = false;
};
