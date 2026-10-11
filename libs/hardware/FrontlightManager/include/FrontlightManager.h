#pragma once

// FreeInk SDK — frontlight manager.
//
// Drives a PWM frontlight described by BoardConfig::ACTIVE.frontlight. Inert on
// boards without one (e.g. Xteink X4/X3), so it is always safe to construct.
//
// Two topologies, selected purely by the board profile:
//   * Single channel (de-link primary LED, LilyGo backlight, Murphy): one PWM pin
//     (frontlight.gpio). setColorTemperature() is a no-op.
//   * Warm/cool pair (Xteink X4 Pro: cool=GPIO8, warm=GPIO9): two independent PWM
//     channels. Overall brightness is the total light; color temperature splits that
//     total between the cool (gpio) and warm (gpioWarm) strings, so brightness stays
//     roughly constant as the color shifts. setColorTemperature() drives the mix.
//
// Enable-gated drivers (Silkscreen: TPS923610, whose ADIM input is both enable and
// analog dimming) opt in through FrontlightConfig::colorSelectGpio / enablePulseUs /
// monitorAdc. There the PWM pin is a plain GPIO held at its inactive level while
// dark; lighting waits offDwellMs, sets the colour-select GPIO (warm at >= 50%),
// drives an enable pulse and only then hands the pin to the LEDC carrier. A colour
// change while lit goes dark first. An optional LED-rail monitor switches the light
// off and latches fault() on over-voltage.
//
// Colour blending (FrontlightConfig::colorBlendHz, with the load probe): a warmth
// strictly between 0 and 100 puts the colour select on its own LEDC carrier, its
// duty the warm share, while lit. It needs both strings lit through a probe since
// the chip last reset (see FrontlightLoad.h colourPlan()): the first such switch-on
// lights the dominant string through its probe window, then the other (with
// blendConfirmWindowMs, the colour select moves to it while lit and the monitor is
// read for that window; otherwise dark, then a full window), and blends once both
// have lit. After that a switch-on blends from the enable pulse
// (its probe window still watches for no light), and warmth and brightness changes
// while blending apply at once. Going dark, the carrier stops at the warm level and
// the colour select then rests at cool as after a warm light.
//
// Load probe (FrontlightConfig::loadProbeWindowMs, FREEINK_CAP_FRONTLIGHT_PROBE),
// for a light that may not be fitted: every enable (a switch-on, or a colour change
// while lit) reads the monitor every loadProbePeriodMs for loadProbeWindowMs after
// the enable pulse, on the monitor's timer, so the call that lit it returns at
// once and probing() stays true until the switch-on has decided. Any reading above
// the trip there means the selected string has no load: the light goes dark at
// once. With a colour select the other string is then tried, once the LED rail
// allows; if it lights, the first is recorded missing and never selected again
// until a cold boot (a later no-light result keeps the mark), and if it does not
// (or there is none), load() reads Absent and fault() latches as for a trip. A
// second try the rail blocks latches fault() with the record unchanged. While a
// switch-on decides, brightness changes apply at once and a colour change waits
// for its result. The record (load(), stringMissing()) is kept in RTC memory
// across deep sleep and software restarts. The consumer enables such a light only
// when the user asks, or at boot once restoreAllowed() agrees. After any trip the
// colour select moves only once the LED rail has fallen to railSafeV, so the
// charge left on the boost output cannot discharge through a string on the other
// side.

#include <Arduino.h>
#include <BoardConfig.h>
#include <FrontlightLoad.h>
#if FREEINK_CAP_FRONTLIGHT
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#if FREEINK_CAP_FRONTLIGHT_PROBE
#include <atomic>
#endif

class FrontlightManager {
 public:
  // Bring up the PWM channel(s). No-op if the board has no frontlight.
  void begin();

  // Set brightness as a 0-100 percentage, mapped to duty through a perceptual
  // gamma-1.6554 curve (1% is the smallest non-zero duty step, not 1% linear
  // duty). 0 turns the light off. On a warm/cool board this is the TOTAL
  // brightness; the current color-temperature split is preserved.
  void setBrightness(uint8_t percent);

  // Set brightness with 8-bit control and a perceptual curve. Level 1 maps to the
  // smallest non-zero hardware duty, making dim night reading possible.
  void setBrightnessLevel(uint8_t level);

  // Convenience: fully off / restore last brightness.
  void off();
  void on();

  // Cut frontlight leakage through deep sleep: drive the LED pads LOW and hold
  // them (so the level survives deep sleep via gpio_deep_sleep_hold_en), and
  // release the LEDC KEEP_ALIVE clock. Call from the consumer's sleep path just
  // before deep sleep. Only meaningful on LEDC frontlights (no-op otherwise).
  // releaseOnWake() must be called at boot before begin() re-attaches the
  // channels.
  // Implemented only under FREEINK_FRONTLIGHT_LS, so guard the declarations to
  // match (otherwise boards without it get an undefined reference at link time).
#ifdef FREEINK_FRONTLIGHT_LS
  void park();

  // Undo park() at boot so begin() can re-attach the LEDC channels. The release is
  // UNCONDITIONAL: park() latches a digital pad hold that survives deep sleep AND
  // the wake reset, while _lsParked (a DRAM flag) is lost on reset — so after a
  // wake the hold is still present even though _lsParked reads false. Releasing
  // unconditionally (gpio_hold_dis on a non-held pad is a harmless no-op) is the
  // only way to guarantee the held pad is cleared; every other driver releases
  // holds unconditionally before driving for this reason.
  void releaseOnWake();
#endif

  // Warm/cool mix, 0 = fully cool, 100 = fully warm, 50 = neutral. Only meaningful on a
  // two-channel board (hasColorTemperature()); a no-op on single-channel frontlights.
  // On a colour-select board it blends while blendsColour(), and is otherwise a fixed
  // choice: warm at >= 50, cool below.
  void setColorTemperature(uint8_t warmPercent);

  // A colour-select light whose warmth between the ends blends the two strings
  // (FrontlightConfig::colorBlendHz, its carrier attached, neither string recorded
  // missing). False on other lights, and on a light that uses a fixed selection.
  bool blendsColour() const;
  // The colour select runs its blend carrier now (lit). warmSelected() then names
  // the dominant string.
  bool blending() const {
#if FREEINK_CAP_FRONTLIGHT
    return _blending;
#else
    return false;
#endif
  }

  // Before deep sleep on an enable-gated frontlight: stop the LED monitor, take the
  // light dark (PWM pin held inactive as a plain GPIO, LEDC duty 0, colour select at
  // its cool rest level) and detach LEDC, so the board's sleep park holds pads that
  // no peripheral drives. The light stays off until begin() runs on the next boot.
  // A no-op on other frontlights.
  void prepareForSleep();

  // True once the LED-rail monitor (FrontlightConfig::monitorAdc) has switched the
  // light off for over-voltage, or a switch-on found no light (load() == Absent).
  // While set, the light stays dark at any brightness. Switching the light off
  // (brightness 0) or clearFault() re-arms it; the next non-zero brightness then
  // runs the full enable sequence.
  bool fault() const { return _fault; }
  void clearFault() { _fault = false; }

  // A switch-on (or a colour change while lit) is still deciding its load: its
  // probe window, or the wait for the LED rail before the other string. load(),
  // stringMissing() and fault() then still describe the last decided one. Any
  // task may call it. Always false without FREEINK_CAP_FRONTLIGHT_PROBE.
  bool probing() const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
    return _probing.load(std::memory_order_acquire);
#else
    return false;
#endif
  }

  // What the load probe found (see the top of this file). Unknown, and no string
  // missing, without FREEINK_CAP_FRONTLIGHT_PROBE or before any probe since a
  // cold boot.
  freeink::frontlight::Load load() const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
    return _loadRecord.load;
#else
    return freeink::frontlight::Load::Unknown;
#endif
  }
  // A single-colour light: `warm`'s string tripped the probe while the other lit.
  // Colour requests for it light the other string.
  bool stringMissing(bool warm) const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
    return _loadRecord.string(warm) == freeink::frontlight::StringState::Missing;
#else
    (void)warm;
    return false;
#endif
  }
  // A consumer restoring its saved "on" at boot (Restore Light on Wake) asks this
  // first: false while the record says no light answered the last switch-on.
  bool restoreAllowed() const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
    return freeink::frontlight::restoreAllowed(_loadRecord);
#else
    return true;
#endif
  }

#if FREEINK_DIAGNOSTICS
  // LED-rail monitor record (all zero without FREEINK_CAP_FRONTLIGHT). Unlike
  // fault(), it survives the light being switched off after a trip, so a test
  // can see a trip the consumer has already handled. mV values are on the LED
  // rail (pin mV x monitorScaleX1000 / 1000) unless named "pin".
  struct MonitorDiagnostics {
    uint32_t trips = 0;            // trips since boot
    uint32_t lastTripLedMv = 0;    // LED rail at the last trip
    uint32_t lastTripMs = 0;       // millis() of the last trip
    uint32_t lastTripLimitMv = 0;  // monitorTripMv in force then
    uint32_t samples = 0;          // monitor readings taken since boot
    uint32_t lastPinMv = 0;        // last monitor reading at the ADC pin
    uint32_t lastLedMv = 0;        // the same reading on the LED rail
    uint32_t lastSampleMs = 0;     // millis() of that reading
    uint32_t skippedSamples = 0;   // ticks that could not take the ADC in time
  };
  MonitorDiagnostics monitorDiagnostics() const;

  // Load-probe record (all zero without FREEINK_CAP_FRONTLIGHT_PROBE). Probe trips
  // are counted here, not in MonitorDiagnostics::trips, which stays the in-use
  // trips. mV values are on the LED rail.
  struct ProbeDiagnostics {
    // load() and both strings, as they are now
    freeink::frontlight::LoadRecord record;
    uint32_t probes = 0;           // strings probed at a switch-on since boot
    uint32_t trips = 0;            // of those, the ones that read above the trip
    uint32_t secondTries = 0;      // the other string tried after a trip
    uint32_t absent = 0;           // switch-ons that ended with no light
    bool lastWarm = false;         // the string of the last probe
    bool lastTripped = false;      // it read above the trip
    uint32_t lastTripAfterUs = 0;  // carrier start to that reading
    // that reading to the PWM pad held at its dark level (ADIM LOW), 0 when it lit
    uint32_t lastDarkAfterTripUs = 0;
    uint32_t lastPeakLedMv = 0;   // highest reading in the last probe window
    uint32_t lastSamples = 0;     // readings in the last probe window
    uint32_t lastMs = 0;          // millis() of the last probe's end
    uint32_t railWaits = 0;       // colour-select moves that waited for the rail after a trip
    uint32_t lastRailWaitMs = 0;  // how long the last one waited
    uint32_t lastRailMv = 0;      // the rail when it ended
    uint32_t railTimeouts = 0;    // waits that ran out
  };
  ProbeDiagnostics probeDiagnostics() const;
#endif

  bool present() const {
#if FREEINK_CAP_FRONTLIGHT
    // A PMIC-driven frontlight (Paper Mono: PM1 PWM0 -> AW9967) has no ESP
#if FREEINK_DEVICE_EEGO_A4
    // An optional EEGO A4 LM3630A is present only after begin() gets an ACK.
    if (BoardConfig::ACTIVE.board == BoardConfig::Board::EegoA4 && BoardConfig::hasI2cFrontlight()) return _begun;
#endif
#if FREEINK_DEVICE_PICCO
    // Picco's dual-channel I2C LED driver is present only after begin() gets an ACK.
    if (BoardConfig::ACTIVE.board == BoardConfig::Board::Picco && BoardConfig::hasI2cFrontlight()) return _begun;
#endif
    // GPIO, so viaPm1Pwm counts as present alongside the LEDC-pin boards.
    return BoardConfig::ACTIVE.frontlight.gpio != BoardConfig::PIN_UNASSIGNED ||
           BoardConfig::ACTIVE.frontlight.viaPm1Pwm;
#else
    return false;  // frontlight code not compiled in (FREEINK_CAP_FRONTLIGHT=0)
#endif
  }

  // True when the board wires a second (warm) channel, so setColorTemperature() does
  // something. False on single-channel frontlights and on boards with none.
  bool hasColorTemperature() const {
#if FREEINK_DEVICE_EEGO_A4 && FREEINK_CAP_FRONTLIGHT
    if (BoardConfig::ACTIVE.board == BoardConfig::Board::EegoA4) {
      return present() && BoardConfig::hasColorTemperatureFrontlight();
    }
#endif
#if FREEINK_DEVICE_PICCO && FREEINK_CAP_FRONTLIGHT
    if (BoardConfig::ACTIVE.board == BoardConfig::Board::Picco) {
      return present() && BoardConfig::hasColorTemperatureFrontlight();
    }
#endif
#if FREEINK_CAP_WARMLIGHT
    return BoardConfig::ACTIVE.frontlight.gpio != BoardConfig::PIN_UNASSIGNED &&
           (BoardConfig::ACTIVE.frontlight.gpioWarm != BoardConfig::PIN_UNASSIGNED ||
            BoardConfig::ACTIVE.frontlight.colorSelectGpio != BoardConfig::PIN_UNASSIGNED);
#else
    return false;  // no warm-channel board in this build (FREEINK_CAP_WARMLIGHT=0)
#endif
  }

  uint8_t brightness() const { return _brightness; }
  uint8_t brightnessLevel() const { return _brightnessLevel; }
  uint8_t colorTemperature() const { return _warmPercent; }
  // On a colour-select light: the string the select picks now (the lit one, or
  // where it rests while dark), which differs from colorTemperature() when a
  // string is missing.
  bool warmSelected() const {
#if FREEINK_CAP_FRONTLIGHT
    return _warmSelected;
#else
    return false;
#endif
  }

 private:
#if FREEINK_CAP_FRONTLIGHT
  // Recompute and write both channels from _brightness + _warmPercent.
  void apply();
  // Enable-gated path (see the top of this file). _gateLock serialises these
  // against the monitor, which runs on the esp_timer task.
  void applyGated(uint32_t duty, uint32_t full);
  void goDark(uint32_t full);
  // secondTry: the other string, after `warm`'s opposite tripped in the same switch-on.
  // blend: start the blend carrier with the light (both strings lit since the reset).
  void lightUp(uint32_t duty, uint32_t full, bool warm, bool secondTry = false, bool blend = false);
  void waitDarkDwell() const;
  // Holds the colour select on one string, taking it off the blend carrier first.
  void selectWarm(bool warm);
  // Puts the colour select on its blend carrier at the current warmth (lit only).
  void blendColour(uint32_t full);
  freeink::frontlight::ColourPlan colourPlan() const;
  // Lit on a string that has lit since the reset, on the way to a blend whose other
  // string has not: check that one while lit (FrontlightConfig::blendConfirmWindowMs).
  bool confirmsWhileLit(const freeink::frontlight::ColourPlan& plan) const;
  void switchLitToConfirm(uint32_t duty, uint32_t full, bool warm);
  void startMonitor(uint64_t periodUs);
  void stopMonitor();
  void checkMonitor();
  static void monitorTick(void* self);
#if FREEINK_CAP_FRONTLIGHT_PROBE
  // Load probe (see the top of this file). Callers hold _gateLock; the ticks run
  // on the monitor's timer.
  enum class ProbePhase : uint8_t {
    Idle,
    Window,    // lit on `warm`, reading the monitor every loadProbePeriodMs
    RailWait,  // dark, waiting for the rail before the colour select moves to `warm`
  };
  struct Probe {
    ProbePhase phase = ProbePhase::Idle;
    bool warm = false;       // the string being probed, or the one to light after the rail wait
    bool secondTry = false;  // the other string, after the first tripped in this switch-on
    bool blend = false;      // the window runs on the blend (both strings lit since the reset)
    uint16_t windowMs = 0;   // loadProbeWindowMs, or blendConfirmWindowMs for a string switched to while lit
    uint32_t duty = 0;       // the duty to light at (brightness changes while deciding land here)
    int64_t startUs = 0;     // carrier start (Window) or wait start (RailWait)
    uint32_t peakLedMv = 0;
    uint32_t samples = 0;
  };
  void beginWindow(uint32_t duty, bool warm, bool secondTry, bool blend, uint16_t windowMs);
  void beginRailWait(uint32_t duty, bool warm, bool secondTry);
  void endProbe();
  // endProbe() for a switch-off or sleep entry: a window on a string not yet lit
  // since the reset leaves the rail counted as charged.
  void abandonProbe();
  void probeTick();
  void railWaitTick();
  void probeTripped(int64_t tripAtUs);
  void probePassed();
  void noteProbeEnd(bool tripped, int64_t tripAtUs);
  void noteRailWait(int64_t sinceUs, bool safe);
  void restColourSelectAfterTrip();
  void markRailCharged();
  bool railSafeNow();
  bool waitRailSafe();
  void restoreRecord();
  void saveRecord() const;
  freeink::frontlight::LoadRecord _loadRecord;
  Probe _probe;
  std::atomic<bool> _probing{false};  // _probe.phase != Idle, for probing()
  bool _railCharged = false;          // a trip left the boost output charged
  int64_t _railChargedAtUs = 0;       // esp_timer time of that trip
  // Each string lit through a probe since the chip last reset (RAM, so a wake
  // re-confirms a light that may have been changed while asleep); counts only while
  // the record still says present. A blend needs both.
  bool _litSinceReset[2] = {false, false};  // [cool, warm]
  void noteStringLit(bool warm) { _litSinceReset[warm ? 1 : 0] = true; }
  bool litSinceReset(bool warm) const {
    return _litSinceReset[warm ? 1 : 0] && _loadRecord.string(warm) == freeink::frontlight::StringState::Present;
  }
#if FREEINK_DIAGNOSTICS
  ProbeDiagnostics _probeDiag;  // written under _gateLock
#endif
#endif
  bool _gatedLit = false;      // PWM pin is on its LEDC carrier (gated path)
  bool _warmSelected = false;  // colour select is at the warm level (blending: the dominant string)
  bool _colourCarrier = false;  // the colour select has a blend carrier attached (colorBlendHz)
  bool _blending = false;       // the colour select is on that carrier
  int64_t _darkSinceUs = 0;    // esp_timer time the light last went dark; 0 = since boot
  esp_timer_handle_t _monitorTimer = nullptr;
  SemaphoreHandle_t _gateLock = nullptr;
#if FREEINK_DEVICE_EEGO_A4
  // LM3630A (I2C) frontlight helpers — see FrontlightManager.cpp.
  bool lm3630aWrite(uint8_t reg, uint8_t value);
  bool lm3630aRead(uint8_t reg, uint8_t& value);
  bool lm3630aUpdate(uint8_t reg, uint8_t mask, uint8_t value);
  bool configureLm3630a();
  void applyLm3630a();
#endif
#if FREEINK_DEVICE_PICCO
  // Picco dual-channel warm/cool I2C LED driver helpers — see FrontlightManager.cpp.
  bool piccoWrite(uint8_t reg, uint8_t value);
  bool piccoUpdate(uint8_t reg, uint8_t mask, uint8_t value);
  bool configurePicco();
  void applyPicco();
#endif
#endif
#ifdef FREEINK_FRONTLIGHT_LS
  // Keep RC_FAST powered through light sleep only while the light is actually
  // lit. The LEDC driver's KEEP_ALIVE config pins RC_FAST (and the digital
  // domain at its higher sleep bias) for every light-sleep window from boot;
  // begin() cancels that via the refcounted sleep sub-mode API and apply()
  // re-arms it on 0<->nonzero total-duty transitions, so dark idle sleeps at
  // full depth.
  void updateLsKeepAlive(bool lit);
  bool _lsAttachOk = false;        // both channel attaches succeeded (refcount is balanced)
  bool _lsKeepAliveArmed = false;  // our own +1 on the RC_FAST sleep sub-mode is active
  bool _lsParked = false;          // park() has driven + held the frontlight pads LOW
#endif

  bool _begun = false;
  volatile bool _fault = false;  // set by the LED monitor on the esp_timer task
#if FREEINK_DIAGNOSTICS && FREEINK_CAP_FRONTLIGHT
  MonitorDiagnostics _monitorDiag;  // written by checkMonitor() under _gateLock
#endif
#if FREEINK_DEVICE_EEGO_A4 || FREEINK_DEVICE_PICCO
  bool _i2cConfigured = false;
#endif
  uint8_t _brightness = 0;
  uint8_t _brightnessLevel = 0;
  bool _useLevel = false;
  uint8_t _lastBrightness = 50;
  uint8_t _warmPercent = 50;  // neutral by default
};
