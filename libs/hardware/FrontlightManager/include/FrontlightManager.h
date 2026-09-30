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

#include <Arduino.h>
#include <BoardConfig.h>
#if FREEINK_CAP_FRONTLIGHT
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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
  // On a colour-select board it is a fixed choice: warm at >= 50, cool below.
  void setColorTemperature(uint8_t warmPercent);

  // Before deep sleep on an enable-gated frontlight: stop the LED monitor, take the
  // light dark (PWM pin held inactive as a plain GPIO, LEDC duty 0, colour select at
  // its cool rest level) and detach LEDC, so the board's sleep park holds pads that
  // no peripheral drives. The light stays off until begin() runs on the next boot.
  // A no-op on other frontlights.
  void prepareForSleep();

  // True once the LED-rail monitor (FrontlightConfig::monitorAdc) has switched the
  // light off for over-voltage. While set, the light stays dark at any brightness.
  // Switching the light off (brightness 0) or clearFault() re-arms it; the next
  // non-zero brightness then runs the full enable sequence.
  bool fault() const { return _fault; }
  void clearFault() { _fault = false; }

  bool present() const {
#if FREEINK_CAP_FRONTLIGHT
    // A PMIC-driven frontlight (Paper Mono: PM1 PWM0 -> AW9967) has no ESP
#if FREEINK_DEVICE_EEGO_A4
    // An optional EEGO A4 LM3630A is present only after begin() gets an ACK.
    if (BoardConfig::ACTIVE.board == BoardConfig::Board::EegoA4 && BoardConfig::hasI2cFrontlight()) return _begun;
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

 private:
#if FREEINK_CAP_FRONTLIGHT
  // Recompute and write both channels from _brightness + _warmPercent.
  void apply();
  // Enable-gated path (see the top of this file). _gateLock serialises these
  // against the monitor, which runs on the esp_timer task.
  void applyGated(uint32_t duty, uint32_t full);
  void goDark(uint32_t full);
  void lightUp(uint32_t duty, uint32_t full, bool warm);
  void waitDarkDwell() const;
  void selectWarm(bool warm);
  void startMonitor();
  void stopMonitor();
  void checkMonitor();
  static void monitorTick(void* self);
  bool _gatedLit = false;      // PWM pin is on its LEDC carrier (gated path)
  bool _warmSelected = false;  // colour select is at the warm level
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
#if FREEINK_DEVICE_EEGO_A4
  bool _i2cConfigured = false;
#endif
  uint8_t _brightness = 0;
  uint8_t _brightnessLevel = 0;
  bool _useLevel = false;
  uint8_t _lastBrightness = 50;
  uint8_t _warmPercent = 50;  // neutral by default
};
