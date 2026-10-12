#pragma once

#include <cstdint>

// FreeInk SDK — deep-sleep / wake power management.
//
// Owns the one hardware concern the rest of the SDK leaves to the consumer: the
// per-SoC deep-sleep GPIO-wakeup difference. RISC-V parts (C3/C6/H2) wake from
// deep sleep via the "gpio" source (esp_deep_sleep_enable_gpio_wakeup); Xtensa
// parts (S3/S2, classic ESP32) wake via RTC ext1 (esp_sleep_enable_ext1_wakeup).
// Hardcoding either one blocks a multi-MCU build (see docs/consumer-mcu-portability.md).
//
// This picks the right source at compile time from SoC capability macros, and
// reads the wake pin + active level from BoardConfig::ACTIVE.input, so the same
// consumer code deep-sleeps correctly on every supported board. The wake pin must
// be RTC-capable on ext1 parts (true for the de-link power button).

namespace freeink {

class PowerManager {
 public:
  // Arm wake-on-power-button using the SoC-correct wakeup source and the active
  // board's power pin + polarity (powerActiveHigh -> wake on HIGH, else LOW).
  // Returns false if the board has no power pin (PIN_UNASSIGNED); nothing armed.
  static bool armPowerButtonWakeup();

  // Arm deep-sleep wake on an arbitrary set of GPIOs (gpioMask, wakeLow = wake on
  // the low level) using the SoC-correct source (ext1 on Xtensa, gpio on RISC-V).
  // Use for extra wake lines beyond the power button — a touch INT, a second
  // button, an IO-expander INT. The pins must be RTC-capable on ext1 parts.
  static void armWakeOnPins(uint64_t gpioMask, bool wakeLow = true);

  // Poll the power-button GPIO (raw read, with the matching pull) until released,
  // so deep sleep isn't immediately cancelled by a still-held press. Gives up
  // after PowerConfig::releaseWaitMaxMs when that is set; returns false if the
  // button was still pressed then.
  static bool waitForPowerButtonRelease();

  // Drive every assigned peripheral power-rail enable in the active board
  // profile (display / SD / touch / mic) to its OFF level and latch it with
  // gpio_hold_en() so the load switches stay off through deep sleep (deepSleep()
  // enables gpio_deep_sleep_hold_en(), which makes the holds persist). Without
  // this, boards with gated rails (e.g. Sticky: GT911 on TP_PWR_EN, SD on
  // SD_PWR_EN, EPD on EP_PWR_EN) leave those peripherals powered all through
  // deep sleep — milliamps of standby drain. No-op on boards whose rails are
  // PIN_UNASSIGNED (X4/X3). Call after the display driver's deep-sleep command
  // and before deepSleep(); wake is a chip reset, so rails re-enable in the
  // normal init path. Display RESET is held LOW when its rail is cut (avoids
  // back-powering an unpowered controller) and HIGH when its rail remains on
  // (keeps deep-sleep state stable). NOTE: cutting the touch rail forfeits
  // touch-to-wake. Also applies the profile's parkedPins sleep levels, takes the
  // awake-only BUSY pulls off (DisplayPins::busyPullDown, busyPullUp), and holds
  // a parkLowWhenOff SDMMC bus LOW.
  static void powerDownRailsForSleep();

  // Put an SSD1677 panel on an always-on rail into deep sleep without its driver,
  // for a sleep entry that must not run the display code (a reset loop): pulse
  // RESET (analog and clock off, the power-on state), wait for BUSY (bounded),
  // then send deep sleep 0x10 / 0x03 bit-banged on the display pins, leaving CS
  // HIGH and SCLK/MOSI/DC LOW. With FREEINK_CAP_DISPLAY_PROBE_POLICY, an
  // UltraChip controller (UC8279/UC8179, as ACTIVE names it once the probe has
  // run) gets its own: BUSY_N waited HIGH, then DSLP 0x07 / 0xA5. On the
  // Silkscreen 3.68" variant, whose panel is the X3's UC8279d, fixed in its
  // profile, it settles 60 ms after the reset releases, as that driver does,
  // instead of 10. Returns false, doing nothing, unless the profile declares one
  // of those controllers with RESET, CS, SCLK, MOSI and DC and no display rail
  // switch (a switched rail is simply cut).
  static bool sleepPanelWithoutDriver();

  // Isolate floating GPIOs to cut sleep current, then enter deep sleep. Does not
  // return — the chip resets on wake.
  [[noreturn]] static void deepSleep();

  // Convenience: wait for release, arm the power-button wakeup, then deep sleep.
  // With PowerConfig::releaseWaitMaxMs set, a button still pressed after that
  // long arms the wake on its release instead (the only wake source, so a stuck
  // key costs a sleeping chip, not a spinning one); takeReleaseWake() then tells
  // the next boot to sleep again at once. After checkPowerPinIdle() found the
  // pin between the logic levels, nothing is armed and the pad keeps its input
  // off through sleep: only a reset wakes the chip.
  [[noreturn]] static void deepSleepUntilPowerButton();

  // PowerConfig::powerPinIdleCheck (with FREEINK_CAP_POWER_POLICY). What the
  // power pin read at this boot's check: Low (released), High (held at boot), or
  // Undefined, between the logic levels, so the ext1 wake would fire at once or
  // never. NotChecked: a deep-sleep wake, no power pin, or the policy off.
  enum class PowerPinIdle : uint8_t { NotChecked, Low, High, Undefined };
  struct PowerPinCheck {
    PowerPinIdle idle = PowerPinIdle::NotChecked;
    int8_t pin = -1;  // the profile's power pin
    uint16_t mv = 0;  // the last of the averaged readings
  };
  // Reads the power pin on the ADC (radio off; ADC2 on an S3 IO18) at any reset
  // but a deep-sleep wake, before anything arms a wake. On Undefined it leaves
  // the pad as the ADC set it (input and pulls off) and sets
  // ACTIVE.input.power to PIN_UNASSIGNED for the session; otherwise it puts the
  // pin back to its input mode. Call once, early in setup().
  static PowerPinCheck checkPowerPinIdle();
  // This boot's result (NotChecked before checkPowerPinIdle()).
  static PowerPinCheck powerPinCheck();

  // True once, on the boot that follows a release wake (see
  // deepSleepUntilPowerButton()). The consumer calls it first thing and, when
  // true, sleeps again before starting any peripheral: the button went up, it
  // did not ask for a wake. Clears the record.
  static bool takeReleaseWake();

  // Consecutive abnormal resets, kept in RTC memory across resets: a panic, a
  // watchdog or CPU lockup, a brown-out, or a deep-sleep entry that aborted
  // (abortedSleep, from takeAbortedSleepInfo()). Any other reset starts the count
  // again. Call once per boot, early; returns the count including this boot. A
  // consumer compares it with PowerConfig::abnormalResetLimit.
  static uint8_t countAbnormalResets(bool abortedSleep);
  // Forget the count, once a boot has proven healthy or before sleeping on it.
  static void clearAbnormalResets();

  // esp_deep_sleep_start() is documented as not returning, but deepSleep()
  // calls esp_restart() if it ever does (instead of a busy-loop -- see that
  // function). This reports whether the *previous* boot took that abort
  // path, and clears the record so it only surfaces once. wakeupCause/
  // wakePinLevel are the values captured at the moment of the abort.
  struct AbortedSleepInfo {
    bool aborted = false;
    int wakeupCause = 0;
    int wakePinLevel = 0;
  };
  static AbortedSleepInfo takeAbortedSleepInfo();

#if defined(FREEINK_DIAGNOSTICS) && FREEINK_DIAGNOSTICS
  // Non-consuming reads for diagnostics. peekAbnormalResets() is the count in
  // RTC memory now (0 before the first countAbnormalResets() since power-on,
  // and after clearAbnormalResets()); peekAbortedSleepInfo() is what
  // takeAbortedSleepInfo() would return now.
  static uint8_t peekAbnormalResets();
  static AbortedSleepInfo peekAbortedSleepInfo();

  // What this boot's countAbnormalResets() and takeAbortedSleepInfo() returned,
  // kept after the consumer has cleared the count or taken the record. An
  // aborted record, once taken, stays reported even if a later take returns
  // none.
  struct BootResetRecord {
    bool counted = false;        // countAbnormalResets() ran this boot
    uint8_t abnormalResets = 0;  // its result
    bool abortedSleepTaken = false;
    AbortedSleepInfo abortedSleep;
  };
  static BootResetRecord bootResetRecord();
#endif
};

}  // namespace freeink
