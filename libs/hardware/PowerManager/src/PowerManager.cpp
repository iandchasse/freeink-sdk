#include "PowerManager.h"

#include <Arduino.h>
#include <BoardConfig.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <soc/soc_caps.h>
#if FREEINK_DEVICE_WS397
#include <Axp2101.h>
#endif

namespace freeink {
namespace {
int8_t powerPin() { return BoardConfig::ACTIVE.input.power; }
bool powerActiveHigh() { return BoardConfig::ACTIVE.input.powerActiveHigh; }
// Input mode for the power pin: the pull opposite its active level, or none when
// the board biases the pin externally (InputPins::powerExternalPull).
uint8_t powerPinMode() {
  if (BoardConfig::ACTIVE.input.powerExternalPull) return INPUT;
  return powerActiveHigh() ? INPUT_PULLDOWN : INPUT_PULLUP;
}

// Records a deepSleep() abort (see deepSleep()) across the esp_restart() that
// follows it, so the next boot can report it via takeAbortedSleepInfo().
// RTC_NOINIT_ATTR survives esp_restart() (a warm reset, same RTC-memory domain)
// but is garbage on a cold boot -- magic-guarded like main.cpp's silentRebootMagic.
constexpr uint32_t ABORTED_SLEEP_MAGIC = 0x41424f52;  // 'ABOR'

struct AbortedSleepRecord {
  bool aborted;
  int wakeupCause;
  int wakePinLevel;
};

// Set just before a sleep that wakes on the power button's release
// (PowerConfig::releaseWaitMaxMs); read and cleared by takeReleaseWake().
constexpr uint32_t RELEASE_WAKE_MAGIC = 0x52454c57;  // 'RELW'
// Guards the consecutive abnormal-reset count (countAbnormalResets()).
constexpr uint32_t ABNORMAL_RESETS_MAGIC = 0x41425253;  // 'ABRS'

// The power pin as a deep-sleep wake input: the pull opposite its active level,
// or both pulls off in the digital and the RTC domain when the board biases the
// pin itself (the RTC pad is what ext1 samples).
void preparePowerPinForSleep(const int8_t pin) {
  pinMode(pin, powerPinMode());
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
  if (BoardConfig::ACTIVE.input.powerExternalPull && rtc_gpio_is_valid_gpio(static_cast<gpio_num_t>(pin))) {
    rtc_gpio_pullup_dis(static_cast<gpio_num_t>(pin));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(pin));
  }
#endif
}
}  // namespace

RTC_NOINIT_ATTR uint32_t abortedSleepMagic;
RTC_NOINIT_ATTR AbortedSleepRecord abortedSleepRecord;
RTC_NOINIT_ATTR uint32_t releaseWakeMagic;
RTC_NOINIT_ATTR uint32_t abnormalResetsMagic;
RTC_NOINIT_ATTR uint32_t abnormalResets;
#if FREEINK_DIAGNOSTICS
// This boot's reads of the two records above (bootResetRecord()); plain RAM.
static PowerManager::BootResetRecord bootRecord;
#endif

void PowerManager::armWakeOnPins(uint64_t gpioMask, bool wakeLow) {
#if SOC_PM_SUPPORT_EXT1_WAKEUP
  // Xtensa (S3/S2, classic ESP32): RTC ext1. Pins must be RTC GPIOs.
  //
  // The classic ESP32 RTC has no "any low" mode — only ESP_EXT1_WAKEUP_ALL_LOW
  // ("wake when ALL selected pins are low"). For a single wake pin (the common
  // power-button case) ALL_LOW and ANY_LOW are identical; a multi-pin low wake on
  // classic ESP32 fires only when every pin is low. S2/S3 expose ANY_LOW directly.
#if defined(CONFIG_IDF_TARGET_ESP32)
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ALL_LOW;
#else
  const esp_sleep_ext1_wakeup_mode_t lowMode = ESP_EXT1_WAKEUP_ANY_LOW;
#endif
  esp_sleep_enable_ext1_wakeup(gpioMask, wakeLow ? lowMode : ESP_EXT1_WAKEUP_ANY_HIGH);
#elif SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
  // RISC-V (C3/C6/H2): the deep-sleep "gpio" wakeup source.
  esp_deep_sleep_enable_gpio_wakeup(gpioMask, wakeLow ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
#error "FreeInk PowerManager: target has no supported deep-sleep GPIO wakeup source"
#endif
}

bool PowerManager::armPowerButtonWakeup() {
  const int8_t pin = powerPin();
  if (pin < 0) return false;
  // Hold the idle level with the opposite pull so the line is defined in sleep,
  // unless the board's own pull already defines it.
  preparePowerPinForSleep(pin);
  armWakeOnPins(1ULL << pin, /*wakeLow=*/!powerActiveHigh());
  return true;
}

bool PowerManager::waitForPowerButtonRelease() {
  const int8_t pin = powerPin();
  if (pin < 0) return true;
  const bool activeHigh = powerActiveHigh();

  pinMode(pin, powerPinMode());
  const int pressedLevel = activeHigh ? HIGH : LOW;
  const uint16_t maxMs = BoardConfig::powerPolicy().releaseWaitMaxMs;
  const uint32_t start = millis();
  while (digitalRead(pin) == pressedLevel) {
    if (maxMs != 0 && millis() - start >= maxMs) return false;
    delay(50);
  }
  return true;
}

namespace {
// Drive a rail-enable pin to `offLevel` and latch it so the level survives deep
// sleep (requires gpio_deep_sleep_hold_en(), done in deepSleep()). gpio_hold_dis
// first: a hold left over from a previous cycle would make the writes no-ops.
// The level is preset before the output is enabled (pinMode keeps GPIO_OUT), so
// a pin whose output register still holds its reset 0 (e.g. an EPD RST on a
// sleep path taken before the display was begun) never glitches LOW.
void holdRailOff(int8_t pin, uint8_t offLevel) {
  if (pin < 0) return;
  const auto g = static_cast<gpio_num_t>(pin);
  gpio_hold_dis(g);
  gpio_set_level(g, offLevel);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offLevel);
  gpio_hold_en(g);
}
}  // namespace

void PowerManager::powerDownRailsForSleep() {
  const auto& b = BoardConfig::ACTIVE;
#if FREEINK_DEVICE_WS397
  // The EPD rail is an AXP2101 LDO, not a GPIO, so holdRailOff() below cannot
  // reach it — drop it here or the panel stays powered all through deep sleep.
  axp2101::setEpdPower(false);
#endif
  // Keep RESET defined through deep sleep, but never drive an unpowered panel's
  // input HIGH: on boards with a gated EPD rail (Sticky), that can back-power the
  // controller through its RESET protection diode and turn sleep into a
  // milliamp-level drain. Hold RESET LOW alongside a switched-off rail. Boards
  // whose panel rail remains powered (X4 Pro) keep RESET HIGH so a UC8179 cannot
  // drift out of DSLP and restart its analog booster. EpdBus and XteinkDetect
  // release the hold before issuing a reset pulse on wake.
  const uint8_t resetSleepLevel = b.display.powerEnable >= 0 ? LOW : HIGH;
  holdRailOff(b.display.rst, resetSleepLevel);
  holdRailOff(b.display.powerEnable, LOW);
  // The BUSY pulls (DisplayPins::busyPullDown, busyPullUp) are for the awake panel
  // only. A controller that drives BUSY HIGH through its deep sleep (SSD1677)
  // would sink into a pull-down all night, and one that drives BUSY_N LOW into a
  // pull-up, so BUSY sleeps as an input with no pulls, never held.
  if ((b.display.busyPullDown || BoardConfig::displayBusyPullUp()) && b.display.busy >= 0) {
    const auto busy = static_cast<gpio_num_t>(b.display.busy);
    gpio_hold_dis(busy);
    gpio_pulldown_dis(busy);
    gpio_pullup_dis(busy);
    gpio_set_direction(busy, GPIO_MODE_INPUT);
  }
  // SdmmcPins::parkLowWhenOff: the bus goes LOW with pulls off before the card is
  // gated off, and stays held LOW through sleep. An unheld RTC pad is undefined in
  // deep sleep, and a bus line that floats HIGH back-feeds the gated card through
  // its pull-ups. The gate itself is re-held just below; holdPowerRails() releases
  // the bus at boot.
  if (FREEINK_SD_SDMMC && b.sdmmc.parkLowWhenOff && b.sdmmc.busWidth != 0) {
    BoardConfig::parkSdmmcBus();
    for (const int8_t pin : {b.sdmmc.clk, b.sdmmc.cmd, b.sdmmc.d0, b.sdmmc.d1, b.sdmmc.d2, b.sdmmc.d3}) {
      if (pin >= 0) gpio_hold_en(static_cast<gpio_num_t>(pin));
    }
  }
  // SD enable OFF = the inactive level: LOW for active-high enables, HIGH for the
  // active-low ones (e.g. X4 Pro's GPIO5, which powers the card while held LOW).
  holdRailOff(b.sd.powerEnable, b.sd.powerActiveHigh ? LOW : HIGH);
  holdRailOff(b.touch.powerEnable, b.touch.powerEnableActiveHigh ? LOW : HIGH);
  // The mic enable also carries a polarity flag; OFF is the inactive level.
  holdRailOff(b.mic.enable, b.mic.enableActiveHigh ? LOW : HIGH);
  // Board-declared sleep levels (BoardConfig::ParkedPin), e.g. an EPD CS with no
  // pull-up held HIGH, or a touch reset held LOW on an unswitched rail.
  for (const BoardConfig::ParkedPin& p : b.parkedPins) {
    if (p.pin < 0) continue;
    switch (p.sleep) {
      case BoardConfig::PinPark::Low:
        holdRailOff(p.pin, LOW);
        break;
      case BoardConfig::PinPark::High:
        holdRailOff(p.pin, HIGH);
        break;
      case BoardConfig::PinPark::InputNoPull:
        gpio_hold_dis(static_cast<gpio_num_t>(p.pin));
        pinMode(p.pin, INPUT);
        gpio_hold_en(static_cast<gpio_num_t>(p.pin));
        break;
      case BoardConfig::PinPark::None:
        break;
    }
  }
}

namespace {
// One SPI mode-0 byte, MSB first, on plain GPIOs (sleepPanelWithoutDriver()).
void bitBangByte(const gpio_num_t sclk, const gpio_num_t mosi, const uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    gpio_set_level(mosi, (value & mask) != 0 ? 1 : 0);
    delayMicroseconds(1);
    gpio_set_level(sclk, 1);
    delayMicroseconds(1);
    gpio_set_level(sclk, 0);
  }
}

void driveLevel(const int8_t pin, const uint8_t level) {
  const auto g = static_cast<gpio_num_t>(pin);
  gpio_hold_dis(g);
  gpio_set_level(g, level);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, level);
}
}  // namespace

bool PowerManager::sleepPanelWithoutDriver() {
  const BoardConfig::DisplayPins& d = BoardConfig::ACTIVE.display;
  const BoardConfig::DisplayController controller = BoardConfig::ACTIVE.displayController;
  // An UltraChip panel that a DisplayProbeConfig board's probe picked takes its own
  // deep-sleep command; BUSY_N is LOW while busy.
  const bool ultraChip = FREEINK_CAP_DISPLAY_PROBE_POLICY && (controller == BoardConfig::DisplayController::UC8279 ||
                                                              controller == BoardConfig::DisplayController::UC8179);
  if ((controller != BoardConfig::DisplayController::SSD1677 && !ultraChip) || d.powerEnable >= 0 || d.rst < 0 ||
      d.cs < 0 || d.sclk < 0 || d.mosi < 0 || d.dc < 0) {
    return false;
  }
  constexpr uint8_t SSD1677_DEEP_SLEEP = 0x10;
  constexpr uint8_t SSD1677_DEEP_SLEEP_MODE_2 = 0x03;
  constexpr uint8_t UC81XX_DEEP_SLEEP = 0x07;        // DSLP
  constexpr uint8_t UC81XX_DEEP_SLEEP_CHECK = 0xA5;  // its check code
  constexpr uint32_t RESET_LOW_MS = 10;
  constexpr uint32_t RESET_SETTLE_MS = 10;
  constexpr uint32_t BUSY_WAIT_MAX_MS = 100;
  driveLevel(d.cs, HIGH);
  driveLevel(d.sclk, LOW);
  driveLevel(d.mosi, LOW);
  driveLevel(d.dc, LOW);
  // A hardware reset returns the controller to its power-on state, analog and
  // clock off, whatever a crash left running; the image stays on the glass.
  driveLevel(d.rst, HIGH);
  digitalWrite(d.rst, LOW);
  delay(RESET_LOW_MS + 1);
  digitalWrite(d.rst, HIGH);
  delay(RESET_SETTLE_MS + 1);
  if (d.busy >= 0) {
    const uint32_t start = millis();
    if (ultraChip) {
      pinMode(d.busy, BoardConfig::displayBusyPullUp() ? INPUT_PULLUP : INPUT);
      while (digitalRead(d.busy) == LOW && millis() - start < BUSY_WAIT_MAX_MS) delay(1);
    } else {
      pinMode(d.busy, d.busyPullDown ? INPUT_PULLDOWN : INPUT);
      while (digitalRead(d.busy) == HIGH && millis() - start < BUSY_WAIT_MAX_MS) delay(1);
    }
  }
  const auto cs = static_cast<gpio_num_t>(d.cs);
  const auto dc = static_cast<gpio_num_t>(d.dc);
  const auto sclk = static_cast<gpio_num_t>(d.sclk);
  const auto mosi = static_cast<gpio_num_t>(d.mosi);
  gpio_set_level(cs, 0);
  gpio_set_level(dc, 0);
  bitBangByte(sclk, mosi, ultraChip ? UC81XX_DEEP_SLEEP : SSD1677_DEEP_SLEEP);
  gpio_set_level(dc, 1);
  bitBangByte(sclk, mosi, ultraChip ? UC81XX_DEEP_SLEEP_CHECK : SSD1677_DEEP_SLEEP_MODE_2);
  gpio_set_level(cs, 1);
  gpio_set_level(mosi, 0);
  gpio_set_level(dc, 0);
  return true;
}

void PowerManager::deepSleep() {
  esp_sleep_config_gpio_isolate();
#if !FREEINK_MCU_C61
  gpio_deep_sleep_hold_en();
#endif
  esp_deep_sleep_start();
  // esp_deep_sleep_start() does not return in normal operation -- reaching
  // this line means sleep entry was rejected/aborted by the SoC rather than
  // the next boot being a fresh wake. A busy-loop here draws full-clock
  // current (~20-30mA) indefinitely and never responds to the power button,
  // making an aborted sleep entry indistinguishable from a real drain bug.
  // Record the abort (RTC_NOINIT_ATTR survives the reset below) so the next
  // boot can report it via takeAbortedSleepInfo(), then reset for real via
  // esp_restart() -- itself [[noreturn]], satisfying this function's contract.
  const int8_t pin = powerPin();
  abortedSleepRecord.aborted = true;
  abortedSleepRecord.wakeupCause = static_cast<int>(esp_sleep_get_wakeup_cause());
  abortedSleepRecord.wakePinLevel = pin >= 0 ? digitalRead(pin) : -1;
  abortedSleepMagic = ABORTED_SLEEP_MAGIC;
  esp_restart();
}

void PowerManager::deepSleepUntilPowerButton() {
  const int8_t pin = powerPin();
  if (!waitForPowerButtonRelease() && pin >= 0) {
    // Still pressed after PowerConfig::releaseWaitMaxMs (a stuck key, a press in
    // a bag): sleep until the release rather than spin here, and let that wake go
    // straight back to sleep (takeReleaseWake()).
    preparePowerPinForSleep(pin);
    armWakeOnPins(1ULL << pin, /*wakeLow=*/powerActiveHigh());
    releaseWakeMagic = RELEASE_WAKE_MAGIC;
    deepSleep();
  }
  releaseWakeMagic = 0;
  armPowerButtonWakeup();
  deepSleep();
}

bool PowerManager::takeReleaseWake() {
  const bool releaseWake = releaseWakeMagic == RELEASE_WAKE_MAGIC && esp_reset_reason() == ESP_RST_DEEPSLEEP;
  releaseWakeMagic = 0;
  return releaseWake;
}

uint8_t PowerManager::countAbnormalResets(const bool abortedSleep) {
  bool abnormal = abortedSleep;
  switch (esp_reset_reason()) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_CPU_LOCKUP:
    case ESP_RST_BROWNOUT:
      abnormal = true;
      break;
    default:
      break;
  }
  // RTC_NOINIT memory is garbage after a power-on: the magic word rejects it.
  if (!abnormal || abnormalResetsMagic != ABNORMAL_RESETS_MAGIC) abnormalResets = 0;
  if (abnormal && abnormalResets < UINT8_MAX) ++abnormalResets;
  abnormalResetsMagic = ABNORMAL_RESETS_MAGIC;
#if FREEINK_DIAGNOSTICS
  bootRecord.counted = true;
  bootRecord.abnormalResets = static_cast<uint8_t>(abnormalResets);
#endif
  return static_cast<uint8_t>(abnormalResets);
}

void PowerManager::clearAbnormalResets() {
  abnormalResets = 0;
  abnormalResetsMagic = ABNORMAL_RESETS_MAGIC;
}

PowerManager::AbortedSleepInfo PowerManager::takeAbortedSleepInfo() {
  AbortedSleepInfo info;
  if (abortedSleepMagic == ABORTED_SLEEP_MAGIC && abortedSleepRecord.aborted) {
    info.aborted = true;
    info.wakeupCause = abortedSleepRecord.wakeupCause;
    info.wakePinLevel = abortedSleepRecord.wakePinLevel;
  }
  // Clear so a stale record doesn't get reported again on a later boot, and
  // re-stamp the magic so an uninitialized cold-boot read still reports false
  // rather than following a garbage `aborted` bit.
  abortedSleepRecord.aborted = false;
  abortedSleepMagic = ABORTED_SLEEP_MAGIC;
#if FREEINK_DIAGNOSTICS
  bootRecord.abortedSleepTaken = true;
  if (info.aborted) bootRecord.abortedSleep = info;
#endif
  return info;
}

#if FREEINK_DIAGNOSTICS
uint8_t PowerManager::peekAbnormalResets() {
  return abnormalResetsMagic == ABNORMAL_RESETS_MAGIC ? static_cast<uint8_t>(abnormalResets) : 0;
}

PowerManager::AbortedSleepInfo PowerManager::peekAbortedSleepInfo() {
  AbortedSleepInfo info;
  if (abortedSleepMagic == ABORTED_SLEEP_MAGIC && abortedSleepRecord.aborted) {
    info.aborted = true;
    info.wakeupCause = abortedSleepRecord.wakeupCause;
    info.wakePinLevel = abortedSleepRecord.wakePinLevel;
  }
  return info;
}

PowerManager::BootResetRecord PowerManager::bootResetRecord() { return bootRecord; }
#endif

}  // namespace freeink
