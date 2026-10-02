#include "FrontlightManager.h"

#if FREEINK_CAP_FRONTLIGHT
#include <M5Pm1.h>
#include <SharedAdc.h>
#include <Wire.h>
#include <driver/gpio.h>
#ifdef FREEINK_FRONTLIGHT_LS
#include <driver/ledc.h>
// esp_sleep_sub_mode_config lives in a private IDF header (no public API exists
// for balancing the refcounted RC_FAST keep-on the LEDC driver takes for
// KEEP_ALIVE channels — the driver manages it through this same header). Pinned
// IDF 5.5; re-check on IDF bumps.
#include <esp_private/esp_sleep_internal.h>
#endif
#if FREEINK_CAP_FRONTLIGHT_PROBE
#include <esp_attr.h>
#include <esp_system.h>
#endif

// Logging: use the firmware's Logging.h LOG_INF facility, NOT esp_log. The
// prebuilt Arduino sdkconfig sets CONFIG_LOG_DEFAULT_LEVEL_ERROR, so ESP_LOGI is
// compiled out, and esp_log writes to the IDF UART console rather than the
// firmware's Serial stream the monitor reads. LOG_INF routes through logPrintf
// to the firmware Serial and the crash-report ring buffer.
#include <Logging.h>

namespace {
constexpr uint32_t maxDuty(uint8_t bits) { return (1u << bits) - 1u; }

// Perception-weighted percent -> duty, gamma 1.6554: 16-bit fixed-point table
// of round(65535 * (pct/100)^1.6554). The exponent is log(2*1023)/log(100),
// picked so on a 10-bit duty range 1% rounds up to exactly 1 LSB (the dimmest
// possible light) and every 1% step maps to a distinct duty — gamma 2 was too
// steep and collapsed 1-4% into the same single-LSB duty.
static constexpr uint16_t GAMMA_TABLE[101] = {
    0,     32,    101,   197,   318,   460,   622,   803,   1001,  1217,  1449,  1697,  1960,  2237,  2529,
    2835,  3155,  3488,  3834,  4193,  4565,  4949,  5345,  5753,  6173,  6604,  7047,  7502,  7967,  8444,
    8931,  9429,  9938,  10457, 10987, 11527, 12077, 12638, 13208, 13789, 14379, 14979, 15588, 16208, 16836,
    17474, 18122, 18779, 19445, 20120, 20804, 21497, 22200, 22911, 23631, 24360, 25097, 25843, 26598, 27362,
    28134, 28914, 29703, 30500, 31306, 32120, 32942, 33772, 34611, 35457, 36312, 37175, 38045, 38924, 39811,
    40705, 41608, 42518, 43436, 44361, 45295, 46236, 47185, 48141, 49105, 50076, 51055, 52042, 53036, 54037,
    55046, 56062, 57086, 58117, 59155, 60200, 61253, 62313, 63380, 64454, 65535};

constexpr uint32_t perceptualDuty(uint32_t pct, uint32_t full) {
  if (pct == 0) return 0;
  if (pct > 100) pct = 100;
  const uint32_t duty = (full * GAMMA_TABLE[pct] + 32767u) / 65535u;
  return duty ? duty : 1u;
}

// Paper Mono: the PWM lives in the M5PM1 PMIC, not the ESP. PM1 GPIO3 routed to
// alt-function PWM0 drives the AW9967 frontlight driver. Duty register is
// 12-bit; the high byte's bit 4 is the channel-enable bit. Perception-weighted
// like M5Unified's bring-up: duty = brightness^2 scaled into 12 bits.
constexpr uint8_t PM1_PWM_ENABLE = 0x10;

void pm1FrontlightAttach(uint32_t freqHz) {
  freeink::m5pm1::beginBus();
  // GPIO3 to push-pull, alt-function PWM0.
  freeink::m5pm1::updateReg(freeink::m5pm1::REG_GPIO_DRV, 1u << 3, 0);
  freeink::m5pm1::updateReg(freeink::m5pm1::REG_GPIO_FUNC0, 0xC0, 0xC0);
  freeink::m5pm1::writeReg16(freeink::m5pm1::REG_PWM_FREQ_L, static_cast<uint16_t>(freqHz));
}

void pm1FrontlightWrite(uint32_t pct) {
  const uint32_t duty = (pct * pct * 4095u) / 10000u;  // 0-100% -> 12-bit, gamma ~2
  const uint8_t data[2] = {static_cast<uint8_t>(duty & 0xFF),
                           static_cast<uint8_t>(((duty >> 8) & 0x0F) | (duty ? PM1_PWM_ENABLE : 0))};
  freeink::m5pm1::writeBytes(freeink::m5pm1::REG_PWM0_DUTY_L, data, sizeof(data));
}

// Fixed LEDC channels for the Arduino-ESP32 2.x path (3.x keys by GPIO and allocates
// channels itself). Frontlight owns 0 (cool/primary) and 1 (warm); no other SDK LEDC
// user on a frontlight board takes these (the Buzzer uses the 3.x gpio-keyed API).
constexpr uint8_t LEDC_CH_COOL = 0;
constexpr uint8_t LEDC_CH_WARM = 1;
// The colour select's blend carrier (FrontlightConfig::colorBlendHz). 2.x clocks
// channel pairs from one timer, so it sits on the next pair to keep its own
// frequency; 3.x gives a new frequency a free timer.
constexpr uint8_t LEDC_CH_BLEND = 2;

// Apply the board's output polarity to a logical 0..full LED duty.
uint32_t physicalDuty(uint32_t logicalDuty, uint32_t full, bool activeHigh) {
  return activeHigh ? logicalDuty : full - logicalDuty;
}

#ifdef FREEINK_FRONTLIGHT_LS
// Light-sleep-surviving LEDC: clock the timer from RC_FAST (~17.5 MHz on the
// S3 — the practical LEDC source that keeps running through light sleep at
// near-zero extra sleep power; XTAL can also be kept up but costs far more in
// sleep current), mark the
// channels KEEP_ALIVE, and disable the GPIO sleep-isolation override on the
// output pins (a documented gotcha: sleep entry reconfigures the pad and kills
// the PWM even when the clock survives). RC_FAST at 10 kHz supports up to
// 10-bit resolution (17.5 MHz / 10 kHz = 1750 >= 1024), so the board profiles'
// full duty range — including setBrightnessLevel's level-1 minimum step — stays
// expressible. Uses the IDF driver directly (fixed LEDC_TIMER_0 + the channel
// ids below) because the Arduino helpers don't expose sleep_mode; safe here
// because frontlight boards using this flag have no other LEDC consumer.
bool attachChannel(int8_t gpio, uint8_t ch, uint32_t freq, uint8_t bits) {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = freq;
  timer.clk_cfg = LEDC_USE_RC_FAST_CLK;
  if (ledc_timer_config(&timer) != ESP_OK) {
    // freq/bits exceed RC_FAST — leave the light unconfigured rather than
    // silently falling back to a clock that freezes in light sleep.
    return false;
  }
  ledc_channel_config_t chan = {};
  chan.gpio_num = gpio;
  chan.speed_mode = LEDC_LOW_SPEED_MODE;
  chan.channel = static_cast<ledc_channel_t>(ch);
  chan.intr_type = LEDC_INTR_DISABLE;
  chan.timer_sel = LEDC_TIMER_0;
  chan.duty = 0;
  chan.hpoint = 0;
  chan.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
  // ledc_channel_config() disables the pad's sleep-isolation override itself
  // for KEEP_ALIVE channels (IDF 5.5), so no explicit gpio_sleep_sel_dis here.
  return ledc_channel_config(&chan) == ESP_OK;
}
void writeChannel(int8_t /*gpio*/, uint8_t ch, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(ch), duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(ch));
}
#elif defined(ARDUINO) && ESP_ARDUINO_VERSION_MAJOR >= 3
bool attachChannel(int8_t gpio, uint8_t /*ch*/, uint32_t freq, uint8_t bits) { return ledcAttach(gpio, freq, bits); }
void writeChannel(int8_t gpio, uint8_t /*ch*/, uint32_t duty) { ledcWrite(gpio, duty); }
#else
bool attachChannel(int8_t gpio, uint8_t ch, uint32_t freq, uint8_t bits) {
  ledcSetup(ch, freq, bits);
  ledcAttachPin(gpio, ch);
  return true;
}
void writeChannel(int8_t /*gpio*/, uint8_t ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// --- Enable-gated path (FrontlightConfig::colorSelectGpio / enablePulseUs / monitorAdc) ---
// The PWM pin moves between the plain GPIO output register and its LEDC channel
// through the GPIO matrix, so the channel and timer stay configured while dark.
// No step touches a pull: gpio_set_direction() and the matrix routing leave the
// pull bits alone, and nothing here calls gpio_reset_pin() (which would enable a
// pull-up and, on an ADIM input, switch the driver on).

constexpr uint64_t MONITOR_PERIOD_US = 100000;  // LED-rail monitor rate while lit
// Dark this long or longer => the driver may have shut down (TPS923610: 2.5 ms), so
// the next enable repeats the enable pulse.
constexpr int64_t REENABLE_AFTER_DARK_US = 2000;
constexpr TickType_t MONITOR_ADC_WAIT = pdMS_TO_TICKS(10);
#if FREEINK_CAP_FRONTLIGHT_PROBE
// After a trip: the rail is read this often while the colour select waits for it
// (on the monitor timer, or in waitRailSafe() at sleep entry), for at most
// RAIL_WAIT_MAX_US. That bound covers a boost output that discharges through its
// monitor divider alone (Silkscreen: 4.7 uF x 1.12 MOhm, 25.5 V down to 15 V in
// 2.8 s at the nameplate capacitance).
constexpr uint64_t RAIL_POLL_US = 20000;
constexpr TickType_t RAIL_POLL = pdMS_TO_TICKS(RAIL_POLL_US / 1000);
constexpr int64_t RAIL_WAIT_MAX_US = 4000000;
// The monitor lags the rail through its RC (Silkscreen: 10.7 ms), so a reading this
// soon after the rail was marked charged can read low: it never counts as safe.
constexpr int64_t RAIL_READ_SETTLE_US = 35000;

// The load record across deep sleep and software restarts (FrontlightLoad.h). A
// power-on leaves RTC_NOINIT memory as garbage, which unpack() rejects.
RTC_NOINIT_ATTR freeink::frontlight::PackedRecord rtcLoadRecord;
#endif

bool gatedPath(const BoardConfig::FrontlightConfig& fl) {
  return fl.colorSelectGpio != BoardConfig::PIN_UNASSIGNED || fl.enablePulseUs != 0 ||
         fl.monitorAdc != BoardConfig::PIN_UNASSIGNED;
}

bool monitorConfigured(const BoardConfig::FrontlightConfig& fl) {
  return fl.monitorAdc != BoardConfig::PIN_UNASSIGNED && fl.monitorScaleX1000 != 0 && fl.monitorTripMv != 0;
}

uint32_t darkLevel(const BoardConfig::FrontlightConfig& fl) { return fl.activeHigh ? 0u : 1u; }

// Drive the pad from the GPIO output register at `level`; its LEDC channel keeps
// running, disconnected.
void padToGpio(int8_t gpio, uint32_t level) {
  const auto g = static_cast<gpio_num_t>(gpio);
  gpio_set_level(g, level);
  gpio_set_direction(g, GPIO_MODE_OUTPUT);
}

// Reconnect the pad to its LEDC channel and clear its GPIO output bit, so a later
// LEDC detach or pinMode() leaves the pad LOW rather than on a stale HIGH.
void padToLedc(int8_t gpio, uint8_t ch) {
#ifdef FREEINK_FRONTLIGHT_LS
  ledc_set_pin(gpio, LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(ch));
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
#elif defined(ARDUINO) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  // Re-routes the pin's LEDC signal and sets GPIO_OUT to 0. Should the pin have
  // lost its channel, drop the pad LOW instead of leaving it active.
  if (!ledcOutputInvert(gpio, false)) gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
#else
  ledcAttachPin(gpio, ch);
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
#endif
}

// Release the LEDC channel. Arduino-ESP32 3.x ledcDetach() only routes the pad
// back to its GPIO output register and frees the channel/timer: no pin reset and
// no pull change.
void detachChannel(int8_t gpio, uint8_t ch) {
#ifdef FREEINK_FRONTLIGHT_LS
  (void)gpio;
  ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(ch), 0);
#elif defined(ARDUINO) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcDetach(gpio);
#else
  (void)ch;
  ledcDetachPin(gpio);
#endif
}

// Holds the gated path's lock for a scope; a no-op without a monitor (then nothing
// runs concurrently with apply()).
class GateGuard {
 public:
  explicit GateGuard(SemaphoreHandle_t lock) : lock_(lock) {
    if (lock_) xSemaphoreTake(lock_, portMAX_DELAY);
  }
  ~GateGuard() {
    if (lock_) xSemaphoreGive(lock_);
  }
  GateGuard(const GateGuard&) = delete;
  GateGuard& operator=(const GateGuard&) = delete;

 private:
  SemaphoreHandle_t lock_;
};
}  // namespace
#endif

void FrontlightManager::begin() {
#if FREEINK_CAP_FRONTLIGHT
  const auto& fl = BoardConfig::ACTIVE.frontlight;
#if FREEINK_DEVICE_EEGO_A4
  const auto& i2c = BoardConfig::ACTIVE.i2cFrontlight;
  if (BoardConfig::ACTIVE.board == BoardConfig::Board::EegoA4 &&
      i2c.controller == BoardConfig::I2cFrontlightController::Lm3630a) {
    if (i2c.sda < 0 || i2c.scl < 0 || i2c.enable < 0 || i2c.address == 0) return;
    Wire.begin(i2c.sda, i2c.scl, i2c.i2cHz);
    Wire.setTimeOut(256);
    pinMode(i2c.enable, OUTPUT);
    digitalWrite(i2c.enable, LOW);

    // The OEM firmware contains an LM3630A path, but at least one retail EEGO
    // A4 revision has no frontlight hardware populated. Probe with the recovered
    // enable sequence so an absent option stays off and is not exposed as a
    // capability merely because its driver exists in a shared firmware image.
    digitalWrite(i2c.enable, HIGH);
    delay(2);
    Wire.beginTransmission(i2c.address);
    const bool detected = Wire.endTransmission() == 0;
    digitalWrite(i2c.enable, LOW);
    if (!detected) return;

    _begun = true;
    _brightness = 0;
    return;
  }
#endif
  if (fl.viaPm1Pwm) {
    pm1FrontlightAttach(fl.pwmFrequency);
    _begun = true;
    setBrightness(0);
    return;
  }
  if (fl.gpio == BoardConfig::PIN_UNASSIGNED) return;

  if (gatedPath(fl)) {
    // Start from defined levels: the PWM pin inactive and colour select at cool,
    // both push-pull with no pulls and any sleep hold released, and the LED
    // monitor pin a plain input (a pull would skew its divider).
    BoardConfig::driveParkedLevel(fl.gpio, darkLevel(fl));
    if (fl.colorSelectGpio != BoardConfig::PIN_UNASSIGNED) {
      BoardConfig::driveParkedLevel(fl.colorSelectGpio, fl.colorSelectWarmHigh ? LOW : HIGH);
    }
    _warmSelected = false;
    _gatedLit = false;
    _fault = false;
    if (monitorConfigured(fl)) {
      pinMode(fl.monitorAdc, INPUT);
      // Both live for the life of the (singleton) manager. The monitor is the only
      // code that runs concurrently with apply(), so boards without one allocate
      // nothing here.
      if (_gateLock == nullptr) _gateLock = xSemaphoreCreateMutex();
      if (_monitorTimer == nullptr && _gateLock != nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = &FrontlightManager::monitorTick;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "fl_monitor";
        args.skip_unhandled_events = true;
        if (esp_timer_create(&args, &_monitorTimer) != ESP_OK) _monitorTimer = nullptr;
      }
      if (_monitorTimer == nullptr) {
        // Never run the light unwatched on a board that declares a monitor.
        LOG_ERR("FrontlightMgr", "begin: LED monitor unavailable, light disabled");
        return;
      }
#if FREEINK_CAP_FRONTLIGHT_PROBE
      _railCharged = false;
      _litSinceReset[0] = _litSinceReset[1] = false;
      endProbe();
      restoreRecord();
#endif
    }
  }

  bool attachOk = attachChannel(fl.gpio, LEDC_CH_COOL, fl.pwmFrequency, fl.pwmResolutionBits);
  if (fl.gpioWarm != BoardConfig::PIN_UNASSIGNED) {
    attachOk = attachChannel(fl.gpioWarm, LEDC_CH_WARM, fl.pwmFrequency, fl.pwmResolutionBits) || attachOk;
  }
#ifdef FREEINK_FRONTLIGHT_LS
  // Defensive: a prior sleep cycle may have left the pads held (park() latches a
  // digital hold that survives deep sleep AND the wake reset while the _lsParked
  // DRAM flag is lost). Release any surviving hold here so begin() always starts
  // from a clean pad — a held pad silently ignores the LEDC drive below. The
  // release is unconditional (gpio_hold_dis on a non-held pad is a harmless no-op)
  // because we cannot trust _lsParked after a reset.
  for (const int8_t pin : {fl.gpio, fl.gpioWarm}) {
    if (pin >= 0) gpio_hold_dis(static_cast<gpio_num_t>(pin));
  }
  _lsParked = false;
  LOG_INF("FrontlightMgr", "begin: cleared any stale held pads");
  // The FIRST successful KEEP_ALIVE channel config takes a single refcounted +1
  // on the RC_FAST sleep sub-mode (esp_sleep_sub_mode_config; the driver's
  // global-clock latch means later configs don't take another), which would
  // keep RC_FAST — and the digital domain at its higher sleep bias — powered
  // through every light-sleep window from boot, even with the light off.
  // Balance it here and let apply() re-arm only while the light is actually
  // lit. attachOk is true when ANY channel config succeeded (exactly the
  // condition under which the driver's +1 was taken); the !_begun guard keeps a
  // hypothetical second begin() from decrementing twice.
  _lsAttachOk = attachOk;
  _lsKeepAliveArmed = false;
  if (attachOk && !_begun) {
    esp_sleep_sub_mode_config(ESP_SLEEP_DIG_USE_RC_FAST_MODE, false);
  }
#else
  (void)attachOk;
#endif
  if (!attachOk && gatedPath(fl)) {
    // Without a carrier the enable pulse would leave the pin active: stay dark.
    BoardConfig::driveParkedLevel(fl.gpio, darkLevel(fl));
    LOG_ERR("FrontlightMgr", "begin: LEDC attach failed on gpio=%d, light disabled", fl.gpio);
    return;
  }
#if FREEINK_CAP_FRONTLIGHT_PROBE && !defined(FREEINK_FRONTLIGHT_LS)
  // Colour blending: the colour select gets its own carrier, and stays on its GPIO
  // register at cool until a blend starts. Without one the selection stays fixed.
  _blending = false;
  _colourCarrier = false;
  if (fl.colorBlendHz != 0 && fl.colorSelectGpio != BoardConfig::PIN_UNASSIGNED && monitorConfigured(fl)) {
    _colourCarrier = attachChannel(fl.colorSelectGpio, LEDC_CH_BLEND, fl.colorBlendHz, fl.pwmResolutionBits);
    if (_colourCarrier) writeChannel(fl.colorSelectGpio, LEDC_CH_BLEND, 0);
    padToGpio(fl.colorSelectGpio, fl.colorSelectWarmHigh ? 0u : 1u);
    if (!_colourCarrier) {
      LOG_ERR("FrontlightMgr", "begin: no blend carrier on gpio=%d, warmth picks one string",
              fl.colorSelectGpio);
    }
  }
#endif
  _begun = true;
  setBrightness(0);
  LOG_INF("FrontlightMgr", "begin: attached gpio=%d warm=%d ok=%d", fl.gpio, fl.gpioWarm, attachOk ? 1 : 0);
#endif
}

#if FREEINK_CAP_FRONTLIGHT
#if FREEINK_DEVICE_EEGO_A4
bool FrontlightManager::lm3630aWrite(const uint8_t reg, const uint8_t value) {
  const auto& cfg = BoardConfig::ACTIVE.i2cFrontlight;
  Wire.beginTransmission(cfg.address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool FrontlightManager::lm3630aRead(const uint8_t reg, uint8_t& value) {
  const auto& cfg = BoardConfig::ACTIVE.i2cFrontlight;
  Wire.beginTransmission(cfg.address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(cfg.address, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) return false;
  value = Wire.read();
  return true;
}

bool FrontlightManager::lm3630aUpdate(const uint8_t reg, const uint8_t mask, const uint8_t value) {
  uint8_t current = 0;
  if (!lm3630aRead(reg, current)) return false;
  return lm3630aWrite(reg, static_cast<uint8_t>((current & ~mask) | (value & mask)));
}

bool FrontlightManager::configureLm3630a() {
  const auto& cfg = BoardConfig::ACTIVE.i2cFrontlight;
  digitalWrite(cfg.enable, HIGH);
  delay(2);
  Wire.beginTransmission(cfg.address);
  if (Wire.endTransmission() != 0) {
    digitalWrite(cfg.enable, LOW);
    return false;
  }

  // Exact EEGO A4 sequence recovered from HalBacklight/LM3630A in CrossLink.
  // Register meanings follow TI SNVS974B: filter, config, boost, max-current A/B,
  // control, then the two brightness banks.
  bool ok = lm3630aWrite(0x50, 0x03);
  ok = lm3630aUpdate(0x01, 0x07, 0x00) && ok;
  ok = lm3630aWrite(0x02, 0x38) && ok;
  ok = lm3630aUpdate(0x05, 0x1f, 0x10) && ok;
  ok = lm3630aUpdate(0x06, 0x1f, 0x10) && ok;
  ok = lm3630aUpdate(0x00, 0x14, 0x00) && ok;
  ok = lm3630aUpdate(0x00, 0x0b, 0x00) && ok;
  delay(2);
  ok = lm3630aWrite(0x03, 0x00) && ok;
  ok = lm3630aWrite(0x04, 0x00) && ok;
  _i2cConfigured = ok;
  if (!ok) digitalWrite(cfg.enable, LOW);
  return ok;
}

void FrontlightManager::applyLm3630a() {
  const auto& cfg = BoardConfig::ACTIVE.i2cFrontlight;
  if (_brightness == 0) {
    if (_i2cConfigured) {
      lm3630aWrite(0x03, 0);
      lm3630aWrite(0x04, 0);
    }
    digitalWrite(cfg.enable, LOW);
    _i2cConfigured = false;
    return;
  }
  if (!_i2cConfigured && !configureLm3630a()) return;

  const uint8_t level = static_cast<uint16_t>(_brightness) * 255 / 100;
  uint8_t warm = static_cast<uint16_t>(level) * _warmPercent / 100;
  uint8_t cool = static_cast<uint16_t>(level) * (100 - _warmPercent) / 100;
  // The IC ignores brightness codes 1..3; preserve a visible nonzero request.
  if (warm != 0 && warm < 4) warm = 4;
  if (cool != 0 && cool < 4) cool = 4;

  lm3630aUpdate(0x00, 0x80, 0x00);  // leave software sleep
  delay(2);
  lm3630aWrite(0x03, warm);
  lm3630aWrite(0x04, cool);
  lm3630aUpdate(0x00, 0x04, warm >= 4 ? 0x04 : 0x00);
  lm3630aUpdate(0x00, 0x02, cool >= 4 ? 0x02 : 0x00);
}
#endif

void FrontlightManager::apply() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_begun) return;
#if FREEINK_DEVICE_EEGO_A4
  if (BoardConfig::ACTIVE.board == BoardConfig::Board::EegoA4 &&
      BoardConfig::ACTIVE.i2cFrontlight.controller == BoardConfig::I2cFrontlightController::Lm3630a) {
    applyLm3630a();
    return;
  }
#endif
  if (fl.viaPm1Pwm) {
    pm1FrontlightWrite(_brightness);
    return;
  }
  if (fl.gpio == BoardConfig::PIN_UNASSIGNED) return;

  const uint32_t full = maxDuty(fl.pwmResolutionBits);
  const bool dual = fl.gpioWarm != BoardConfig::PIN_UNASSIGNED;

  // Convert brightness to PWM precision BEFORE splitting it between channels.
  // Splitting integer percentages first loses both fractional parts at low
  // levels: brightness=1, warmth=50 previously became cool=0% + warm=0%.
  // Splitting the total duty also keeps cool+warm equal to the requested total.
  uint32_t totalDuty = 0;
  if (_useLevel && _brightnessLevel > 0) {
    const uint32_t n = static_cast<uint32_t>(_brightnessLevel - 1u);
    totalDuty = 1u + (n * n * (full - 1u)) / (254u * 254u);
  } else if (!_useLevel) {
    totalDuty = perceptualDuty(_brightness, full);
  }
  if (gatedPath(fl)) {
    applyGated(totalDuty, full);
    return;
  }
  uint32_t warmDuty = 0;
  uint32_t coolDuty = totalDuty;
  if (dual) {
    warmDuty = (totalDuty * _warmPercent + 50u) / 100u;
    coolDuty = totalDuty - warmDuty;
  }
#ifdef FREEINK_FRONTLIGHT_LS
  updateLsKeepAlive(totalDuty != 0);
#endif
  writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(coolDuty, full, fl.activeHigh));

  if (dual) {
    writeChannel(fl.gpioWarm, LEDC_CH_WARM, physicalDuty(warmDuty, full, fl.activeHigh));
  }
  LOG_INF("FrontlightMgr", "apply: brightness=%u level=%u totalDuty=%u coolDuty=%u warmDuty=%u lit=%d", _brightness,
           _brightnessLevel, totalDuty, coolDuty, warmDuty, totalDuty != 0 ? 1 : 0);
}

void FrontlightManager::applyGated(const uint32_t duty, const uint32_t full) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  const GateGuard guard(_gateLock);
  if (duty == 0) {
    _fault = false;  // switching the light off re-arms the monitor latch
#if FREEINK_CAP_FRONTLIGHT_PROBE
    abandonProbe();  // and abandons a switch-on still deciding its load
#endif
    goDark(full);
    return;
  }
  if (_fault) return;  // tripped: stays dark until switched off or clearFault()
  const freeink::frontlight::ColourPlan plan = colourPlan();
#if FREEINK_CAP_FRONTLIGHT_PROBE
  if (_probe.phase != ProbePhase::Idle) {
    // A switch-on still deciding its load: the brightness applies now, and so does
    // a new mix while it blends; any other colour change waits until it has decided
    // (probePassed()).
    _probe.duty = duty;
    if (_gatedLit) writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(duty, full, fl.activeHigh));
    if (_blending && plan.blend) blendColour(full);
    return;
  }
#endif
  // Lit: a blend moves its mix at once, and one that ends holds the colour select on
  // a string while lit (one more of its own edges).
  if (_gatedLit && (plan.blend || _blending || plan.warm == _warmSelected)) {
    writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(duty, full, fl.activeHigh));  // brightness
    if (plan.blend) {
      blendColour(full);
    } else if (_blending) {
      selectWarm(plan.warm);
    }
    return;
  }
  if (confirmsWhileLit(plan)) {
    switchLitToConfirm(duty, full, plan.warm);
    return;
  }
  if (_gatedLit) goDark(full);  // colour change: switch strings only with the driver dark
  lightUp(duty, full, plan.warm, false, plan.blend);
}

bool FrontlightManager::confirmsWhileLit(const freeink::frontlight::ColourPlan& plan) const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_gatedLit || _blending || plan.blend || plan.warm == _warmSelected || fl.blendConfirmWindowMs == 0 ||
      !monitorConfigured(fl)) {
    return false;
  }
  // Only on the way to a blend: the lit string has lit since the reset and the
  // other, which the plan asks for, has not.
  const bool colourSelect = fl.colorSelectGpio != BoardConfig::PIN_UNASSIGNED;
  return freeink::frontlight::blendPossible(_loadRecord, colourSelect, _colourCarrier) && _warmPercent > 0 &&
         _warmPercent < 100 && litSinceReset(_warmSelected) && !litSinceReset(plan.warm);
#else
  (void)plan;
  return false;
#endif
}

// Caller holds _gateLock, lit (confirmsWhileLit()). Moving the colour select while
// lit is what every blend edge does; an open string on the other side drives the
// rail to the OVP, which the window reads, and a trip takes the usual second-try
// path back to the lit string once the rail allows.
void FrontlightManager::switchLitToConfirm(const uint32_t duty, const uint32_t full, const bool warm) {
#if FREEINK_CAP_FRONTLIGHT_PROBE
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  stopMonitor();
  writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(duty, full, fl.activeHigh));
  selectWarm(warm);
  LOG_DBG("FrontlightMgr", "gated: lit switch to %s for the blend", warm ? "warm" : "cool");
  beginWindow(duty, warm, false, false, fl.blendConfirmWindowMs);
#else
  (void)duty;
  (void)full;
  (void)warm;
#endif
}

freeink::frontlight::ColourPlan FrontlightManager::colourPlan() const {
  const bool colourSelect = BoardConfig::ACTIVE.frontlight.colorSelectGpio != BoardConfig::PIN_UNASSIGNED;
#if FREEINK_CAP_FRONTLIGHT_PROBE
  return freeink::frontlight::colourPlan(_loadRecord, colourSelect, _colourCarrier, _warmPercent,
                                         litSinceReset(false), litSinceReset(true));
#else
  return {false, colourSelect && _warmPercent >= 50};
#endif
}

void FrontlightManager::waitDarkDwell() const {
  const int64_t dwellUs = static_cast<int64_t>(BoardConfig::ACTIVE.frontlight.offDwellMs) * 1000;
  const int64_t darkUs = esp_timer_get_time() - _darkSinceUs;
  if (darkUs < dwellUs) delayMicroseconds(static_cast<uint32_t>(dwellUs - darkUs));
}

void FrontlightManager::selectWarm(const bool warm) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (fl.colorSelectGpio == BoardConfig::PIN_UNASSIGNED) return;
  const uint32_t level = warm == fl.colorSelectWarmHigh ? 1u : 0u;
  if (_blending) {
    padToGpio(fl.colorSelectGpio, level);  // off the blend carrier, onto the GPIO register
    writeChannel(fl.colorSelectGpio, LEDC_CH_BLEND, 0);
    _blending = false;
    LOG_DBG("FrontlightMgr", "gated: blend off, %s", warm ? "warm" : "cool");
  } else {
    gpio_set_level(static_cast<gpio_num_t>(fl.colorSelectGpio), level);
  }
  _warmSelected = warm;
}

// Caller holds _gateLock, with the light lit and both strings lit since the reset
// (colourPlan()). The carrier shares the PWM pin's resolution.
void FrontlightManager::blendColour(const uint32_t full) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  const uint32_t warmDuty = (static_cast<uint32_t>(_warmPercent) * full + 50u) / 100u;
  writeChannel(fl.colorSelectGpio, LEDC_CH_BLEND, fl.colorSelectWarmHigh ? warmDuty : full - warmDuty);
  if (!_blending) {
    padToLedc(fl.colorSelectGpio, LEDC_CH_BLEND);
    _blending = true;
    LOG_DBG("FrontlightMgr", "gated: blend on, warm %u%%", static_cast<unsigned>(_warmPercent));
  }
  _warmSelected = _warmPercent >= 50;
}

// Caller holds _gateLock.
void FrontlightManager::goDark(const uint32_t full) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  stopMonitor();
  padToGpio(fl.gpio, darkLevel(fl));
  writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(0, full, fl.activeHigh));
  if (_gatedLit) {
    _darkSinceUs = esp_timer_get_time();
    LOG_DBG("FrontlightMgr", "gated: dark");
  }
  _gatedLit = false;
  // A blend stops at the warm level. Both strings were on the rail through it, so
  // the select then rests at cool as after a warm light.
  if (_blending) selectWarm(true);
  // Colour select rests at cool, its parked level (HIGH would cost current through
  // its pull-down), but moves only once the driver has been dark for offDwellMs.
  if (_warmSelected) {
    waitDarkDwell();
#if FREEINK_CAP_FRONTLIGHT_PROBE
    // After a trip the boost output is still charged: the monitor tick moves the
    // colour select once the rail has fallen.
    if (!railSafeNow()) {
      startMonitor(MONITOR_PERIOD_US);
      return;
    }
#endif
    selectWarm(false);
  }
}

// Caller holds _gateLock; the pad is dark. With a load probe, the switch-on decides
// on the monitor timer (probing() until then).
void FrontlightManager::lightUp(const uint32_t duty, const uint32_t full, const bool warm, const bool secondTry,
                                const bool blend) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  waitDarkDwell();  // also clears a latched driver fault (TPS923610 OVP) before the enable
#if FREEINK_CAP_FRONTLIGHT_PROBE
  // The colour select never moves onto a string while a trip's charge is still on
  // the boost output: the monitor timer waits for the rail, then lights.
  if (warm != _warmSelected && !railSafeNow()) {
    beginRailWait(duty, warm, secondTry);
    return;
  }
  stopMonitor();  // a rail watch may be running
#else
  (void)secondTry;
#endif
  selectWarm(warm);
  // Program the carrier while GPIO still holds the pad; the enable pulse is longer
  // than a PWM period, so the channel has latched the duty by the handover.
  writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(duty, full, fl.activeHigh));
  const bool pulsed = fl.enablePulseUs != 0 && esp_timer_get_time() - _darkSinceUs >= REENABLE_AFTER_DARK_US;
  if (pulsed) {
    gpio_set_level(static_cast<gpio_num_t>(fl.gpio), darkLevel(fl) ^ 1u);
    delayMicroseconds(fl.enablePulseUs);
  }
  padToLedc(fl.gpio, LEDC_CH_COOL);
  _gatedLit = true;
  if (blend) blendColour(full);
  LOG_DBG("FrontlightMgr", "gated: lit duty=%u warm=%d blend=%d pulse=%d", static_cast<unsigned>(duty), warm ? 1 : 0,
          blend ? 1 : 0, pulsed ? 1 : 0);
#if FREEINK_CAP_FRONTLIGHT_PROBE
  if (fl.loadProbeWindowMs != 0 && monitorConfigured(fl)) {
    beginWindow(duty, warm, secondTry, blend, fl.loadProbeWindowMs);  // dark again if it trips
    return;
  }
  freeink::frontlight::noteLit(_loadRecord, warm);
  noteStringLit(warm);
  saveRecord();
  endProbe();
#endif
  startMonitor(MONITOR_PERIOD_US);
}

// Runs the monitor timer at `periodUs`, restarting it if it runs (also from its
// own callback).
void FrontlightManager::startMonitor(const uint64_t periodUs) {
  if (_monitorTimer == nullptr) return;
  esp_timer_stop(_monitorTimer);  // ESP_ERR_INVALID_STATE when idle: fine
  esp_timer_start_periodic(_monitorTimer, periodUs);
}

void FrontlightManager::stopMonitor() {
  if (_monitorTimer != nullptr) esp_timer_stop(_monitorTimer);  // ESP_ERR_INVALID_STATE when idle: fine
}

void FrontlightManager::monitorTick(void* self) { static_cast<FrontlightManager*>(self)->checkMonitor(); }

// esp_timer task, every MONITOR_PERIOD_US while lit, and (load probe) every
// loadProbePeriodMs in a switch-on's probe window, every RAIL_POLL_US while it
// waits for the rail, and while dark after a trip until the colour select can rest
// at cool.
void FrontlightManager::checkMonitor() {
  // apply() mid-transition: skip this tick rather than block the timer task.
  if (xSemaphoreTake(_gateLock, 0) != pdTRUE) return;
#if FREEINK_CAP_FRONTLIGHT_PROBE
  if (_probe.phase != ProbePhase::Idle) {
    if (_probe.phase == ProbePhase::Window) {
      probeTick();
    } else {
      railWaitTick();
    }
    xSemaphoreGive(_gateLock);
    return;
  }
#endif
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (_gatedLit) {
    uint32_t pinMv = 0;
    bool sampled = false;
    {
      const freeink::adc::Lock adc(MONITOR_ADC_WAIT);
      if (adc.held()) {
        pinMv = analogReadMilliVolts(fl.monitorAdc);
        sampled = true;
      }
    }
    const uint32_t ledMv = pinMv * fl.monitorScaleX1000 / 1000u;
#if FREEINK_DIAGNOSTICS
    if (sampled) {
      ++_monitorDiag.samples;
      _monitorDiag.lastPinMv = pinMv;
      _monitorDiag.lastLedMv = ledMv;
      _monitorDiag.lastSampleMs = millis();
    } else {
      ++_monitorDiag.skippedSamples;
    }
    if (sampled && ledMv > fl.monitorTripMv) {
      ++_monitorDiag.trips;
      _monitorDiag.lastTripLedMv = ledMv;
      _monitorDiag.lastTripMs = millis();
      _monitorDiag.lastTripLimitMv = fl.monitorTripMv;
    }
#endif
    if (sampled && ledMv > fl.monitorTripMv) {
      _fault = true;
#if FREEINK_CAP_FRONTLIGHT_PROBE
      freeink::frontlight::noteInUseTrip(_loadRecord, _warmSelected);
      if (_blending) freeink::frontlight::noteInUseTrip(_loadRecord, !_warmSelected);  // both were lit
      saveRecord();
      markRailCharged();
#endif
      goDark(maxDuty(fl.pwmResolutionBits));
      LOG_ERR("FrontlightMgr", "LED rail %u mV > %u mV trip: light off until switched on again",
              static_cast<unsigned>(ledMv), static_cast<unsigned>(fl.monitorTripMv));
    }
  }
#if FREEINK_CAP_FRONTLIGHT_PROBE
  else if (!_warmSelected || railSafeNow()) {
    // Dark after a trip: the rail has fallen, so the colour select can rest at cool.
    selectWarm(false);
    stopMonitor();
  } else if (esp_timer_get_time() - _railChargedAtUs >= RAIL_WAIT_MAX_US) {
    LOG_ERR("FrontlightMgr", "LED rail still above %u V %u ms after the trip: colour select left on warm",
            static_cast<unsigned>(fl.railSafeV), static_cast<unsigned>(RAIL_WAIT_MAX_US / 1000));
    stopMonitor();
  }
#endif
  xSemaphoreGive(_gateLock);
}

#if FREEINK_CAP_FRONTLIGHT_PROBE
// Caller holds _gateLock, with the light just enabled on `warm`: the monitor timer
// reads the probe window from now on (probeTick()).
void FrontlightManager::beginWindow(const uint32_t duty, const bool warm, const bool secondTry, const bool blend,
                                    const uint16_t windowMs) {
  _probe.phase = ProbePhase::Window;
  _probe.warm = warm;
  _probe.secondTry = secondTry;
  _probe.windowMs = windowMs;
  _probe.blend = blend;
  _probe.duty = duty;
  _probe.startUs = esp_timer_get_time();
  _probe.peakLedMv = 0;
  _probe.samples = 0;
  _probing.store(true, std::memory_order_release);
  const uint8_t periodMs = BoardConfig::ACTIVE.frontlight.loadProbePeriodMs;
  startMonitor(static_cast<uint64_t>(periodMs != 0 ? periodMs : 1) * 1000u);
}

// Caller holds _gateLock, dark after a trip: the monitor timer lights `warm` once
// the rail allows the colour select to move (railWaitTick()).
void FrontlightManager::beginRailWait(const uint32_t duty, const bool warm, const bool secondTry) {
  _probe.phase = ProbePhase::RailWait;
  _probe.warm = warm;
  _probe.secondTry = secondTry;
  _probe.blend = false;
  _probe.duty = duty;
  _probe.startUs = esp_timer_get_time();
  _probing.store(true, std::memory_order_release);
  startMonitor(RAIL_POLL_US);
}

// The switch-on has decided (or was abandoned). Last, so probing() reads false
// only once the record and fault() say what it found.
void FrontlightManager::endProbe() {
  _probe.phase = ProbePhase::Idle;
  _probing.store(false, std::memory_order_release);
}

// Switched off (or sleep) while a switch-on decides. A window still open on a string
// that has not lit since the reset may already have an open load's charge on the
// boost output that no reading has caught yet (the monitor lags the rail), so the
// rail counts as charged: the colour select then moves only once a settled reading
// shows it has fallen (goDark()).
void FrontlightManager::abandonProbe() {
  if (_probe.phase == ProbePhase::Window && !litSinceReset(_probe.warm)) markRailCharged();
  endProbe();
}

// Monitor timer, every loadProbePeriodMs in a probe window. Any reading above the
// trip means the string has no load.
void FrontlightManager::probeTick() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  uint32_t pinMv = 0;
  bool sampled = false;
  {
    const freeink::adc::Lock adc(MONITOR_ADC_WAIT);
    if (adc.held()) {
      pinMv = analogReadMilliVolts(fl.monitorAdc);
      sampled = true;
    }
  }
  const int64_t nowUs = esp_timer_get_time();
  if (sampled) {
    ++_probe.samples;
    const uint32_t ledMv = pinMv * fl.monitorScaleX1000 / 1000u;
    if (ledMv > _probe.peakLedMv) _probe.peakLedMv = ledMv;
    if (ledMv > fl.monitorTripMv) {
      probeTripped(nowUs);
      return;
    }
  }
  if (nowUs - _probe.startUs >= static_cast<int64_t>(_probe.windowMs) * 1000) probePassed();
}

// No load on the probed string: dark at once, and the rail counts as charged. The
// other string follows once the rail allows, unless this was it (or it is known
// to be missing): then no light answered. A blend that trips had both strings on
// the rail, so nothing else is tried.
void FrontlightManager::probeTripped(const int64_t tripAtUs) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  padToGpio(fl.gpio, darkLevel(fl));
#if FREEINK_DIAGNOSTICS
  _probeDiag.lastDarkAfterTripUs = static_cast<uint32_t>(esp_timer_get_time() - tripAtUs);
#endif
  writeChannel(fl.gpio, LEDC_CH_COOL, physicalDuty(0, maxDuty(fl.pwmResolutionBits), fl.activeHigh));
  _gatedLit = false;
  _darkSinceUs = esp_timer_get_time();
  markRailCharged();
  noteProbeEnd(true, tripAtUs);
  if (_blending) selectWarm(true);  // stops at warm; rests at cool once the rail allows
  const bool colourSelect = fl.colorSelectGpio != BoardConfig::PIN_UNASSIGNED;
  const bool warm = _probe.warm;
  if (freeink::frontlight::noteTripped(_loadRecord, colourSelect, warm, _probe.secondTry || _probe.blend)) {
#if FREEINK_DIAGNOSTICS
    ++_probeDiag.secondTries;
#endif
    beginRailWait(_probe.duty, !warm, true);
    return;
  }
  saveRecord();
  _fault = true;  // no light: dark at any brightness until switched off or clearFault()
#if FREEINK_DIAGNOSTICS
  ++_probeDiag.absent;
#endif
  LOG_ERR("FrontlightMgr", "No front light answered the switch-on: dark until switched on again");
  restColourSelectAfterTrip();
  endProbe();
}

// The window ended with no reading above the trip: the string is lit, and the 100 ms
// monitor takes over. A colour asked for meanwhile is a new switch-on on that string,
// and so is the other string a blend still needs lit; once both are, the blend starts.
void FrontlightManager::probePassed() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  _railCharged = false;  // the rail sits at the lit string's forward voltage
  noteProbeEnd(false, 0);
  const bool warm = _probe.warm;
  if (_probe.secondTry) {
    freeink::frontlight::noteOtherLit(_loadRecord, !warm);
    LOG_ERR("FrontlightMgr", "No %s string: the %s one is used until a cold boot", warm ? "cool" : "warm",
            warm ? "warm" : "cool");
  } else {
    freeink::frontlight::noteLit(_loadRecord, warm);
    if (_probe.blend) freeink::frontlight::noteLit(_loadRecord, !warm);
  }
  noteStringLit(warm);
  saveRecord();
  const freeink::frontlight::ColourPlan plan = colourPlan();
  const uint32_t full = maxDuty(fl.pwmResolutionBits);
  if (plan.blend) {
    blendColour(full);
  } else if (_blending) {
    selectWarm(plan.warm);
  } else if (plan.warm != _warmSelected) {
    // probing() stays true through either path.
    if (confirmsWhileLit(plan)) {
      switchLitToConfirm(_probe.duty, full, plan.warm);
      return;
    }
    goDark(full);
    lightUp(_probe.duty, full, plan.warm);
    return;
  }
  startMonitor(MONITOR_PERIOD_US);
  endProbe();
}

// Monitor timer, every RAIL_POLL_US while a switch-on waits for the rail before the
// colour select moves to its string.
void FrontlightManager::railWaitTick() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  const int64_t nowUs = esp_timer_get_time();
  const int64_t dwellUs = static_cast<int64_t>(fl.offDwellMs) * 1000;
  if (nowUs - _darkSinceUs >= dwellUs && railSafeNow()) {
    noteRailWait(_probe.startUs, true);
    lightUp(_probe.duty, maxDuty(fl.pwmResolutionBits), _probe.warm, _probe.secondTry);
    return;
  }
  if (nowUs - _railChargedAtUs < RAIL_WAIT_MAX_US) return;
  noteRailWait(_probe.startUs, false);
  // That string was never tried: the colour select could not move. Not a no-light
  // result, so the record stays as it was; the switch-off that clears the fault
  // (goDark) rests the colour select at cool.
  _fault = true;
  if (_probe.secondTry) {
    LOG_ERR("FrontlightMgr", "LED rail did not fall after the %s trip: the %s string was not tried; light off",
            _probe.warm ? "cool" : "warm", _probe.warm ? "warm" : "cool");
  } else {
    LOG_ERR("FrontlightMgr", "LED rail did not fall after the last trip: light off until switched on again");
  }
  stopMonitor();
  endProbe();
}

// After a no-light result: the colour select rests at cool once the rail allows;
// until then the monitor timer watches the rail (checkMonitor()).
void FrontlightManager::restColourSelectAfterTrip() {
  if (_warmSelected && !railSafeNow()) {
    startMonitor(MONITOR_PERIOD_US);
    return;
  }
  stopMonitor();
  if (!_warmSelected) return;
  waitDarkDwell();
  selectWarm(false);
}

void FrontlightManager::noteProbeEnd(const bool tripped, const int64_t tripAtUs) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
#if FREEINK_DIAGNOSTICS
  ++_probeDiag.probes;
  if (tripped) ++_probeDiag.trips;
  _probeDiag.lastWarm = _probe.warm;
  _probeDiag.lastTripped = tripped;
  _probeDiag.lastTripAfterUs = tripped ? static_cast<uint32_t>(tripAtUs - _probe.startUs) : 0;
  if (!tripped) _probeDiag.lastDarkAfterTripUs = 0;
  _probeDiag.lastPeakLedMv = _probe.peakLedMv;
  _probeDiag.lastSamples = _probe.samples;
  _probeDiag.lastMs = millis();
#endif
  if (tripped) {
    LOG_INF("FrontlightMgr", "load probe: %s string %u mV > %u mV after %u us: no load", _probe.warm ? "warm" : "cool",
            static_cast<unsigned>(_probe.peakLedMv), static_cast<unsigned>(fl.monitorTripMv),
            static_cast<unsigned>(tripAtUs - _probe.startUs));
  } else if (_probe.samples == 0) {
    LOG_ERR("FrontlightMgr", "load probe: ADC busy for %u ms, the 100 ms monitor takes over",
            static_cast<unsigned>(_probe.windowMs));
  } else {
    LOG_DBG("FrontlightMgr", "load probe: %s string lit, peak %u mV over %u readings", _probe.warm ? "warm" : "cool",
            static_cast<unsigned>(_probe.peakLedMv), static_cast<unsigned>(_probe.samples));
  }
}

void FrontlightManager::noteRailWait(const int64_t sinceUs, const bool safe) {
  const auto waitedMs = static_cast<uint32_t>((esp_timer_get_time() - sinceUs) / 1000);
#if FREEINK_DIAGNOSTICS
  ++_probeDiag.railWaits;
  _probeDiag.lastRailWaitMs = waitedMs;
  if (!safe) ++_probeDiag.railTimeouts;
#endif
  if (safe) {
    LOG_INF("FrontlightMgr", "LED rail down to %u V after %u ms: colour select free",
            static_cast<unsigned>(BoardConfig::ACTIVE.frontlight.railSafeV), static_cast<unsigned>(waitedMs));
  } else {
    LOG_ERR("FrontlightMgr", "LED rail still above %u V after %u ms",
            static_cast<unsigned>(BoardConfig::ACTIVE.frontlight.railSafeV), static_cast<unsigned>(waitedMs));
  }
}

void FrontlightManager::markRailCharged() {
  _railCharged = true;
  _railChargedAtUs = esp_timer_get_time();
}

// One reading. True when no trip has left the rail charged, or it has fallen to
// railSafeV (which clears the mark). An ADC it cannot take counts as charged.
bool FrontlightManager::railSafeNow() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_railCharged) return true;
  if (fl.railSafeV == 0 || !monitorConfigured(fl)) {
    _railCharged = false;
    return true;
  }
  if (esp_timer_get_time() - _railChargedAtUs < RAIL_READ_SETTLE_US) return false;
  uint32_t pinMv = 0;
  {
    const freeink::adc::Lock adc(MONITOR_ADC_WAIT);
    if (!adc.held()) return false;
    pinMv = analogReadMilliVolts(fl.monitorAdc);
  }
  const uint32_t ledMv = pinMv * fl.monitorScaleX1000 / 1000u;
#if FREEINK_DIAGNOSTICS
  _probeDiag.lastRailMv = ledMv;
#endif
  if (ledMv > static_cast<uint32_t>(fl.railSafeV) * 1000u) return false;
  _railCharged = false;
  return true;
}

// railSafeNow(), polling until RAIL_WAIT_MAX_US after the trip, for the sleep entry.
// Blocks the caller (never the esp_timer task) for that long at most.
bool FrontlightManager::waitRailSafe() {
  if (railSafeNow()) return true;
  const int64_t startUs = esp_timer_get_time();
  bool safe = false;
  while (!(safe = railSafeNow()) && esp_timer_get_time() - _railChargedAtUs < RAIL_WAIT_MAX_US) {
    vTaskDelay(RAIL_POLL);
  }
  noteRailWait(startUs, safe);
  return safe;
}

// begin(): keep the record the last boot left after a deep-sleep wake or a software
// restart. Any other reset (power-on, the reset button, a crash, a brown-out) is a
// cold boot and starts it empty, so Restore Light on Wake may try once again.
void FrontlightManager::restoreRecord() {
  namespace rec = freeink::frontlight;
  const esp_reset_reason_t reason = esp_reset_reason();
  const rec::ResetKind kind = reason == ESP_RST_DEEPSLEEP ? rec::ResetKind::DeepSleepWake
                              : reason == ESP_RST_SW      ? rec::ResetKind::SoftwareRestart
                                                          : rec::ResetKind::Cold;
  rec::LoadRecord record;
  const bool kept = rec::keepRecord(kind) && rec::unpack(rtcLoadRecord, record);
  _loadRecord = record;
  saveRecord();
  LOG_INF("FrontlightMgr", "load record %s: load=%u cool=%u warm=%u", kept ? "kept" : "new",
          static_cast<unsigned>(_loadRecord.load), static_cast<unsigned>(_loadRecord.cool),
          static_cast<unsigned>(_loadRecord.warm));
}

void FrontlightManager::saveRecord() const { rtcLoadRecord = freeink::frontlight::pack(_loadRecord); }
#endif

#ifdef FREEINK_FRONTLIGHT_LS
void FrontlightManager::updateLsKeepAlive(const bool lit) {
  // Refcounted, so strictly transition-edged: one +1 while lit, returned at 0.
  // Skipped when the attach failed (see begin()) — the driver never took its
  // +1 there, and RC_FAST keep-alive is moot without a working LS channel.
  if (!_lsAttachOk || lit == _lsKeepAliveArmed) return;
  esp_sleep_sub_mode_config(ESP_SLEEP_DIG_USE_RC_FAST_MODE, lit);
  _lsKeepAliveArmed = lit;
}

void FrontlightManager::park() {
  // Frontlight leakage through deep sleep (Xteink X4 Pro — Mark31415,
  // crosspoint-reader#3215). The channels are configured LEDC_SLEEP_MODE_KEEP_ALIVE
  // so the PWM keeps driving GPIO8/9 (cool/warm) through light sleep; at deep
  // sleep the panel rail is held up (PR #3215 holds power.latch0 / GPIO1 HIGH for
  // fast-wake), so the frontlight driver IC stays powered and the KEEP_ALIVE pad
  // keeps drawing quiescent + leakage current. Cut it at the source: drive both
  // pads LOW (active-high frontlight -> LED off, no booster bias) and hold them
  // LOW so the level survives deep sleep via gpio_deep_sleep_hold_en() (called by
  // PowerManager::deepSleep()). The LEDC peripheral clock (RC_FAST) is also
  // released so the driver's refcounted +1 is dropped and the clock can fully
  // stop in deep sleep. releaseOnWake() must undo this before begin() re-attaches
  // the LEDC channels on boot.
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_begun) return;
  LOG_INF("FrontlightMgr", "park: begun, driving pads LOW + hold");
  // Return the LEDC driver's refcounted RC_FAST keep-alive it took at attach, if
  // it is still armed (apply() re-arms only while lit; off()/setBrightness(0)
  // returns it, but be safe if the light was parked while lit).
  updateLsKeepAlive(false);
  // Tear down the KEEP_ALIVE LEDC channels so the pads no longer answer to the
  // peripheral; the explicit GPIO hold below then owns the pad level.
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  if (fl.gpioWarm != BoardConfig::PIN_UNASSIGNED) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  }
  for (const int8_t pin : {fl.gpio, fl.gpioWarm}) {
    if (pin < 0) continue;
    const auto g = static_cast<gpio_num_t>(pin);
    // Release any surviving pad hold first: a held pad silently ignores the drive
    // below (same trap as HalPowerManager's latch loop).
    gpio_hold_dis(g);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    gpio_hold_en(g);
    _lsParked = true;
  }
}

void FrontlightManager::releaseOnWake() {
  // Undo park() so begin() can re-attach the LEDC channels cleanly: drop the pad
  // holds (a held pad would make ledc_channel_config()'s drive a no-op) and clear
  // the parked flag. Called from the consumer at boot, before Frontlight.begin().
  //
  // CRITICAL: the release must be UNCONDITIONAL. park() latches a digital pad hold
  // (gpio_hold_en) that survives deep sleep AND the wake reset, but _lsParked is a
  // plain DRAM flag that is lost on the same reset. After a wake, _lsParked is
  // always false even though the pad is still held — gating the release on it would
  // leave the pad held forever (light dark until power-cycle). gpio_hold_dis on a
  // non-held pad is a harmless no-op, so releasing unconditionally is safe and
  // idempotent. Every other driver in this codebase releases holds unconditionally
  // before driving for exactly this reason.
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  for (const int8_t pin : {fl.gpio, fl.gpioWarm}) {
    if (pin < 0) continue;
    gpio_hold_dis(static_cast<gpio_num_t>(pin));
  }
  _lsParked = false;
  LOG_INF("FrontlightMgr", "releaseOnWake: cleared pad holds (unconditional)");
}
#endif
#endif

void FrontlightManager::setBrightness(uint8_t percent) {
#if FREEINK_CAP_FRONTLIGHT
  if (percent > 100) percent = 100;
  _brightness = percent;
  _brightnessLevel = (static_cast<uint16_t>(percent) * 255u) / 100u;
  _useLevel = false;
  if (percent > 0) _lastBrightness = percent;
  apply();
#else
  (void)percent;
#endif
}

void FrontlightManager::setBrightnessLevel(uint8_t level) {
#if FREEINK_CAP_FRONTLIGHT
  _brightnessLevel = level;
  _brightness = (static_cast<uint16_t>(level) * 100u) / 255u;
  _useLevel = true;
  apply();
#else
  (void)level;
#endif
}

void FrontlightManager::prepareForSleep() {
#if FREEINK_CAP_FRONTLIGHT
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_begun || fl.gpio == BoardConfig::PIN_UNASSIGNED || !gatedPath(fl)) return;
  const GateGuard guard(_gateLock);
#if FREEINK_CAP_FRONTLIGHT_PROBE
  abandonProbe();  // a switch-on still deciding its load is abandoned
#endif
  goDark(maxDuty(fl.pwmResolutionBits));
#if FREEINK_CAP_FRONTLIGHT_PROBE
  // A rail watch may still hold the colour select on warm: wait for the rail here
  // (bounded), then rest it at cool, as the sleep park will anyway.
  stopMonitor();
  if (_warmSelected) {
    waitRailSafe();
    selectWarm(false);
  }
#endif
  detachChannel(fl.gpio, LEDC_CH_COOL);
  padToGpio(fl.gpio, darkLevel(fl));  // the detached pad keeps the GPIO path; re-assert its level
  if (_colourCarrier) {
    // goDark() took the colour select off its blend carrier; release that too.
    detachChannel(fl.colorSelectGpio, LEDC_CH_BLEND);
    padToGpio(fl.colorSelectGpio, _warmSelected == fl.colorSelectWarmHigh ? 1u : 0u);
    _colourCarrier = false;
  }
  _begun = false;  // later apply() calls are ignored until the next begin()
  LOG_DBG("FrontlightMgr", "prepareForSleep: dark, LEDC detached");
#endif
}

#if FREEINK_DIAGNOSTICS
FrontlightManager::MonitorDiagnostics FrontlightManager::monitorDiagnostics() const {
#if FREEINK_CAP_FRONTLIGHT
  // The monitor writes the record under _gateLock; a copy taken while it is
  // held elsewhere for longer than the wait may be one reading apart.
  const bool locked = _gateLock != nullptr && xSemaphoreTake(_gateLock, MONITOR_ADC_WAIT) == pdTRUE;
  const MonitorDiagnostics copy = _monitorDiag;
  if (locked) xSemaphoreGive(_gateLock);
  return copy;
#else
  return {};
#endif
}

FrontlightManager::ProbeDiagnostics FrontlightManager::probeDiagnostics() const {
#if FREEINK_CAP_FRONTLIGHT_PROBE
  const bool locked = _gateLock != nullptr && xSemaphoreTake(_gateLock, MONITOR_ADC_WAIT) == pdTRUE;
  ProbeDiagnostics copy = _probeDiag;
  copy.record = _loadRecord;
  if (locked) xSemaphoreGive(_gateLock);
  return copy;
#else
  return {};
#endif
}
#endif

void FrontlightManager::off() { setBrightness(0); }
void FrontlightManager::on() { setBrightness(_lastBrightness); }

void FrontlightManager::setColorTemperature(uint8_t warmPercent) {
#if FREEINK_CAP_FRONTLIGHT
  _warmPercent = warmPercent > 100 ? 100 : warmPercent;
  // Only re-drives hardware when a warm channel exists; on single-channel boards this just
  // records the request (apply() ignores _warmPercent without a second channel).
  apply();
#else
  (void)warmPercent;
#endif
}

bool FrontlightManager::blendsColour() const {
#if FREEINK_CAP_FRONTLIGHT && FREEINK_CAP_FRONTLIGHT_PROBE
  const bool colourSelect = BoardConfig::ACTIVE.frontlight.colorSelectGpio != BoardConfig::PIN_UNASSIGNED;
  return freeink::frontlight::blendPossible(_loadRecord, colourSelect, _colourCarrier);
#else
  return false;
#endif
}
