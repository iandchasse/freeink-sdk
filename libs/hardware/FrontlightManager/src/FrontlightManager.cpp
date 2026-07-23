#include "FrontlightManager.h"

#if FREEINK_CAP_FRONTLIGHT

#include <driver/gpio.h>
#include <driver/ledc.h>

namespace {
uint32_t maxDuty(uint8_t bits) { return (1u << bits) - 1u; }

// LEDC assignment for the two-temperature path. Timer 0 carries the brightness
// channel at the profile's PWM frequency; timer 1 carries the warm/cool pair at
// the much lower blend frequency, so they cannot share a timer.
//
// Both are clocked from the internal RC fast oscillator rather than APB: it is
// independent of the CPU clock, so the PWM frequency does not shift when dynamic
// frequency scaling kicks in — which would show up as visible flicker. (esp-idf
// renamed this source from RTC8M; LEDC_USE_RTC8M_CLK is now a deprecated alias.)
constexpr ledc_mode_t kSpeed = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimerBright = LEDC_TIMER_0;
constexpr ledc_timer_t kTimerBlend = LEDC_TIMER_1;
constexpr ledc_channel_t kChBright = LEDC_CHANNEL_0;
constexpr ledc_channel_t kChWarm = LEDC_CHANNEL_1;
constexpr ledc_channel_t kChCool = LEDC_CHANNEL_2;

void writeDuty(ledc_channel_t ch, uint32_t duty) {
  ledc_set_duty(kSpeed, ch, duty);
  ledc_update_duty(kSpeed, ch);
}

void zeroAllChannels() {
  writeDuty(kChBright, 0);
  writeDuty(kChWarm, 0);
  writeDuty(kChCool, 0);
}
}  // namespace

#endif  // FREEINK_CAP_FRONTLIGHT

void FrontlightManager::begin() {
#if FREEINK_CAP_FRONTLIGHT
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (fl.gpio == BoardConfig::PIN_UNASSIGNED) return;

  if (hasColorTemperature()) {
    beginMultiChannel();
    _begun = true;
    return;
  }

#if defined(ARDUINO) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 3.x LEDC API.
  ledcAttach(fl.gpio, fl.pwmFrequency, fl.pwmResolutionBits);
#else
  // Arduino-ESP32 2.x fallback.
  ledcSetup(0, fl.pwmFrequency, fl.pwmResolutionBits);
  ledcAttachPin(fl.gpio, 0);
#endif
  _begun = true;
  setBrightness(0);
#endif
}

#if FREEINK_CAP_FRONTLIGHT

void FrontlightManager::beginMultiChannel() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;

  // attachInterruptArg / gpio_isr_handler_add need the shared ISR service. It is
  // process-wide, so a second install fails harmlessly if another driver got
  // there first.
  gpio_install_isr_service(0);

  // Supply-rail gate. Plain GPIO, never PWM: chopping the rail would fight the
  // boost converter's own feedback loop, so brightness rides the sink channel.
  gpio_config_t railCfg = {};
  railCfg.pin_bit_mask = 1ULL << fl.railEnableGpio;
  railCfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&railCfg);
  setRail(false);

  if (fl.senseGpio != BoardConfig::PIN_UNASSIGNED) {
    gpio_config_t senseCfg = {};
    senseCfg.pin_bit_mask = 1ULL << fl.senseGpio;
    senseCfg.mode = GPIO_MODE_INPUT;
    senseCfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&senseCfg);
    // LOW = in regulation, so a rising edge is the fault.
    gpio_set_intr_type(static_cast<gpio_num_t>(fl.senseGpio), GPIO_INTR_POSEDGE);
  }

  ledc_timer_config_t brightTimer = {};
  brightTimer.speed_mode = kSpeed;
  brightTimer.duty_resolution = static_cast<ledc_timer_bit_t>(fl.pwmResolutionBits);
  brightTimer.timer_num = kTimerBright;
  brightTimer.freq_hz = fl.pwmFrequency;
  brightTimer.clk_cfg = LEDC_USE_RC_FAST_CLK;
  ledc_timer_config(&brightTimer);

  ledc_timer_config_t blendTimer = {};
  blendTimer.speed_mode = kSpeed;
  blendTimer.duty_resolution = static_cast<ledc_timer_bit_t>(fl.pwmResolutionBits);
  blendTimer.timer_num = kTimerBlend;
  blendTimer.freq_hz = fl.blendFrequency;
  blendTimer.clk_cfg = LEDC_USE_RC_FAST_CLK;
  ledc_timer_config(&blendTimer);

  const struct {
    ledc_channel_t channel;
    ledc_timer_t timer;
    int8_t gpio;
  } channels[] = {
      {kChBright, kTimerBright, fl.gpio},
      {kChWarm, kTimerBlend, fl.warmGpio},
      {kChCool, kTimerBlend, fl.coolGpio},
  };
  for (const auto& c : channels) {
    ledc_channel_config_t cfg = {};
    cfg.gpio_num = c.gpio;
    cfg.speed_mode = kSpeed;
    cfg.channel = c.channel;
    cfg.intr_type = LEDC_INTR_DISABLE;
    cfg.timer_sel = c.timer;
    cfg.duty = 0;
    // The blend channels share hpoint 0 deliberately: the overlap is
    // make-before-break. Briefly conducting both sinks is harmless (they are
    // series-resistor limited); leaving the converter output open is not.
    cfg.hpoint = 0;
    ledc_channel_config(&cfg);
  }
}

void FrontlightManager::setRail(bool on) {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (fl.railEnableGpio == BoardConfig::PIN_UNASSIGNED) return;
  // P-FET gate: pulling LOW turns the rail ON.
  gpio_set_level(static_cast<gpio_num_t>(fl.railEnableGpio), on ? 0 : 1);
}

void FrontlightManager::probeIsr(void* arg) { static_cast<FrontlightManager*>(arg)->_probeTriggered = true; }

void FrontlightManager::senseIsr(void* arg) {
  auto* self = static_cast<FrontlightManager*>(arg);
  const auto& fl = BoardConfig::ACTIVE.frontlight;

  // Re-read the line: only a genuinely HIGH sense is a fault, which filters the
  // edge glitches the sinks produce as they switch.
  if (gpio_get_level(static_cast<gpio_num_t>(fl.senseGpio)) == 0) return;

  self->_hardwareFault = true;
  self->_enabled = false;
  self->_senseArmed = false;
  // Cut the sinks immediately. The rail is dropped later from the main loop via
  // disable()/clearFault(), so this handler stays short and allocation-free.
  zeroAllChannels();
}

void FrontlightManager::armSense() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (fl.senseGpio == BoardConfig::PIN_UNASSIGNED || _senseArmed) return;
  _senseArmed = true;
  gpio_isr_handler_add(static_cast<gpio_num_t>(fl.senseGpio), senseIsr, this);
}

void FrontlightManager::disarmSense() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (fl.senseGpio == BoardConfig::PIN_UNASSIGNED) return;
  _senseArmed = false;
  // Unconditional: harmless when nothing is attached, and it stops a stale
  // handler firing across an enable/disable cycle.
  gpio_isr_handler_remove(static_cast<gpio_num_t>(fl.senseGpio));
}

bool FrontlightManager::probeRegulation() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  const uint32_t full = maxDuty(fl.pwmResolutionBits);

  // No sense line means nothing to probe against, so trust the rail.
  if (fl.senseGpio == BoardConfig::PIN_UNASSIGNED) {
    setRail(true);
    return true;
  }

  _probeTriggered = false;
  setRail(true);

  // Falling edge = the converter reached regulation.
  attachInterruptArg(digitalPinToInterrupt(fl.senseGpio), probeIsr, this, FALLING);

  // Drive everything hard: the converter needs a real load to regulate into.
  writeDuty(kChBright, full);
  writeDuty(kChWarm, full);
  writeDuty(kChCool, full);

  const uint32_t start = millis();
  while (!_probeTriggered && (millis() - start) < PROBE_TIMEOUT_MS) {
    delay(1);
  }

  detachInterrupt(digitalPinToInterrupt(fl.senseGpio));

  if (!_probeTriggered) {
    // Only tear down on failure. On success the channels stay driven, so there
    // is no open-circuit gap between the probe and the real duty being applied.
    zeroAllChannels();
    setRail(false);
  }
  return _probeTriggered;
}

void FrontlightManager::updateMultiChannel() {
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  const uint32_t full = maxDuty(fl.pwmResolutionBits);

  if (!_enabled || _hardwareFault) {
    zeroAllChannels();
    return;
  }

  writeDuty(kChBright, (static_cast<uint32_t>(_brightness) * full) / 100u);

  // Cool before warm, so the pair is never simultaneously off: see the hpoint
  // note in beginMultiChannel().
  const uint32_t warmDuty = (static_cast<uint32_t>(_warmPercent) * full) / 100u;
  const uint32_t coolDuty = ((100u - static_cast<uint32_t>(_warmPercent)) * full) / 100u;
  writeDuty(kChCool, coolDuty);
  writeDuty(kChWarm, warmDuty);
}

#endif  // FREEINK_CAP_FRONTLIGHT

void FrontlightManager::setBrightness(uint8_t percent) {
#if FREEINK_CAP_FRONTLIGHT
  const auto& fl = BoardConfig::ACTIVE.frontlight;
  if (!_begun || fl.gpio == BoardConfig::PIN_UNASSIGNED) return;
  if (percent > 100) percent = 100;

  if (hasColorTemperature()) {
    // 0% would starve the converter of load, so route it to a real shutdown.
    if (percent == 0) {
      disable();
      return;
    }
    if (percent < MIN_BRIGHTNESS) percent = MIN_BRIGHTNESS;
    _brightness = percent;
    _lastBrightness = percent;
    if (!_enabled) {
      enable();  // raises the rail and applies the new duty
      return;
    }
    updateMultiChannel();
    return;
  }

  _brightness = percent;
  if (percent > 0) _lastBrightness = percent;

  const uint32_t full = maxDuty(fl.pwmResolutionBits);
  uint32_t duty = (static_cast<uint32_t>(percent) * full) / 100u;
  if (!fl.activeHigh) duty = full - duty;

#if defined(ARDUINO) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(fl.gpio, duty);
#else
  ledcWrite(0, duty);
#endif
#else
  (void)percent;
#endif
}

void FrontlightManager::off() {
#if FREEINK_CAP_FRONTLIGHT
  if (hasColorTemperature()) {
    disable();
    return;
  }
#endif
  setBrightness(0);
}

void FrontlightManager::on() { setBrightness(_lastBrightness); }

void FrontlightManager::setColorTemperature(uint8_t warmPercent) {
  _warmPercent = warmPercent > 100 ? 100 : warmPercent;
#if FREEINK_CAP_FRONTLIGHT
  if (hasColorTemperature() && _enabled) updateMultiChannel();
#endif
}

bool FrontlightManager::enable() {
#if FREEINK_CAP_FRONTLIGHT
  if (!hasColorTemperature()) {
    on();
    _enabled = _brightness > 0;
    return true;
  }
  if (!_begun) return false;

  disarmSense();
  _hardwareFault = false;
  _enabled = false;

  if (_brightness < MIN_BRIGHTNESS) _brightness = _lastBrightness;
  if (_brightness < MIN_BRIGHTNESS) _brightness = MIN_BRIGHTNESS;

  if (!probeRegulation()) {
    _hardwareFault = true;
    return false;
  }

  _enabled = true;
  updateMultiChannel();

  delay(SENSE_SETTLE_MS);
  armSense();
  return true;
#else
  return false;
#endif
}

void FrontlightManager::disable() {
#if FREEINK_CAP_FRONTLIGHT
  if (!hasColorTemperature()) {
    _enabled = false;
    setBrightness(0);
    return;
  }
  disarmSense();
  _enabled = false;
  _brightness = 0;
  zeroAllChannels();
  setRail(false);
#endif
}
