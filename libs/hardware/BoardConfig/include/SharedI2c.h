#pragma once

// FreeInk SDK — the board's shared I2C sensor bus.
//
// SensorsConfig describes the bus the on-board sensors use, and other drivers
// (a touch controller, a gauge) may sit on the same pins. Arduino's Wire ignores
// a second begin() on a running controller, so whichever driver starts the bus
// first sets its clock. These helpers make SensorsConfig::i2cHz that clock for
// every driver on the sensor-bus pins, and implement SensorsConfig::i2cBusRecovery
// (free a stuck bus at boot, release the bus before deep sleep; compiled in with
// FREEINK_CAP_I2C_RECOVERY).

#include <Arduino.h>
#include <BoardConfig.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp32-hal-periman.h>
#include <esp_rom_sys.h>
#include <soc/soc_caps.h>

namespace freeink {
namespace i2c {

// Arduino controller behind the sensor bus (SensorsConfig::i2cBus).
inline TwoWire& sensorWire() {
#if SOC_I2C_NUM > 1
  return BoardConfig::ACTIVE.sensors.i2cBus == 1 ? Wire1 : Wire;
#else
  return Wire;
#endif
}

inline bool isSensorBus(const int8_t sda, const int8_t scl) {
  const BoardConfig::SensorsConfig& s = BoardConfig::ACTIVE.sensors;
  return sda >= 0 && scl >= 0 && sda == s.i2cSda && scl == s.i2cScl;
}

// Clock for a driver on (sda, scl): the sensor-bus clock when it shares those
// pins, otherwise the driver's own rate.
inline uint32_t busHz(const int8_t sda, const int8_t scl, const uint32_t ownHz) {
  const uint32_t sensorHz = BoardConfig::ACTIVE.sensors.i2cHz;
  return isSensorBus(sda, scl) && sensorHz != 0 ? sensorHz : ownHz;
}

// Free a bus whose SDA a slave holds LOW (a transfer cut short by a reset or a
// sleep): up to nine SCL clocks let it finish its byte, then a STOP. Both lines
// are driven open-drain, so they only ever pull LOW against the external
// pull-ups, and they end as inputs with no pulls. Only for a bus no I2C driver
// currently owns. Returns false when SDA is still LOW (SCL held by a slave, or
// a shorted line).
inline bool recoverStuckBus(const int8_t sda, const int8_t scl) {
  if (sda < 0 || scl < 0) return true;
  const auto sdaPin = static_cast<gpio_num_t>(sda);
  const auto sclPin = static_cast<gpio_num_t>(scl);
  constexpr uint32_t HALF_PERIOD_US = 5;  // 100 kHz
  gpio_set_level(sdaPin, 1);
  gpio_set_level(sclPin, 1);
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
  cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
  esp_rom_delay_us(HALF_PERIOD_US);
  if (gpio_get_level(sdaPin) == 0) {
    for (int i = 0; i < 9 && gpio_get_level(sdaPin) == 0; ++i) {
      gpio_set_level(sclPin, 0);
      esp_rom_delay_us(HALF_PERIOD_US);
      gpio_set_level(sclPin, 1);
      esp_rom_delay_us(HALF_PERIOD_US);
    }
    // STOP: SDA rises while SCL is HIGH.
    gpio_set_level(sclPin, 0);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(sdaPin, 0);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(sclPin, 1);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(sdaPin, 1);
    esp_rom_delay_us(HALF_PERIOD_US);
  }
  const bool released = gpio_get_level(sdaPin) != 0;
  cfg.mode = GPIO_MODE_INPUT;
  gpio_config(&cfg);
  return released;
}

// SensorsConfig::i2cBusRecovery, folded to false without FREEINK_CAP_I2C_RECOVERY.
inline bool busRecoveryEnabled() {
#if FREEINK_CAP_I2C_RECOVERY
  return BoardConfig::ACTIVE.sensors.i2cBusRecovery;
#else
  return false;
#endif
}

#if FREEINK_CAP_I2C_RECOVERY
// Set once the sensor bus has had its boot check (i2cBusRecovery).
inline bool sensorBusChecked = false;
#endif

// Start `wire` on (sda, scl). The first start of an i2cBusRecovery sensor bus in
// a boot frees the bus first if a slave holds SDA LOW.
inline bool beginBus(TwoWire& wire, const int8_t sda, const int8_t scl, const uint32_t hz) {
#if FREEINK_CAP_I2C_RECOVERY
  if (!sensorBusChecked && busRecoveryEnabled() && isSensorBus(sda, scl)) {
    sensorBusChecked = true;
    // Never under a running controller: only a bus no driver has started yet.
    if (perimanGetPinBusType(static_cast<uint8_t>(sda)) != ESP32_BUS_TYPE_I2C_MASTER_SDA) recoverStuckBus(sda, scl);
  }
#endif
  return wire.begin(sda, scl, hz);
}

// Deep-sleep entry on an i2cBusRecovery board: end the sensor bus, then leave
// SDA/SCL as unheld inputs with no pulls so the external pull-ups keep them HIGH,
// clocking the bus free if a slave still holds SDA LOW. Wire::end() takes the
// bus lock, so a transfer in flight on another task finishes first. Call after
// the last transaction before sleep. Returns false when SDA stays LOW; true (and
// does nothing) on other boards.
inline bool releaseSensorBusForSleep() {
  const BoardConfig::SensorsConfig& s = BoardConfig::ACTIVE.sensors;
  if (!busRecoveryEnabled() || s.i2cSda < 0 || s.i2cScl < 0) return true;
  sensorWire().end();
  gpio_hold_dis(static_cast<gpio_num_t>(s.i2cSda));
  gpio_hold_dis(static_cast<gpio_num_t>(s.i2cScl));
  return recoverStuckBus(s.i2cSda, s.i2cScl);
}

}  // namespace i2c
}  // namespace freeink
