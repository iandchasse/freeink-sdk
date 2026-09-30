#include "SdmmcBlockDevice.h"

#if FREEINK_SD_SDMMC

// SdmmcPins::parkLowWhenOff mount sequence, kept apart from the legacy mount in
// SdmmcBlockDevice.cpp (which logs through Serial) so it can log through
// Logging.h.

#include <Arduino.h>
#include <Logging.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "sdmmc_cmd.h"

namespace freeink {

// SdmmcPins::parkLowWhenOff mount: every attempt is a real power cycle. The card
// is gated off with the bus driven LOW (pulls off) for at least powerOffMs —
// counted from the park, which may be the boot park in holdPowerRails() — then
// the bus is released to Hi-Z, the card is powered and allowed to settle, and
// only then does the host touch the pins. On failure the host is torn down and
// the next attempt parks again; after the last one the card is left gated off
// with the bus parked, since there is no card-detect and a missing card is the
// usual reason.
bool SdmmcBlockDevice::beginParked(const BoardConfig::SdmmcPins& pins) {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = pins.maxFreqKhz != 0 ? pins.maxFreqKhz : SDMMC_FREQ_DEFAULT;

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = pins.busWidth;
  slot.clk = static_cast<gpio_num_t>(pins.clk);
  slot.cmd = static_cast<gpio_num_t>(pins.cmd);
  slot.d0 = static_cast<gpio_num_t>(pins.d0);
  slot.d1 = static_cast<gpio_num_t>(pins.d1);
  slot.d2 = static_cast<gpio_num_t>(pins.d2);
  slot.d3 = static_cast<gpio_num_t>(pins.d3);
  if (pins.internalPullups) slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  auto* card = static_cast<sdmmc_card_t*>(calloc(1, sizeof(sdmmc_card_t)));
  _dmaBuffer = static_cast<uint8_t*>(heap_caps_malloc(kSectorSize * kMaxTransferSectors,
                                                      MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!card || !_dmaBuffer) {
    LOG_ERR("SD", "SDMMC mount: out of memory");
    free(card);
    heap_caps_free(_dmaBuffer);
    _dmaBuffer = nullptr;
    BoardConfig::parkSdmmcBus();
    return false;
  }

  esp_err_t mountErr = ESP_FAIL;
  for (int attempt = 0; attempt < kParkedMountAttempts; attempt++) {
    BoardConfig::parkSdmmcBus();
    BoardConfig::unparkSdmmcBus(pins.powerOffMs);
    delay(pins.powerOnSettleMs);

    mountErr = sdmmc_host_init();
    if (mountErr == ESP_OK) mountErr = sdmmc_host_init_slot(host.slot, &slot);
    if (mountErr == ESP_OK) {
      memset(card, 0, sizeof(*card));
      mountErr = sdmmc_card_init(&host, card);
    }
    // Prove real block I/O before publishing the card, as the legacy mount does.
    if (mountErr == ESP_OK) mountErr = sdmmc_read_sectors(card, _dmaBuffer, 0, 1);
    if (mountErr == ESP_OK) break;
    LOG_DBG("SD", "SDMMC mount attempt %d failed: %s", attempt + 1, esp_err_to_name(mountErr));
    sdmmc_host_deinit();  // no-op if the host never came up
  }

  if (mountErr != ESP_OK) {
    LOG_ERR("SD", "SDMMC mount failed after %d power cycles: %s", kParkedMountAttempts, esp_err_to_name(mountErr));
    heap_caps_free(_dmaBuffer);
    _dmaBuffer = nullptr;
    free(card);
    BoardConfig::parkSdmmcBus();
    delay(20);  // bus LOW while SD_VDD bleeds down
    return false;
  }

  if (!pins.internalPullups) {
    // Slot init and the 4-bit switch reset the pads (gpio_reset_pin), which turns
    // their internal pull-ups back on. The external ones are enough.
    for (const int8_t pin : {pins.clk, pins.cmd, pins.d0, pins.d1, pins.d2, pins.d3}) {
      if (pin >= 0) gpio_pullup_dis(static_cast<gpio_num_t>(pin));
    }
  }
  _card = card;
  return true;
}

}  // namespace freeink

#endif  // FREEINK_SD_SDMMC
