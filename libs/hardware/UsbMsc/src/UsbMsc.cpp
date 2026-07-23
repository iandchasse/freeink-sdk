#include "UsbMsc.h"

#if FREEINK_CAP_USB_MSC

#include <SDCardManager.h>
#include <driver/sdmmc_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <soc/usb_serial_jtag_struct.h>

namespace freeink {
namespace {

// One SD card, one USB device, so a single session is all there can be. File
// scope rather than members because the write-back task needs to reach it.
sdmmc_card_t* g_card = nullptr;

uint8_t* g_writeBuf = nullptr;
uint32_t g_writeBufSize = 0;
uint32_t g_writeLba = 0;
uint32_t g_writeSectors = 0;
SemaphoreHandle_t g_writeDone = nullptr;
TaskHandle_t g_writeTask = nullptr;
volatile bool g_writePending = false;
volatile bool g_writeOk = true;

// Block until the background commit finishes. Every read and every new write
// goes through here first, so the host can never observe a stale sector.
void waitForPendingWrite() {
  if (!g_writePending) return;
  xSemaphoreTake(g_writeDone, portMAX_DELAY);
  g_writePending = false;
}

// Take the error from the last commit and reset it, so a failed write is
// reported to the host exactly once.
bool takeWriteError() {
  if (g_writeOk) return false;
  g_writeOk = true;
  return true;
}

void writeTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    g_writeOk = sdmmc_write_sectors(g_card, g_writeBuf, g_writeLba, g_writeSectors) == ESP_OK;
    xSemaphoreGive(g_writeDone);
  }
}

}  // namespace

bool UsbMsc::begin() { return begin(Config{}); }

bool UsbMsc::begin(const Config& config) {
  if (_active) return true;

  g_card = SdMan.sdmmcCard();
  if (g_card == nullptr || g_card->csd.sector_size == 0) return false;

  // DMA-capable: the SDMMC peripheral reads this buffer directly.
  if (g_writeBuf == nullptr) {
    g_writeBuf = static_cast<uint8_t*>(heap_caps_malloc(config.writeBufferBytes, MALLOC_CAP_DMA));
    if (g_writeBuf == nullptr) return false;
    g_writeBufSize = config.writeBufferBytes;
  }
  if (g_writeDone == nullptr) {
    g_writeDone = xSemaphoreCreateBinary();
    if (g_writeDone == nullptr) return false;
  }
  if (g_writeTask == nullptr) {
    // Above the USB stack's own priority so a queued commit is not left waiting
    // behind the next transfer.
    if (xTaskCreate(writeTask, "usbmsc_write", 4096, nullptr, 5, &g_writeTask) != pdPASS) {
      g_writeTask = nullptr;
      return false;
    }
  }

  _active = true;
  return true;
}

uint32_t UsbMsc::sectorSize() const { return g_card ? g_card->csd.sector_size : 0; }

uint32_t UsbMsc::sectorCount() const { return g_card ? static_cast<uint32_t>(g_card->csd.capacity) : 0; }

int32_t UsbMsc::readSectors(uint32_t lba, void* buffer, uint32_t bytes) {
  const uint32_t secSize = sectorSize();
  if (!secSize) return -1;
  const uint32_t sectors = bytes / secSize;
  if (sectors == 0) return 0;

  waitForPendingWrite();
  if (takeWriteError()) return -1;

  return sdmmc_read_sectors(g_card, buffer, lba, sectors) == ESP_OK ? static_cast<int32_t>(bytes) : -1;
}

int32_t UsbMsc::writeSectors(uint32_t lba, const uint8_t* buffer, uint32_t bytes) {
  const uint32_t secSize = sectorSize();
  if (!secSize) return -1;
  const uint32_t sectors = bytes / secSize;
  if (sectors == 0) return 0;

  waitForPendingWrite();
  if (takeWriteError()) return -1;

  // Buffer and acknowledge immediately where it fits; the task commits it. A
  // transfer too large for the buffer commits inline instead.
  if (g_writeBuf != nullptr && bytes <= g_writeBufSize) {
    memcpy(g_writeBuf, buffer, bytes);
    g_writeLba = lba;
    g_writeSectors = sectors;
    g_writePending = true;
    xTaskNotifyGive(g_writeTask);
    return static_cast<int32_t>(bytes);
  }
  return sdmmc_write_sectors(g_card, buffer, lba, sectors) == ESP_OK ? static_cast<int32_t>(bytes) : -1;
}

void UsbMsc::flush() { waitForPendingWrite(); }

void UsbMsc::end() {
  if (!_active) return;
  waitForPendingWrite();

  // Release the PHY's control of the shared pins back to the GPIO matrix, then
  // drive D+/D- low briefly: a soft disconnect alone leaves some hosts still
  // believing the device is attached.
  USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
  USB_SERIAL_JTAG.conf0.dp_pullup = 0;
  USB_SERIAL_JTAG.conf0.pad_pull_override = 1;
  delay(10);

  constexpr int USB_DM_PIN = 19;
  constexpr int USB_DP_PIN = 20;
  pinMode(USB_DM_PIN, OUTPUT);
  pinMode(USB_DP_PIN, OUTPUT);
  digitalWrite(USB_DM_PIN, LOW);
  digitalWrite(USB_DP_PIN, LOW);
  delay(500);

  forceSerialJtagPhy();
  delay(100);
  _active = false;
}

void UsbMsc::forceSerialJtagPhy() {
  // Drop the software override of the PHY selection so it reverts to the
  // hardware/eFuse default, which routes to the serial/JTAG controller.
  CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL);
  CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_USB_PHY_SEL);

  // Re-assert the serial/JTAG controller's own defaults; a prior MSC session
  // left these configured for the OTG peripheral.
  USB_SERIAL_JTAG.conf0.phy_sel = 0;
  USB_SERIAL_JTAG.conf0.pad_pull_override = 0;
  USB_SERIAL_JTAG.conf0.dp_pullup = 1;
  USB_SERIAL_JTAG.conf0.usb_pad_enable = 1;
}

}  // namespace freeink

#else  // !FREEINK_CAP_USB_MSC

namespace freeink {
bool UsbMsc::begin() { return false; }
bool UsbMsc::begin(const Config&) { return false; }
uint32_t UsbMsc::sectorCount() const { return 0; }
uint32_t UsbMsc::sectorSize() const { return 0; }
int32_t UsbMsc::readSectors(uint32_t, void*, uint32_t) { return -1; }
int32_t UsbMsc::writeSectors(uint32_t, const uint8_t*, uint32_t) { return -1; }
void UsbMsc::flush() {}
void UsbMsc::end() {}
void UsbMsc::forceSerialJtagPhy() {}
}  // namespace freeink

#endif  // FREEINK_CAP_USB_MSC
