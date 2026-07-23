#include "UsbMsc.h"

#if FREEINK_CAP_USB_MSC

#include <SDCardManager.h>
#include <USB.h>
#include <USBMSC.h>
#include <driver/sdmmc_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <soc/usb_serial_jtag_struct.h>
#include <tusb.h>

namespace freeink {
namespace {

// The USBMSC callbacks are plain function pointers with no user argument, so the
// session state they need has to live at file scope. Only one USB device exists,
// so a single session is all there can be.
USBMSC* g_msc = nullptr;
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

// Takes the error from the last commit and resets it, so one failed write is
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

// Sector size from the card's CSD; 0 means the card is not usable.
uint32_t sectorSize() { return g_card ? g_card->csd.sector_size : 0; }

int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  const uint32_t secSize = sectorSize();
  if (!secSize) return -1;
  const uint32_t sectors = bufsize / secSize;
  if (sectors == 0) return 0;

  waitForPendingWrite();
  if (takeWriteError()) return -1;

  return sdmmc_read_sectors(g_card, buffer, lba, sectors) == ESP_OK ? static_cast<int32_t>(bufsize) : -1;
}

int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  const uint32_t secSize = sectorSize();
  if (!secSize) return -1;
  const uint32_t sectors = bufsize / secSize;
  if (sectors == 0) return 0;

  waitForPendingWrite();
  if (takeWriteError()) return -1;

  // Buffer and acknowledge immediately where it fits; the task commits it. A
  // transfer too large for the buffer commits inline instead.
  if (g_writeBuf != nullptr && bufsize <= g_writeBufSize) {
    memcpy(g_writeBuf, buffer, bufsize);
    g_writeLba = lba;
    g_writeSectors = sectors;
    g_writePending = true;
    xTaskNotifyGive(g_writeTask);
    return static_cast<int32_t>(bufsize);
  }
  return sdmmc_write_sectors(g_card, buffer, lba, sectors) == ESP_OK ? static_cast<int32_t>(bufsize) : -1;
}

bool onStartStop(uint8_t, bool, bool) {
  // The host ejecting the drive is its last chance to have data committed.
  waitForPendingWrite();
  return true;
}

}  // namespace

bool UsbMsc::begin(const Config& config) {
  if (_active) return true;

  g_card = SDCard.sdmmcCard();
  if (g_card == nullptr) return false;
  const uint32_t secSize = sectorSize();
  if (secSize == 0) return false;

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

  if (g_msc == nullptr) {
    static USBMSC msc;
    g_msc = &msc;
  }
  g_msc->vendorID(config.vendorId);
  g_msc->productID(config.productId);
  g_msc->productRevision(config.revision);
  g_msc->onRead(onRead);
  g_msc->onWrite(onWrite);
  g_msc->onStartStop(onStartStop);
  g_msc->mediaPresent(true);
  g_msc->begin(g_card->csd.capacity, secSize);

  USB.begin();
  _active = true;
  return true;
}

void UsbMsc::flush() { waitForPendingWrite(); }

void UsbMsc::end() {
  if (!_active) return;
  waitForPendingWrite();

  // Soft-disconnect so the host unmounts cleanly rather than reporting surprise
  // removal.
  if (tud_inited()) {
    tud_disconnect();
    delay(100);
  }

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
bool UsbMsc::begin(const Config&) { return false; }
void UsbMsc::flush() {}
void UsbMsc::end() {}
void UsbMsc::forceSerialJtagPhy() {}
}  // namespace freeink

#endif  // FREEINK_CAP_USB_MSC
