#pragma once

// FreeInk SDK — SD sector access for USB Mass Storage.
//
// Named FreeInkUsbMsc.h rather than the obvious UsbMsc.h because Windows and
// macOS resolve includes case-insensitively: a header called UsbMsc.h in this
// directory shadows the Arduino core's USBMSC.h whenever this library's include
// path is searched first, so `#include <USBMSC.h>` silently lands here and the
// core's USBMSC class never gets declared.
//
// The hardware half of exposing the SD card to a host PC as a USB drive: raw
// sector reads and writes against the esp-idf card handle, the write-back task
// that keeps host transfers fast enough not to time out, and the USB PHY
// handling needed to get the port back to the serial/JTAG console afterwards.
//
// Deliberately does NOT touch the Arduino USB stack. Wiring these calls to a
// USBMSC LUN is a dozen lines that belong to the application, which already owns
// the USB device identity, the on-screen UI, and the decision to enter the mode.
// Keeping <USB.h>/<USBMSC.h> out of the SDK also keeps this library independent
// of which USB mode the consumer's build selects.
//
// Typical use:
//
//   freeink::UsbMsc sd;
//   if (sd.begin()) {
//     msc.onRead ([&](uint32_t lba, uint32_t, void* b, uint32_t n)   { return sd.readSectors (lba, b, n); });
//     msc.onWrite([&](uint32_t lba, uint32_t, uint8_t* b, uint32_t n){ return sd.writeSectors(lba, b, n); });
//     msc.onStartStop([&](uint8_t, bool, bool) { sd.flush(); return true; });
//     msc.begin(sd.sectorCount(), sd.sectorSize());
//   }
//
// Sector I/O bypasses the filesystem: the host owns the FAT while it is
// attached, so the caller must not have the volume mounted for writing at the
// same time. In practice a device enters this mode from a dedicated boot path
// and reboots on the way out.
//
// Requires native SDMMC (FREEINK_SD_SDMMC) and an S3-class MCU; gated by
// FREEINK_CAP_USB_MSC. Where it is off every method links as an inert stub, so
// callers need no #ifdef.

#include <Arduino.h>
#include <BoardConfig.h>

namespace freeink {

class UsbMsc {
 public:
  struct Config {
    // DMA-capable write-back buffer size. Writes larger than this bypass the
    // buffer and commit synchronously.
    uint32_t writeBufferBytes = 32 * 1024;
  };

  // Claim the card that SDCardManager has already mounted and start the
  // write-back task. Returns false if there is no card, the buffer cannot be
  // allocated, or the build has no MSC support.
  //
  // Two overloads rather than a `= {}` default argument: the toolchain (GCC 8.4)
  // rejects brace-initialising a nested struct in a default argument at every
  // language standard.
  bool begin();
  bool begin(const Config& config);

  // Card geometry, for the host's MSC LUN. Zero before a successful begin().
  uint32_t sectorCount() const;
  uint32_t sectorSize() const;

  // Sector I/O in the shape the USBMSC callbacks want: byte counts in and out,
  // negative on error. `bytes` must be a whole number of sectors; a partial
  // sector returns 0 (nothing transferred) rather than an error.
  //
  // Reads wait out any in-flight commit, so the host can never observe a stale
  // sector. Writes are acknowledged as soon as they are buffered and committed
  // by the background task, because a synchronous commit per request is slow
  // enough that hosts time out on large copies; a failed commit is reported to
  // the host exactly once, on the next transfer.
  int32_t readSectors(uint32_t lba, void* buffer, uint32_t bytes);
  int32_t writeSectors(uint32_t lba, const uint8_t* buffer, uint32_t bytes);

  // Wait for any buffered write to reach the card. Call from the host's
  // start/stop (eject) handler and before rebooting, or the last transfer the
  // host believes it completed may not be on the card.
  void flush();

  // Flush, then hand the USB PHY back to the serial/JTAG controller so the port
  // enumerates as a console again after a restart. Does not itself detach the
  // USB device — the application owns that, since it owns the USB stack.
  void end();

  bool active() const { return _active; }

  // Route the USB PHY back to the serial/JTAG controller. Firmware that can boot
  // straight into MSC mode also calls this on the normal-boot path, so a device
  // last used as a drive still comes back as a serial console.
  static void forceSerialJtagPhy();

 private:
  bool _active = false;
};

}  // namespace freeink
