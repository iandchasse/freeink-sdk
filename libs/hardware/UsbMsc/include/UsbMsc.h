#pragma once

// FreeInk SDK — USB Mass Storage.
//
// Exposes the SD card to a host PC as a USB drive, so books can be copied on
// without going through the device's own web server. This is the hardware half
// only: mounting the card as a MSC LUN, servicing the host's sector reads and
// writes, and the USB PHY handling needed to get back out again. What the
// screen shows and how the user leaves the mode are the application's business.
//
// Sector I/O bypasses the filesystem completely — the host owns the FAT while
// this is active. The caller must therefore not have the volume mounted for
// writing at the same time; in practice a device enters MSC mode from a
// dedicated boot path and reboots on the way out.
//
// Writes are acknowledged to the host as soon as they are buffered and are
// committed by a background task, because a synchronous sdmmc_write_sectors()
// per request is slow enough that hosts time out on large copies. flush() (and
// every read) waits out an in-flight commit, so ordering is still correct.
//
// Requires native SDMMC (FREEINK_SD_SDMMC) and an S3-class USB-OTG peripheral;
// FREEINK_CAP_USB_MSC gates the implementation. Where it is off, every method
// links as a no-op returning false, so callers need no #ifdef.

#include <Arduino.h>
#include <BoardConfig.h>

namespace freeink {

class UsbMsc {
 public:
  struct Config {
    // Identification the host shows for the drive. SCSI pads/truncates these to
    // 8 / 16 / 4 characters respectively.
    const char* vendorId = "FreeInk";
    const char* productId = "SD Card";
    const char* revision = "1.0";
    // DMA-capable write-back buffer size. Writes larger than this bypass the
    // buffer and commit synchronously.
    uint32_t writeBufferBytes = 32 * 1024;
  };

  // Bring up USB MSC over the card that SDCardManager has already mounted.
  // Returns false if the card is unavailable, the buffer cannot be allocated,
  // or the build has no MSC support. begin() does NOT block: once it returns
  // true the host may start issuing transfers, and the caller is expected to
  // run its own idle loop.
  //
  // Two overloads rather than a `= {}` default argument: the toolchain (GCC 8.4)
  // rejects brace-initialising a nested struct in a default argument at every
  // language standard.
  bool begin();
  bool begin(const Config& config);

  // Wait for any buffered write to reach the card. Call before rebooting or
  // cutting power, or the last transfer the host believes it completed may not
  // be on the card.
  void flush();

  // Detach from the host and hand the USB PHY back to the serial/JTAG
  // controller, so the port enumerates as a console again after a restart.
  // Flushes first. Safe to call when never begun.
  void end();

  bool active() const { return _active; }

  // Route the USB PHY back to the serial/JTAG controller without tearing down a
  // running MSC session. Firmware that boots straight into MSC mode calls this
  // on the normal-boot path so a device that was last used as a drive still
  // comes back as a serial console.
  static void forceSerialJtagPhy();

 private:
  bool _active = false;
};

}  // namespace freeink
