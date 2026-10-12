// Every check below is an assert(): a build with -DNDEBUG must not turn this into a pass that ran nothing.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>
#include "driver/Uc8279Driver.h"
#include "lut/Uc8279X3Luts.h"

int main() {
  using namespace freeink;
  constexpr size_t n = 792 / 8 * 528;
  EpdBus bus;
  Uc8279Driver driver;
  std::vector<uint8_t> bw(n, 0x88), lsb(n, 0xAA), msb(n, 0xCC);
  assert(driver.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
  assert(driver.grayscaleCapabilities().stripUploads && !driver.grayscaleCapabilities().asyncBase);
  driver.begin(bus);
  driver.displayGrayscaleBase(bus, bw.data(), RefreshMode::Half, false);
  driver.copyGrayscaleLsb(bus, lsb.data());
  driver.copyGrayscaleMsb(bus, msb.data());
  // Inputs are passed unchanged regardless of gray-pixel coverage.
  assert(bus.oldPlane == lsb && bus.newPlane == msb);
  driver.displayGray(bus, bw.data(), false, nullptr, false);
  assert(bus.rawRegisters == std::vector<uint8_t>({0x20, 0x23, 0x22, 0x21, 0x24}));
  assert(bus.lastBank == std::vector<uint8_t>(kUc8279X3_XtfAa[0], kUc8279X3_XtfAa[0] + 49));

  for (bool turnOff : {false, true}) {
    driver.displayGrayscaleBase(bus, bw.data(), RefreshMode::Half, false);
    driver.copyGrayscaleLsb(bus, lsb.data());
    driver.copyGrayscaleMsb(bus, msb.data());
    driver.displayGray(bus, bw.data(), turnOff, nullptr, true);
    assert(bus.rawRegisters == std::vector<uint8_t>({0x20, 0x24, 0x22, 0x23, 0x21}));
    assert(bus.lastBank == std::vector<uint8_t>(kUc8279X3_Xth4[0], kUc8279X3_Xth4[0] + 49));
    // Cleanup must not clear the required physical rebase after an absolute pass.
    driver.cleanupGrayscaleBuffers(bus, bw.data());
    driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, true);
    assert(bus.oldPlane == bw && bus.newPlane == bw);
    assert(bus.lastBwBank == std::vector<uint8_t>(kUc8279X3_BwGc[0]+1, kUc8279X3_BwGc[0]+43));
  }
  for (auto fallback : {RefreshMode::Half, RefreshMode::Fast}) {
    const auto before = bus.refreshes;
    driver.beginGrayscale(bus, bw.data(), GrayscaleMode::Direct, fallback, false);
    driver.copyGrayscaleLsb(bus, lsb.data());
    driver.copyGrayscaleMsb(bus, msb.data());
    assert(bus.refreshes == before);
    assert(bus.oldPlane == lsb && bus.newPlane == msb);
    driver.displayGray(bus, bw.data(), false, nullptr, true);
    assert(bus.refreshes == before + 1);
    assert(bus.lastBank == std::vector<uint8_t>(kUc8279X3_Xth4[0], kUc8279X3_Xth4[0] + 49));
    driver.cleanupGrayscaleBuffers(bus, bw.data());
    driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, true);
    assert(bus.oldPlane == bw && bus.newPlane == bw);
  }
  driver.deepSleep(bus);
  driver.begin(bus);
  driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, false);
  assert(bus.oldPlane == bw && bus.newPlane == bw);

  // Nothing above may have called the bus while a beginTxn() Transaction was held: EpdBus::cmd() opens a
  // transaction of its own, and the second one blocks its task for ever on arduino-esp32 (non-recursive SPI mutex).
  assert(bus.model.nestedCalls == 0);

  // Per-strip upload, the reader's gray-page route on this driver (stripUploads): the PTL window in gate
  // space, the plane's RAM command, the rows bottom-first as ONE CS-low burst, then PTOUT once that
  // burst's Transaction has ended. PTOUT inside the burst was the soft lock a grayscale image page caused.
  {
    constexpr uint16_t kRows = 3, kY = 100, kWb = 792 / 8;
    std::vector<uint8_t> strip(size_t(kRows) * kWb);
    for (uint16_t r = 0; r < kRows; ++r) {
      std::fill_n(strip.begin() + size_t(r) * kWb, kWb, static_cast<uint8_t>(0x11 * (r + 1)));
    }
    for (const auto plane : {GrayPlane::Lsb, GrayPlane::Msb}) {
      bus.model.clear();
      const unsigned opened = bus.model.opened;
      driver.writeGrayscalePlaneStrip(bus, plane, strip.data(), kY, kRows);
      assert(bus.model.nestedCalls == 0);
      assert(bus.model.maxAlive == 1 && bus.model.alive == 0 && !bus.model.csLow());
      assert(bus.model.opened == opened + 1);  // one burst for the whole strip

      const auto& log = bus.model.log;
      assert(log.size() == 3u + 1u + kRows + 1u + 1u);
      assert(log[0].kind == BusEvent::Cmd && log[0].value == 0x91);  // PTIN
      // PTL: X 0..791, the gate rows of logical rows 100..102 (527 - 102 .. 527 - 100), PT_SCAN
      assert(log[1].kind == BusEvent::CmdData && log[1].value == 0x90);
      assert(log[1].bytes == std::vector<uint8_t>({0x00, 0x00, 0x03, 0x17, 0x01, 0xA9, 0x01, 0xAB, 0x01}));
      assert(log[2].kind == BusEvent::Cmd && log[2].value == (plane == GrayPlane::Lsb ? 0x10 : 0x13));
      assert(log[3].kind == BusEvent::Open);
      for (uint16_t r = 0; r < kRows; ++r) {  // bottom row first
        const auto& w = log[4 + r];
        assert(w.kind == BusEvent::Write && w.bytes.size() == kWb);
        assert(w.bytes == std::vector<uint8_t>(kWb, static_cast<uint8_t>(0x11 * (kRows - r))));
      }
      assert(log[4 + kRows].kind == BusEvent::End);  // CS released ...
      assert(log[5 + kRows].kind == BusEvent::Cmd && log[5 + kRows].value == 0x92);  // ... then PTOUT
    }
    // The LSB strip armed the MSB upload, as the whole-plane LSB does.
    std::vector<uint8_t> msbPlane(size_t(792 / 8) * 528, 0x5A);
    driver.writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, strip.data(), kY, kRows);
    driver.copyGrayscaleMsb(bus, msbPlane.data());
    assert(bus.newPlane == msbPlane);

    // Out-of-range and empty strips touch nothing.
    bus.model.clear();
    driver.writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, strip.data(), 527, kRows);
    driver.writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, strip.data(), 528, 1);
    driver.writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, strip.data(), 0, 0);
    driver.writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, nullptr, 0, 1);
    assert(bus.model.log.empty());
  }
  assert(bus.model.nestedCalls == 0 && bus.model.alive == 0);
  std::cout << "PASS: UC8279 overlay/absolute LUT mapping, unchanged planes, cleanup, wake and strip uploads\n";
}
