#!/usr/bin/env python3
"""Compile the real UC8253 X3 driver with a recording bus and host Arduino shims.

The recording bus models EpdBus's transaction contract (epdbus_txn_model.h): a bus call made while a
beginTxn() Transaction is alive would block its task for ever on hardware, and the test fails on it.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "src"
# UC8253_SOURCE names another copy of Uc8253X3Driver.cpp to run (a control: without the strip's txn.end() the
# transaction test below must fail)
DRIVER = Path(os.environ.get("UC8253_SOURCE", str(SOURCE / "driver/Uc8253X3Driver.cpp")))
with tempfile.TemporaryDirectory(prefix="uc8253_power-test-") as directory:
    root = Path(directory)
    for folder in ("driver", "bus", "lut"):
        (root / folder).mkdir()
    for name in ("Uc8253X3Driver.h", "PanelDriver.h"):
        shutil.copy2(SOURCE / "driver" / name, root / "driver" / name)
    shutil.copy2(DRIVER, root / "driver" / "Uc8253X3Driver.cpp")
    shutil.copy2(SOURCE / "lut/Uc8253X3Luts.h", root / "lut/Uc8253X3Luts.h")
    shutil.copy2(SOURCE / "lut/UltraChipDirectGrayLuts.h", root / "lut/UltraChipDirectGrayLuts.h")
    shutil.copy2(SOURCE.parent / "include/GrayscaleCapabilities.h", root / "GrayscaleCapabilities.h")
    shutil.copy2(HERE / "epdbus_txn_model.h", root / "epdbus_txn_model.h")
    panel = root / "driver/PanelDriver.h"
    panel.write_text(panel.read_text().replace("../../include/GrayscaleCapabilities.h", "../GrayscaleCapabilities.h"))
    (root / "Arduino.h").write_text('''#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#define HIGH 1
#define PROGMEM
#define log_e(...) std::fprintf(stderr, __VA_ARGS__)
inline unsigned long millis() { static unsigned long t; return ++t; }
inline void delay(unsigned long) {}
inline int digitalRead(int) { return 0; }
''')
    (root / "BoardConfig.h").write_text('''#pragma once
namespace BoardConfig {
inline struct { unsigned short displayWidth=792, displayHeight=528; unsigned displaySpiHz=0; } ACTIVE;
}
''')
    (root / "bus/EpdBus.h").write_text('''#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include "../epdbus_txn_model.h"
namespace freeink {
enum class BusyPolarity { X3TwoPhase };
class EpdBus {
  uint8_t command=0;
  void setCommand(uint8_t c) { command=c; if(c == 4) ++powerOns; if(c == 0x12) ++refreshes; }
  void payload(const uint8_t* p, size_t n) {
    if(n == 42 || n == 49) {
      if(command == 0x20) rawRegisters.clear();
      rawRegisters.push_back(command);
      if(command == 0x20) lastBank.assign(p,p+n);
    }
  }
public:
  std::vector<uint8_t> oldPlane, newPlane, lastBank, rawRegisters;
  unsigned powerOns=0, refreshes=0;
  // The real bus opens a Transaction for each call below and holds one for beginTxn(): see the model.
  EpdBusTxnModel model;
  using Transaction = EpdBusTxn;
  void cmd(uint8_t c) { model.call(BusEvent::Cmd, "cmd", c); setCommand(c); }
  void cmdData2(uint8_t c, uint8_t a, uint8_t b) {
    const uint8_t d[2] = {a, b};
    model.call(BusEvent::CmdData, "cmdData2", c, d, 2);
    setCommand(c);
  }
  void data(uint8_t) { model.call(BusEvent::Data, "data", -1, nullptr, 1); }
  void data(const uint8_t* p, size_t n) { model.call(BusEvent::Data, "data", -1, p, n); payload(p,n); }
  void cmdData(uint8_t c, const uint8_t* p, size_t n) {
    model.call(BusEvent::CmdData, "cmdData", c, p, n);
    setCommand(c);
    payload(p,n);
  }
  void sendPlaneFlipped(uint8_t c, const uint8_t* p, uint16_t h, uint16_t wb) {
    model.call(BusEvent::Plane, "sendPlaneFlipped", c, nullptr, size_t(h)*wb);
    (c == 0x10 ? oldPlane : newPlane).assign(p,p+size_t(h)*wb);
  }
  void fillPlane(uint8_t c, uint8_t v, uint16_t h, uint16_t wb) {
    model.call(BusEvent::Plane, "fillPlane", c, nullptr, size_t(h)*wb);
    (c == 0x10 ? oldPlane : newPlane).assign(size_t(h)*wb,v);
  }
  void reset(int) {}
  void waitBusy(const char*) {}
  void waitRefreshComplete(const char*) {}
  struct Pins { int8_t busy=0; };
  Pins pins() const { return {}; }
  Transaction beginTxn() { return Transaction(&model); }
};
}
''')
    (root / "test.cpp").write_text("""
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <vector>
#include <iostream>
#include "driver/Uc8253X3Driver.h"
int main() {
 using freeink::BusEvent;
 freeink::EpdBus bus; freeink::Uc8253X3Driver d;
 const auto caps = d.grayscaleCapabilities();
 assert(caps.supported() && caps.stripUploads && !caps.asyncBase && !caps.stagingWhileBusy);
 assert(d.grayscaleCapabilities(freeink::GrayscaleMode::Direct).base == freeink::GrayscaleBase::Combined);
 std::vector<uint8_t> fb(792/8*528, 0xAA); d.begin(bus);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Full,false);
 assert(bus.powerOns == 1);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Full,false);
 assert(bus.powerOns == 1);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Half,true);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false);
 assert(bus.powerOns == 2);
 for (auto fallback : {freeink::RefreshMode::Half, freeink::RefreshMode::Fast}) {
   const auto before = bus.refreshes;
   d.beginGrayscale(bus, fb.data(), freeink::GrayscaleMode::Direct, fallback, false);
   d.copyGrayscaleLsb(bus, fb.data());
   d.copyGrayscaleMsb(bus, fb.data());
   assert(bus.refreshes == before);
   d.displayGray(bus, fb.data(), false, nullptr, true);
   assert(bus.refreshes == before + 1);
   const auto vcom = freeink::uc8253X3DefaultConfig().directGray->vcom;
   assert(bus.lastBank == std::vector<uint8_t>(vcom, vcom + 42));
   d.cleanupGrayscaleBuffers(bus, fb.data());
   d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Full,false);
   assert(bus.newPlane == fb && bus.oldPlane == fb);
 }
 // Nothing above may have called the bus while a beginTxn() Transaction was held (the second blocks for ever).
 assert(bus.model.nestedCalls == 0);

 // Per-strip upload (the direct-gray route and the reader's gray pages): the PTL window in gate space, the
 // plane's RAM command, the rows bottom-first as ONE CS-low burst, then PTOUT once that burst has ended.
 {
   constexpr uint16_t kRows = 3, kY = 100, kWb = 792 / 8;
   std::vector<uint8_t> strip(size_t(kRows) * kWb);
   for (uint16_t r = 0; r < kRows; ++r) std::fill_n(strip.begin() + size_t(r) * kWb, kWb, uint8_t(0x11 * (r + 1)));
   for (const auto plane : {freeink::GrayPlane::Lsb, freeink::GrayPlane::Msb}) {
     bus.model.clear();
     d.writeGrayscalePlaneStrip(bus, plane, strip.data(), kY, kRows);
     assert(bus.model.nestedCalls == 0 && bus.model.maxAlive == 1 && bus.model.alive == 0);
     const auto& log = bus.model.log;
     assert(log.size() == 3u + 1u + kRows + 1u + 1u);
     assert(log[0].kind == BusEvent::Cmd && log[0].value == 0x91);
     assert(log[1].kind == BusEvent::CmdData && log[1].value == 0x90);
     assert(log[1].bytes == std::vector<uint8_t>({0x00, 0x00, 0x03, 0x17, 0x01, 0xA9, 0x01, 0xAB, 0x01}));
     assert(log[2].kind == BusEvent::Cmd && log[2].value == (plane == freeink::GrayPlane::Lsb ? 0x10 : 0x13));
     assert(log[3].kind == BusEvent::Open);
     for (uint16_t r = 0; r < kRows; ++r) {
       assert(log[4 + r].kind == BusEvent::Write);
       assert(log[4 + r].bytes == std::vector<uint8_t>(kWb, uint8_t(0x11 * (kRows - r))));
     }
     assert(log[4 + kRows].kind == BusEvent::End);
     assert(log[5 + kRows].kind == BusEvent::Cmd && log[5 + kRows].value == 0x92);
   }
 }
 assert(bus.model.nestedCalls == 0 && bus.model.alive == 0);
 std::cout << "PASS: UC8253 cold power-on, warm Full, power-off/wake and strip uploads\\n";
}
""")
    exe = root / "test_uc8253_power"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Wno-unused-parameter", "-I"+str(root), str(root / "test.cpp"),
                    str(root / "driver/Uc8253X3Driver.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
