#!/usr/bin/env python3
"""Compile the real UC8279 driver with a recording bus and host Arduino shims.

The recording bus models EpdBus's transaction contract (epdbus_txn_model.h): a bus call made while a
beginTxn() Transaction is alive would block its task for ever on hardware, and the test fails on it.
The driver is built as the Silkscreen 3.68" variant (-DFREEINK_SILKSCREEN_368=1), the build that carries the
strip-transaction fix. Two environment variables run the controls: UC8279_SOURCE names another copy of
Uc8279Driver.cpp (the source before the fix must fail this test), and FREEINK_SILKSCREEN_368=0 builds the
C3 X3's configuration, which keeps upstream's lines and fails it on the strip upload (a known upstream defect).
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "src"
DRIVER = Path(os.environ.get("UC8279_SOURCE", str(SOURCE / "driver/Uc8279Driver.cpp")))
VARIANT = os.environ.get("FREEINK_SILKSCREEN_368", "1")
with tempfile.TemporaryDirectory(prefix="uc8279-test-") as directory:
    root = Path(directory)
    for folder in ("driver", "bus", "lut"):
        (root / folder).mkdir()
    for name in ("Uc8279Driver.h", "PanelDriver.h"):
        shutil.copy2(SOURCE / "driver" / name, root / "driver" / name)
    shutil.copy2(DRIVER, root / "driver" / "Uc8279Driver.cpp")
    shutil.copy2(HERE / "epdbus_txn_model.h", root / "epdbus_txn_model.h")
    shutil.copy2(SOURCE / "lut/Uc8279X3Luts.h", root / "lut/Uc8279X3Luts.h")
    shutil.copy2(SOURCE.parent / "include/GrayscaleCapabilities.h", root / "GrayscaleCapabilities.h")
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
  void setCommand(uint8_t c) { command=c; if(c==0x12) ++refreshes; }
  void payload(const uint8_t* p, size_t n) {
    if(command == 0x20 && n == 42) lastBwBank.assign(p,p+n);
    if(n == 49) {
      if(command == 0x20) rawRegisters.clear();
      rawRegisters.push_back(command);
      if(command == 0x20) lastBank.assign(p,p+n);
    }
  }
public:
  std::vector<uint8_t> oldPlane, newPlane, lastBank, rawRegisters, lastBwBank;
  unsigned refreshes=0;
  // The real bus opens a Transaction for each call below and holds one for beginTxn(): see the model.
  EpdBusTxnModel model;
  using Transaction = EpdBusTxn;
  void cmd(uint8_t c) { model.call(BusEvent::Cmd, "cmd", c); setCommand(c); }
  void data(uint8_t) { model.call(BusEvent::Data, "data", -1, nullptr, 1); }
  void data(const uint8_t* p, size_t n) { model.call(BusEvent::Data, "data", -1, p, n); payload(p,n); }
  void cmdData(uint8_t c, const uint8_t* p, size_t n) {
    model.call(BusEvent::CmdData, "cmdData", c, p, n);
    setCommand(c);
    payload(p,n);
  }
  void sendPlaneFlippedInverted(uint8_t c, const uint8_t* p, uint16_t h, uint16_t wb) {
    model.call(BusEvent::Plane, "sendPlaneFlippedInverted", c, nullptr, size_t(h)*wb);
    auto& dst = c == 0x10 ? oldPlane : newPlane;
    dst.assign(p,p+size_t(h)*wb);
    for(auto& b : dst) b = ~b;
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
    exe = root / "test_uc8279"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Wno-unused-parameter", "-DFREEINK_SILKSCREEN_368=" + VARIANT, "-I"+str(root),
                    str(HERE / "test_uc8279.cpp"),
                    str(root / "driver/Uc8279Driver.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
