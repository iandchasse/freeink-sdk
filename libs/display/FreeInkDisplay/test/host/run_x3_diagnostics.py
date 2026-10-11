#!/usr/bin/env python3
"""Compile the real X3 drivers (UC8253, UC8279d) with FREEINK_DIAGNOSTICS and a recording bus."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "src"
with tempfile.TemporaryDirectory(prefix="x3-diagnostics-test-") as directory:
    root = Path(directory)
    for folder in ("driver", "bus", "lut"):
        (root / folder).mkdir()
    for name in ("Uc8253X3Driver", "Uc8279Driver"):
        for ext in ("cpp", "h"):
            shutil.copy2(SOURCE / "driver" / f"{name}.{ext}", root / "driver" / f"{name}.{ext}")
    shutil.copy2(SOURCE / "driver/PanelDriver.h", root / "driver/PanelDriver.h")
    for name in ("Uc8253X3Luts.h", "UltraChipDirectGrayLuts.h", "Uc8279X3Luts.h"):
        shutil.copy2(SOURCE / "lut" / name, root / "lut" / name)
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
namespace freeink {
enum class BusyPolarity { X3TwoPhase };
// What EpdBus::noteRefresh() recorded, and how often the completion wait closed it.
struct RefreshReport {
  unsigned count = 0, closed = 0;
  uint8_t mode = 0xFF, sequence = 0;
  bool async = false, turnOff = false, customLut = false, inFlight = false;
};
class EpdBus {
public:
  static inline RefreshReport report;
  static void noteRefresh(uint8_t mode, uint8_t sequence, bool async, bool turnOff, bool customLut) {
    ++report.count;
    report.mode = mode; report.sequence = sequence; report.async = async;
    report.turnOff = turnOff; report.customLut = customLut; report.inFlight = true;
  }
  void cmd(uint8_t) {}
  void data(uint8_t) {}
  void data(const uint8_t*, size_t) {}
  void cmdData(uint8_t, const uint8_t*, size_t) {}
  void cmdData2(uint8_t, uint8_t, uint8_t) {}
  void sendPlaneFlipped(uint8_t, const uint8_t*, uint16_t, uint16_t) {}
  void sendPlaneFlippedInverted(uint8_t, const uint8_t*, uint16_t, uint16_t) {}
  void fillPlane(uint8_t, uint8_t, uint16_t, uint16_t) {}
  void reset(int) {}
  void waitBusy(const char*) {}
  void waitRefreshComplete(const char*) {
    if (report.inFlight) { report.inFlight = false; ++report.closed; }
  }
  struct Pins { int8_t busy = 0; };
  Pins pins() const { return {}; }
  struct Transaction { void writeBytes(const uint8_t*, size_t) {} void end() {} };
  Transaction beginTxn() { return {}; }
};
}
''')
    exe = root / "test_x3_diagnostics"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Wno-unused-parameter", "-DFREEINK_DIAGNOSTICS=1", "-I" + str(root),
                    str(HERE / "test_x3_diagnostics.cpp"),
                    str(root / "driver/Uc8253X3Driver.cpp"), str(root / "driver/Uc8279Driver.cpp"),
                    "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
