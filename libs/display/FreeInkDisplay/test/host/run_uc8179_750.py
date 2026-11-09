#!/usr/bin/env python3
"""Run the GDEY075T7 driver and facade against the recording SPI bus."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
LIB = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="freeink-750-") as directory:
    root = Path(directory)
    for folder in ("src/driver", "src/bus", "include"):
        (root / folder).mkdir(parents=True, exist_ok=True)
    for header in (LIB / "src/driver").glob("*.h"):
        shutil.copy2(header, root / "src/driver" / header.name)
    shutil.copy2(LIB / "src/driver/Gdey075t7Driver.cpp", root / "src/driver/Gdey075t7Driver.cpp")
    shutil.copy2(LIB / "src/FreeInkDisplay.cpp", root / "src/FreeInkDisplay.cpp")
    for name in ("FreeInkDisplay.h", "LgfxEpdConfig.h", "GrayscaleCapabilities.h"):
        shutil.copy2(LIB / "include" / name, root / "include" / name)
    for name in ("Arduino.h", "BoardConfig.h", "SPI.h", "esp_heap_caps.h", "sdkconfig.h"):
        shutil.copy2(HERE / "pro_stubs" / name, root / name)
    shutil.copy2(HERE / "pro_stubs/EpdBus.h", root / "src/bus/EpdBus.h")
    for single in (True, False):
        command = [os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                   "-Wno-unused-parameter", "-Wno-unused-function", "-Wno-unused-variable",
                   "-DARDUINO=1", "-DBOARD_HAS_PSRAM=1",
                   "-DFREEINK_DRIVER_UC8179_750=1", "-I" + str(root), "-I" + str(root / "include")]
        if single:
            command += ["-DEINK_DISPLAY_SINGLE_BUFFER_MODE=1"]
        binary = root / ("single" if single else "dual")
        command += [str(HERE / "test_uc8179_750.cpp"), str(root / "src/FreeInkDisplay.cpp"),
                    str(root / "src/driver/Gdey075t7Driver.cpp"), "-o", str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
