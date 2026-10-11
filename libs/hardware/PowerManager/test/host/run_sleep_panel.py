#!/usr/bin/env python3
"""Run PowerManager::sleepPanelWithoutDriver() as the firmware compiles it, for the 4.26" Silkscreen board and
for its 3.68" variant.

PowerManager.cpp needs the ESP-IDF sleep, RTC and ADC APIs to compile on a host, so this cuts the function
and the two helpers it uses out of the production file (it stops if it does not find exactly those), puts
them in a translation unit with a recording GPIO model, and runs test_sleep_panel.cpp against it once per
board. Needs a C++17 compiler: CXX, or c++. SLEEP_PANEL_SOURCE names another copy of PowerManager.cpp to cut
from (a mutated copy, as a negative control).
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
LIBS = HERE.parents[3]
SOURCE = Path(os.environ.get("SLEEP_PANEL_SOURCE", HERE.parent.parent / "src" / "PowerManager.cpp"))
FUNCTION = "bool PowerManager::sleepPanelWithoutDriver() {"
HELPERS = "namespace {\n// One SPI mode-0 byte, MSB first, on plain GPIOs (sleepPanelWithoutDriver()).\n"


def matching_brace(text: str, open_at: int) -> int:
    """Index of the brace that closes the one at open_at, skipping comments, strings and char literals."""
    depth = 0
    i = open_at
    while i < len(text):
        c = text[i]
        if text.startswith("//", i):
            i = text.index("\n", i)
            continue
        if text.startswith("/*", i):
            i = text.index("*/", i) + 2
            continue
        if c in "\"'":
            j = i + 1
            while text[j] != c:
                j += 2 if text[j] == "\\" else 1
            i = j + 1
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise SystemExit("run_sleep_panel: unbalanced braces in PowerManager.cpp")


def cut_production_text() -> str:
    text = SOURCE.read_text(encoding="utf-8").replace("\r\n", "\n")
    for needle in (HELPERS, FUNCTION):
        if text.count(needle) != 1:
            raise SystemExit(f"run_sleep_panel: expected one occurrence in PowerManager.cpp of: {needle.strip()!r}")
    helpers_at = text.index(HELPERS)
    function_at = text.index(FUNCTION)
    if helpers_at > function_at:
        raise SystemExit("run_sleep_panel: the helpers no longer precede the function")
    end = matching_brace(text, function_at + len(FUNCTION) - 1)
    return text[helpers_at:end + 1] + "\n"


ARDUINO_H = """#pragma once
#include <cstddef>
#include <cstdint>
#include <initializer_list>
constexpr int HIGH = 1, LOW = 0, INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5, INPUT_PULLDOWN = 9;
#define RTC_DATA_ATTR
void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
void delay(unsigned long);
void delayMicroseconds(unsigned int);
unsigned long millis();
struct SerialStub {
  explicit operator bool() const { return false; }
  template <typename... Args> void printf(const char*, Args...) {}
};
inline SerialStub Serial;
"""

GPIO_H = """#pragma once
using gpio_num_t = int;
enum gpio_mode_t { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 };
void gpio_hold_dis(gpio_num_t);
void gpio_hold_en(gpio_num_t);
void gpio_set_level(gpio_num_t, int);
void gpio_set_direction(gpio_num_t, gpio_mode_t);
void gpio_pullup_dis(gpio_num_t);
void gpio_pulldown_dis(gpio_num_t);
"""

BOARDS = (
    ("4.26\" board", []),
    ("3.68\" variant", ["-DSILKSCREEN_PANEL_368=1", "-DFREEINK_CAP_TOUCH=0"]),
)


def main() -> None:
    production = cut_production_text()
    cxx = os.environ.get("CXX", "c++")
    with tempfile.TemporaryDirectory(prefix="sleep-panel-test-") as directory:
        root = Path(directory)
        (root / "stubs" / "driver").mkdir(parents=True)
        (root / "stubs" / "Arduino.h").write_text(ARDUINO_H)
        (root / "stubs" / "driver" / "gpio.h").write_text(GPIO_H)
        (root / "sleep_panel_under_test.inc").write_text(production)
        for label, flags in BOARDS:
            exe = root / ("test_sleep_panel_" + ("variant" if flags else "board"))
            subprocess.run(
                [cxx, "-std=c++17", "-Wall", "-Wextra", "-Wno-unused-parameter", "-DFREEINK_DEVICE_SILKSCREEN=1", *flags,
                 "-I" + str(root), "-I" + str(root / "stubs"),
                 "-I" + str(LIBS / "hardware/XteinkDetect/test/host/stubs"),
                 "-I" + str(LIBS / "hardware/BoardConfig/include"),
                 str(HERE / "test_sleep_panel.cpp"), "-o", str(exe)],
                check=True,
            )
            print(f"== {label}", flush=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    sys.exit(main())
