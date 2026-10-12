# Testing FreeInk SDK

Run these commands from the repository root. Host tests use production code
with desktop adapters or hardware stubs; they do not require a device or PlatformIO.
You need a C/C++ compiler with C++17 support, Python 3, and a POSIX shell.

## UI, books, credentials, and input

```sh
sh libs/ui/FreeInkUI/test/host/run.sh
sh libs/book/FreeInkBook/test/host/run.sh
sh libs/book/ContentProtection/test/host/run.sh
sh libs/hardware/InputManager/test/host/run.sh
sh libs/hardware/InputManager/test/host/run_metalio.sh
sh libs/hardware/HapticManager/test/host/run.sh
sh libs/hardware/BatteryMonitor/test/host/run.sh
```

The BatteryMonitor suite covers the analog charge-status ladder decoder
(`ChargeLadderDecoder.h`): window classification, debounce, the no-battery
blink, the settling tail and the unknown-band hold.

The book suite also needs `zip`. It uses `sips` or ImageMagick's `convert` for
JPEG fixtures and optionally Pillow for progressive JPEG fixtures; the related
checks skip when those converters are unavailable. It builds layout tests for
the default, SMALL, and LARGE memory profiles. Build outputs go under the system
temporary directory.

## Battery gauge

```sh
sh libs/hardware/BatteryMonitor/test/host/run.sh
```

Compiles `BatteryMonitor` and the real X3 board profile against a model BQ27220
that holds the Design Capacity load to TRM SLUUBD4A 6.1: key timing, CONFIG
UPDATE, checksummed Data Memory writes, and the X3's block re-select. Every
refused I2C transaction must still leave the gauge out of CONFIG UPDATE and
sealed, and the next start must finish the load.

The stubs next to it stand in for the app's `Logging.h` and for the FreeRTOS
mutex behind `SharedAdc.h`, which `BatteryMonitor.cpp` includes.

## Real-time clock

```sh
python3 libs/hardware/Rtc/test/host/run.py
```

Builds `Rtc.cpp` with a recording I2C bus, once with `FREEINK_CAP_RTC` on and once off (it needs `CXX` or `c++`
and a compiler that takes `-fsanitize`, or a shim that ignores it). The RTC-on build checks the RX8130's time
reads and writes (the valid-time flag, the stop bit, the cleared voltage-low flag) and its countdown timer
(`armWakeTimer()`, `cancelWakeTimer()`, `readWakeTimer()`: the register values, every refused write, a counter or
enable that does not stick, another interrupt owner, an unset clock), plus the PaperMono board's PMIC wake
registers (`papermono::setRtcWake()`, `ensureBooted()`'s snapshot and `requestShutdown()`'s abort). Nothing in
this SDK calls the timer functions: they are API for a consumer. The stubs in `test/host/stubs` stand in for
`BoardConfig.h` (only the sensor fields the RTC reads), `soc/soc_caps.h`, `driver/gpio.h`,
`esp32-hal-periman.h` and `esp_rom_sys.h` (what `SharedI2c.h` includes); the `Wire.h` is BatteryMonitor's.

## Silkscreen battery log

```sh
c++ -std=c++17 -fsyntax-only -Wall -Wextra -DSILKSCREEN_BATLOG=1 \
  -Ilibs/hardware/PowerManager/include \
  libs/hardware/PowerManager/test/host/test_batterylog_core.cpp
```

The log exists only in the `silkscreen-batlog` image (`SILKSCREEN_BATLOG=1`). This test is all `static_assert`s, so
compiling it is running it: the CRC-16, the CSV row formatter against the design's example row, the burst
arithmetic, and the record's state machine, including a silent restart, the SD back-off and the lost-row cases.
`libs/hardware/PowerManager/test/host/README.md` has the list, the state table and the negative control. The
hardware layer (`BatteryLog.cpp`) is only compiled, not run.

## Driver-free panel sleep

```sh
python3 libs/hardware/PowerManager/test/host/run_sleep_panel.py
```

`PowerManager.cpp` needs the ESP-IDF sleep and ADC APIs, so this cuts
`sleepPanelWithoutDriver()` and its two helpers out of it and runs them over a
recording GPIO model, for the 4.26" Silkscreen board and for its 3.68" variant:
the command bytes, the BUSY pull, the reset's settle time (10 ms on the 4.26"
board, 60 ms for the variant's UC8279d) and the BUSY_N bound. The README next to
it has the details and the negative control.

## Fonts

```sh
sh libs/font/FreeInkFont/test/host/run.sh
sh libs/font/FreeInkFont/test/host/run_ligature.sh
```

Covers `FtFont::RenderOptions` (every `HintingMode` plus monochrome) against a
real bundled font, plus a minimal build with none of the three
`FREEINK_FONT_ENABLE_*` flags set to confirm existing consumers see no change
in output and unsupported requests are reported rather than silently degraded.

Covers `FtFont::ligature()`/`ligatureGlyphId()` (GSUB-based) against a real
bundled font: exact-value assertions for the five standard Latin ligatures,
the codepoint-vs-glyph-ID contract boundary, deinit()/re-init cache
invalidation, and a null-pointer safety check on `Gsub::LigatureGlyphId`.

## BLE HID host

```sh
sh libs/network/BleKeyboardHost/test/host/run.sh
```

Compiles `BleKeyboardHost` with `FREEINK_CAP_BLE_HID_HOST=1` against a fake
NimBLE and FreeRTOS layer, with the connection task on a real thread. Covers
report decode into key events, auto-reconnect, scanning and teardown order.
The fake has no radio, so pairing, link timing and memory use still need a
device.

## Display drivers

```sh
python3 libs/display/FreeInkDisplay/test/host/run_pro.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8279.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8279c_a4.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8253_power.py
python3 libs/display/FreeInkDisplay/test/host/run_x3_diagnostics.py
```

These compile production drivers against recording buses. The Pro suite also
compiles the facade in single- and dual-buffer modes. The checks cover transfer
sequences, grayscale uploads, refresh lifecycle, and power-state behavior for
the drivers included in each harness.

`run_x3_diagnostics.py` compiles the X3 UC8279d driver with
`FREEINK_DIAGNOSTICS=1` and checks the refreshes it reports to
`EpdBus::noteRefresh()` (mode run, CDI byte, blocking or deferred, turn-off).

`run_uc8279.py` and `run_uc8253_power.py` record the bus through
`epdbus_txn_model.h`, a model of `EpdBus`'s transaction contract. `cmd()`, `data()`,
`cmdData()` and the plane streamers each open a Transaction, and `beginTxn()` holds one:
a bus call made while one is alive blocks its task for ever in arduino-esp32's
non-recursive `SPI.beginTransaction()` mutex (no panic, no watchdog, a frozen screen).
The tests fail on such a call, and check a strip upload's order: the window, the plane's
RAM command, the rows as one burst, then PTOUT after the burst has ended. Both drivers
end the strip's transaction before PTOUT, so every configuration passes: the UC8279d's
`txn.end()` is upstream's fix (Free-Ink PR #139, `c2d711a`), built ungated for the C3 X3
and the Silkscreen 3.68" variant alike. A copy of the driver can be run instead with
`UC8279_SOURCE` or `UC8253_SOURCE`, each the path of the copy (the controls: upstream's
UC8279d source before that fix, written with
`git show c2d711a^:libs/display/FreeInkDisplay/src/driver/Uc8279Driver.cpp > file`, and
the UC8253 driver without its `txn.end()`; both fail on `EpdBus::cmd(0x92) while a
beginTxn() Transaction is alive`).

For the board-profile and display-probe regression test, follow the
[XteinkDetect host instructions](../libs/hardware/XteinkDetect/test/host/README.md),
which also cover the Silkscreen 3.68" variant (`test_silkscreen_368_policy.cpp`: a UC8279d
panel, fixed in its profile, with no bus traffic to identify it at any reset).

## Board profile rules

```sh
python3 libs/hardware/BoardConfig/test/host/run_profile_asserts.py
```

The Silkscreen profiles (the 4.26" board and the 3.68" variant,
`SILKSCREEN_PANEL_368`) are guarded by `static_assert`s and `#error`s in
`BoardConfig.h` that no test executes. This runs the compiler's syntax check on the
header (the PlatformIO xtensa g++, or `CXX`; C++20 with GNU extensions) against the
XteinkDetect host stubs, for each flag combination that must build and each that must
be refused, and says which expectation failed: a build that did not compile, one that
compiled and should not have, or a refusal by a different assert than the one meant
for it. A few cases edit a scratch copy of the header to prove an assert is live (the
profile a build boots with, the probe policy). A new profile rule gets a case here.

## Validation limits

Host suites cover only the code and configurations each harness compiles. Run
consumer firmware builds for affected device and capability combinations, then
validate relevant refresh quality, peripheral behavior, and sleep/wake on hardware.
This SDK supplies sample PlatformIO configurations; firmware entry points belong
to the consumer project.

When checking dead code, distinguish private implementation details from public
SDK APIs. A public method, board profile, compatibility wrapper, or capability-gated
driver can have no in-repository callers and still be used by downstream firmware.
Check references across all configurations before removing internal symbols, and
exclude vendored dependencies and generated assets from routine source cleanup.
