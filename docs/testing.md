# Testing FreeInk SDK

Run these commands from the repository root. Host tests use production code
with desktop adapters or hardware stubs; they do not require a device or PlatformIO.
You need a C/C++ compiler with C++17 support, Python 3, and a POSIX shell.

## Serial transport

```sh
python3 libs/hardware/BoardConfig/test/host/run_serial.py
```

Checks that native USB Serial/JTAG output is serviced without a blocking flush,
and that UART, TinyUSB, Sticky's UART bridge, and simulator builds remain no-ops.
Device tests must also verify timely acknowledgements and intact transfer data.

## Paper Mono bring-up

```sh
python3 libs/hardware/BoardConfig/test/host/run_papermono_boot.py
```

Runs `PaperMonoBoard.h` against a model PMIC and IO expander. An expander that
misses its first probes after a PMIC restart must still come up within the boot
wait, whichever manager starts first. A failed setup must stay cached, so no rail
in use is switched off later, and the power button must keep working. A failed
expander read or mode write must not leave the touch controller in reset.

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

## Progressive JPEG

```sh
python3 libs/assets/ProgressiveJpeg/test/host/run.py
```

Needs Pillow, `cjpeg` and `djpeg`. Decodes generated progressive JPEGs (4:2:0,
4:2:2, 4:4:4, grayscale, restart intervals, odd sizes) and compares the luma
with `djpeg -grayscale`; also checks that baseline files are refused and that
a truncated stream decodes without failing. Set `PJ_EXTRA_JPEGS` to a
colon-separated list of files to check real-world images too.

## Battery gauge

```sh
sh libs/hardware/BatteryMonitor/test/host/run.sh
```

Compiles `BatteryMonitor` and the real X3 board profile against a model BQ27220
that holds the Design Capacity load to TRM SLUUBD4A 6.1: key timing, CONFIG
UPDATE, checksummed Data Memory writes, and the X3's block re-select. Every
refused I2C transaction must still leave the gauge out of CONFIG UPDATE and
sealed, and the next start must finish the load. It also compiles the EEGO A4
profile against a model CW2015: the battery-profile check and upload sequence,
the voltage and percentage conversion, and the ADC fallback when no gauge
answers.

The battery suite also covers Picco's SGM41562 status as a single register
snapshot, three-conversion ADC median, charging endpoint and completion state,
and the allocation-free `BatteryEstimate` presentation filter. The estimator
checks include unplug settling, low-voltage correction, failed samples, sleep
reconciliation and retained-state validation.
The flash-at-near-full regression covers a 4.340 V cell still reporting active
charging: its stock voltage estimate can reach 100% without inventing charger
completion. Lower voltage held on USB alone does not become full over time.
These are model checks; actual voltage calibration, charge termination and power
usage still need a Picco.

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

## Motion sensors

```sh
sh libs/hardware/Imu/test/host/run.sh
```

Compiles `Imu` with the Metalio EInk4 profile against a model SC7A20H (and
the QMI8658 and LSM6DS3 scaling), once more with `FREEINK_CAP_IMU=0`, and with
the Paper Mono profile against a model BMI270: `begin()` must only identify
the part, the first `wake()` or `read()` must stream the whole configuration
file through the INIT window in order with a longer bus timeout, a warm part
must skip the upload, and the board's axis flips apply. The model does not run
the configuration file, so the sensor's data path still needs a device.

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

## Frontlight driver

```sh
sh libs/hardware/FrontlightManager/test/host/run.sh
```

Compiles the Picco frontlight backend and real board profile against a register
model with a wake-settling interval. Covers the first nonzero brightness write,
off/on level restoration, warm/cool channels, slider updates without repeated
wake delays, and retry after a failed wake transaction. Sweeps brightness in both
directions at all 101 warmth settings, checking fine PWM steps, minimum and maximum
output, disabled channels at zero PWM, and restoration after off/on. Physical light
output and the actual controller settling time still require a device check.

## Display drivers

```sh
python3 libs/display/FreeInkDisplay/test/host/run_bus.py
python3 libs/display/FreeInkDisplay/test/host/run_pro.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8279.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8279c_a4.py
python3 libs/display/FreeInkDisplay/test/host/run_uc8253_power.py
python3 libs/display/FreeInkDisplay/test/host/run_x3_diagnostics.py
python3 libs/display/FreeInkDisplay/test/host/run_picco.py
```

These compile production drivers against recording buses. The Pro suite also
compiles the facade in single- and dual-buffer modes, checks every concrete driver
header against the shared interface, and verifies selected controller identity
independently of board metadata. The Picco suite builds the
SSD1677/SSD2677 drivers as a Picco and as a Sticky, including each board's
bit-banged panel probe. The checks cover transfer
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

## SecureNet bulk transfers

```sh
sh libs/network/SecureNet/test/host/run.sh
WOLFSSL_INTERNAL_SOURCE=/path/to/Arduino-wolfSSL/src/src/internal.c \
  python3 libs/network/SecureNet/test/host/test_record_reservation.py
```

The shell suite exercises HTTP control flow with scripted transports, including
reservation timing after the request, cancellation, and resumed transfers. The
Python harness compiles the production reservation/stop methods and the
consumer's pinned wolfSSL buffer functions with a bounded allocator, sanitizers,
and competing clients. It requires the consumer's verified input-retention and
empty-buffer-release patches. It covers both allocation failure before the fix
and retained medium/full records afterward; it does not perform TLS handshakes
or measure device heap. Consumers without `FREEINK_WOLFSSL_RETAIN_INPUT_BUFFER`
use ordinary on-demand record growth without proactive reservation. Reservations
use wolfSSL's own connection-owned buffer and allocator, with no global allocator
replacement or cross-client loan.

## Streaming JSON previews

```sh
sh libs/network/JsonSax/test/host/run.sh
```

Checks optional bounded string prefixes, unchanged default overflow handling,
continued parsing after long strings, and rejection of oversized keys under ASan/UBSan.
