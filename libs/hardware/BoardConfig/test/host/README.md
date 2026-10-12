# Boot-time hardware check test

`test_probe_checks.cpp` runs the rules in `ProbeChecks.h` behind the boot-time
checks a board can opt into:

- the key-ladder dual-pull test (`PowerConfig::ladderPresenceProbe`,
  `InputManager::begin()`): every fitted ladder stays on, released or with any
  key held, for internal pulls from 20 to 100 kΩ, and only a ladder with no
  pull-up is switched off;
- the FocalTech chip ids (`TouchConfig::requireFocalTechId`), against an
  all-zero window, a floating bus and common J6 devices at 0x38;
- the DS3231 register pattern (`SensorsConfig::rtcIdentityCheck`, `Rtc`), on a
  power-on and a running DS3231 and on other parts found at 0x68, one fixed bit
  at a time;
- the power pin's idle level (`PowerConfig::powerPinIdleCheck`,
  `PowerManager::checkPowerPinIdle()`), including Silkscreen's R36 + R72 misfit.

From the SDK root:

```sh
mkdir -p .cache/probe-checks-build
c++ -std=c++17 -Wall -Wextra -Werror \
  -Ilibs/hardware/BoardConfig/include \
  libs/hardware/BoardConfig/test/host/test_probe_checks.cpp \
  -o .cache/probe-checks-build/test_probe_checks
.cache/probe-checks-build/test_probe_checks
```

The header has no hardware dependency, so no stubs are needed. The readings
themselves (the ADC with each pull on, the touch and RTC register reads, IO18 on
ADC2) happen only on a board: the Silkscreen test firmware's P1, P2, T3, T11 and
T13 report them.

# Profile rules

The Silkscreen profiles are guarded by `static_assert`s and `#error`s in
`BoardConfig.h`, which nothing executes. `run_profile_asserts.py` compiles the
header (syntax only, `-std=gnu++2a`) against the XteinkDetect host stubs for each
flag combination that must build and each that must be refused, and fails with the
expectation that did not hold. It runs the PlatformIO xtensa g++ or `CXX`:

```sh
python3 libs/hardware/BoardConfig/test/host/run_profile_asserts.py -v
```

- **Must build:** the 4.26" board (default, forced `ssd1677` or `uc8279`, alternate
  touch jumpers, `SILKSCREEN_PANEL_368=0`), the 3.68" variant (default, touch
  written out as 0, `uc8279` or `UC8279` accepted as a no-op, diagnostics), and the
  X3, X4, X3+X4, X4 Pro, X4 Classic and de-link headers, with no warning.
- **Must be refused, by the assert meant for it:** the variant with touch, with
  `ssd1677` (its driver is not linked), `uc8253` (no Silkscreen build links the
  UC8253 driver) or `uc8179` (not declared), with the alternate jumpers, beside an
  SSD1677 device or without the probe policy; the macro at 2 or without the
  Silkscreen device; the 4.26" board with `uc8253`, without the touch code, without
  the probe policy; a Silkscreen build without probe identity, power policy or ADC
  ladders, or with a C3 device.
- **Asserts proven live (a scratch copy of the header is edited):** the variant's
  `DEFAULT_DEVICE` set to the 4.26" profile and the reverse, its controller made a
  UC8253, its probe policy left probing (`forced` off) or caching a verdict, an
  inherited probe field changed in `derive()`, `x4ClassPanel` turned off, and a
  field appended to `DisplayProbeConfig` that SILKSCREEN sets, which the variant
  must inherit.
