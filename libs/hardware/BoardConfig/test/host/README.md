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
