# Display probe regression test

From the SDK root:

```sh
mkdir -p .cache/x3-detection-build
c++ -std=c++17 -Wall -Wextra -Werror \
  -DFREEINK_DEVICE_X3=1 -DFREEINK_DEVICE_X4=1 \
  -Ilibs/hardware/XteinkDetect/test/host/stubs \
  -Ilibs/hardware/BoardConfig/include \
  -Ilibs/hardware/XteinkDetect/include \
  libs/hardware/XteinkDetect/test/host/test_display_probe.cpp \
  libs/hardware/XteinkDetect/src/XteinkDetect.cpp \
  -o .cache/x3-detection-build/test_display_probe
.cache/x3-detection-build/test_display_probe
```

The test compiles the production detector and real board profiles against GPIO,
time, I2C, and NVS stubs. The simulated display rejects reads on the wrong clock
phase or with the wrong SDA mode and records command bytes, reset timestamps,
read lengths, and pin release. It covers stock X3 ID selection, absent FLG/MTP,
delayed/stuck BUSY, unknown IDs, direct X3 probing with the X4 boot profile,
controller promotion, and unchanged X4 VER/FLG/MTP transactions.

This verifies host protocol behavior, not the electrical behavior of a physical
display. Test affected hardware with the new `[XTDET] X3 stock probe` log,
including cold boots and sleep/wake.

## Silkscreen probe policy

`test_silkscreen_policy.cpp` runs the same detector on the Silkscreen profile,
whose `DisplayProbeConfig` takes every UltraChip answer as a UC8279 and keeps a
conclusive verdict in RTC memory for a deep-sleep wake. Build it twice: as is,
and with the build override that skips the probe.

```sh
mkdir -p .cache/silkscreen-probe-build
for forced in "" "-DSILKSCREEN_DISPLAY_CONTROLLER=uc8279"; do
  c++ -std=c++17 -Wall -Wextra -Werror -DFREEINK_DEVICE_SILKSCREEN=1 $forced \
    -Ilibs/hardware/XteinkDetect/test/host/stubs \
    -Ilibs/hardware/BoardConfig/include \
    -Ilibs/hardware/XteinkDetect/include \
    libs/hardware/XteinkDetect/test/host/test_silkscreen_policy.cpp \
    libs/hardware/XteinkDetect/src/XteinkDetect.cpp \
    -o .cache/silkscreen-probe-build/test_silkscreen_policy &&
  .cache/silkscreen-probe-build/test_silkscreen_policy
done
```

It covers a UC8279 found on a cold boot and kept across a deep sleep with the bus
silent, the vendor 50 ms reset on the screening pass, a dropped verdict that the
next wake probes again, a floating bus (SSD1677) after every other reset reason,
LUT_VER values that must not select UC8179 (not linked for Silkscreen), a
UC8279d that drives only a stable FLG (VER floating or all zero, blank MTP), an
inconclusive probe (FLG on one pass only) that is not kept, and a forced build
that never touches the bus.

## Silkscreen 3.68" variant (`SILKSCREEN_PANEL_368`)

`test_silkscreen_368_policy.cpp` runs the production detector on the Silkscreen board
with the Xteink X3's panel on J2. The panel is the X3's UC8279d and nothing else (the
owner's SEO368NQW25 module, whose stock VER read answers `00 0F 00`, runs on that driver):
the profile carries the controller, and its probe policy is forced with no verdict cache,
so nothing is identified at a boot or a wake. The test counts every call that could be
identification traffic (pin modes, writes, reads, sleep-hold releases, waits) and requires
none of them at a cold boot, a deep-sleep wake and every other reset reason. Build it
twice: as is, and with the override that the variant accepts as a no-op.

```sh
mkdir -p .cache/silkscreen-368-probe-build
for forced in "" "-DSILKSCREEN_DISPLAY_CONTROLLER=uc8279"; do
  c++ -std=c++17 -Wall -Wextra -Werror -DFREEINK_DEVICE_SILKSCREEN=1 -DSILKSCREEN_PANEL_368=1 \
    -DFREEINK_CAP_TOUCH=0 $forced \
    -Ilibs/hardware/XteinkDetect/test/host/stubs \
    -Ilibs/hardware/BoardConfig/include \
    -Ilibs/hardware/XteinkDetect/include \
    libs/hardware/XteinkDetect/test/host/test_silkscreen_368_policy.cpp \
    libs/hardware/XteinkDetect/src/XteinkDetect.cpp \
    -o .cache/silkscreen-368-probe-build/test_silkscreen_368_policy &&
  .cache/silkscreen-368-probe-build/test_silkscreen_368_policy
done
```

It covers the profile the build selects (`DEFAULT_DEVICE` and `ACTIVE`: 792x528, UC8279, no
touch and the keys kept, IO11 parked HIGH, the X3's 10 MHz clock and the BUSY_N pull-up),
the forced policy without a cache, `applyXteinkDisplayController()` at every reset reason
(nothing promoted, source `Forced`, no probe record, nothing for `forgetCachedDisplayController()`
to drop), and the C3-only fingerprints, inert on an S3. `=uc8253` and `=ssd1677` are refused
by `BoardConfig.h`, which `libs/hardware/BoardConfig/test/host/run_profile_asserts.py` checks.
