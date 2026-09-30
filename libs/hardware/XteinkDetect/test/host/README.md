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
silent, a floating bus (SSD1677) after every other reset reason, LUT_VER values
that must not select UC8179 (not linked for Silkscreen), an inconclusive probe
that is not kept, and a forced build that never touches the bus.
