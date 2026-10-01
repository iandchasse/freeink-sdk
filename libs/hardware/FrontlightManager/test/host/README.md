# Front-light load record test

`test_load_policy.cpp` runs the rules in `FrontlightLoad.h` that
`FrontlightManager` applies around its switch-on load probe
(`FrontlightConfig::loadProbeWindowMs`, `FREEINK_CAP_FRONTLIGHT_PROBE`): which string a
switch-on lights, when the other string is tried after a trip, when a string is
recorded missing, when the record says no light (and Restore Light on Wake stays
dark), what an in-use trip changes, and the RTC-memory form with its checks.

From the SDK root:

```sh
mkdir -p .cache/frontlight-load-build
c++ -std=c++17 -Wall -Wextra -Werror \
  -Ilibs/hardware/FrontlightManager/include \
  libs/hardware/FrontlightManager/test/host/test_load_policy.cpp \
  -o .cache/frontlight-load-build/test_load_policy
.cache/frontlight-load-build/test_load_policy
```

The header has no hardware dependency, so no stubs are needed. The probe itself
(the readings of `LED_MONIT` on the monitor timer after the enable pulse, the rail
wait before the colour select moves) runs only on a board: the Silkscreen test firmware's T9 and
T10 report it.
