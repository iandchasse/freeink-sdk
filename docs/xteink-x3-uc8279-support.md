# Xteink X3 — UC8279d controller variant

Newer X3 production units (Xteink heads-up, July 2026) ship the same ESP32-C3
board and 792×528 glass with a **UC8279d** panel controller in place of the
UC8253. Everything else — pinout, ADC ladder input, BQ27220/DS3231/QMI8658
peripherals, SD wiring — is unchanged. The variant has its own sibling profile,
`BoardConfig::XTEINK_X3_UC8279` (`Board::XteinkX3Uc8279`).

Build: nothing new — `-DFREEINK_DEVICE_X3=1` links both X3 drivers
(`FREEINK_DRIVER_UC8253_X3` and `FREEINK_DRIVER_UC8279`); which one runs is
decided at boot.

The UC8279d driver also drives the X3's panel on an ESP32-S3: Silkscreen's 3.68"
variant (`-DFREEINK_DEVICE_SILKSCREEN=1 -DSILKSCREEN_PANEL_368=1`) links it, and
only it (no UC8253 driver), for the profile `SILKSCREEN_368` (792×528, no touch),
on Silkscreen's own pins rather than the X3's. **That variant supports the
UC8279d and nothing else** (owner decision: decided 2026-10-11, made
2026-10-12): its controller is fixed in the profile, so it never runs the stock
probe below, reads no VER, and keeps no verdict in RTC memory; a cold boot and a
deep-sleep wake make no bus traffic for identification. It applies the same
driver recipe, waveforms and refresh policy; the differences are the BUSY_N
pull-up and the driver-free panel sleep (behind the Silkscreen probe policy)
and, under `FREEINK_SILKSCREEN_368`, a `txn.end()` before PARTIAL_OUT in the
strip upload (`4988be4`; upstream's copy holds a transaction across
`bus.cmd()`). See the README's per-batch controller section.

**Observed signature of the module that set this decision.** The Xteink 3.68"
panel marked `SEO368NQW25` (flex marking), on a Silkscreen board with the
stock X3 probe, read `VER=00 0F 00`: the reserved `0x00`, a CHIP_VER of `0x0F`,
and a LUT_VER byte of `0x00`, with no BUSY timeout. That is a driven UltraChip
answer, and it is neither the `0x66` of the X3's own UC8279d module (the id the
stock selection above keys on) nor the `0xFF` of a UC8253. The probe's unknown-id
default ran the UC8253 recipe (`Wait complete: X3_PON (41 ms)`) and the glass stayed
blank; with the UC8279d driver forced, the panel displayed a picture. One module,
one bench session: nothing here says what other `SEO368NQW25` units, or other
batches of the 3.68" panel, read. The variant's build does not read VER.

**What has run on a board (as of 2026-10-12).** One module (`SEO368NQW25`) has
run, in the owner's first session (2026-10-11, a CrossPoint build): the stock X3
probe read `VER=00 0F 00`, the UC8253 default recipe left it blank (`Wait
complete: X3_PON (41 ms)`), and with the UC8279d driver forced by a local build
flag it displayed a picture. That forced build then froze (no panic, no
watchdog) on its first grayscale image page. The code reads as a bus transaction
held across a bus command (a deadlock on arduino-esp32's non-recursive SPI
mutex): that cause is inferred from the code, not from a backtrace. SDK
`4988be4` ends the transaction first and the fix is visible in the images'
disassembly; it has not yet been seen on glass. The owner also reported a quiet,
harsh buzz at every display refresh, from the e-paper boost stage; it is
unexplained, and the boost inductor `L1` (47 µH, sized to the SSD1677 reference
and unverified for this panel: the Silkscreen board's HARDWARE.md §6.2) and the
4.7 µF / 50 V X5R MLCCs `C11` and `C13`-`C17` are the usual singers. Nothing
else has run on a board, and CrossInk's variant images have never been flashed.

The detection section reflects the stock V6.3.15 protocol. The driver uses
recovered external waveforms and register initialization; the initial OTP-only
proposal is no longer the implementation.

## Runtime detection

After the X3 I2C fingerprint, `detectX3DisplayController()` uses the fixed X3
pins (SCLK 8 / SDA 10 / CS 21 / DC 4 / RST 5 / BUSY 6), even while the active
profile still names X4. `detectXteinkDisplayController()` uses the same X3
protocol when either X3 profile is already active.

The probe follows stock V6.3.15: RESET high 10 ms, low 50 ms, high 50 ms;
wait up to 300 ms for BUSY high; release SDA to input and read three bytes of
**VER (0x70)**, sampling after SCLK rises. A BUSY timeout is recorded but does
not suppress the read, as in stock firmware. The third byte selects the panel:
`0x66` confirms UC8279, `0xFF` assumes UC8253, other IDs are inconclusive and
leave the default UC8253 selected. FLG and MTP are not read or required.

The legacy five-byte VER out-param contains the three read bytes and two zeros;
the FLG out-param is zero (not sampled). Diagnostics expose `verBytesRead=3`
and `busyTimedOut`. Boot logs show `[XTDET] X3 stock probe VER=...` with the
selection and timeout status. X4-family probes retain their existing protocol.

Hardware validation remains necessary: collect these logs on affected units
and check cold boot and sleep/wake. This change updates identification only;
display refresh reset timing, SPI frequency, and waveforms are separate tests.

## Driver — `Uc8279Driver`

KW mode (`PSR KW/R=1`): 1-bpp, DTM1 = OLD plane, DTM2 = NEW plane,
differential refresh — the same paradigm as the UC8253 X3 driver, and a
near-identical command set (PSR/PON/POF, DTM1 `0x10`, DSP `0x11`, DRF `0x12`,
DTM2 `0x13`, CDI `0x50`, TRES `0x61`, DSLP `0x07`+`0xA5`).

The driver programs the panel explicitly with external LUTs (`PSR REG=1`).
The module's blank MTP cannot supply the factory defaults assumed by the original
OTP-only proposal. The recovered initialization script configures power,
booster, PLL, and a 792×528 partial window; RAM writes and refreshes use that
window to avoid the controller's native 800×600 stride.

- B/W refreshes use recovered full (GC) and fast (DU) banks.
- Overlay grayscale uses XTF_AA waveforms with a separate B/W conditioning pass.
- Absolute four-tone images use the XTH4 bank. Both grayscale modes support
  complete-plane and strip uploads; neither advertises asynchronous grayscale
  conditioning or staging while busy.
- Ordinary B/W refreshes support the split `displayStart()` / `displayFinish()`
  interface.

See the [driver implementation](../libs/display/FreeInkDisplay/src/driver/Uc8279Driver.cpp),
[waveform tables](../libs/display/FreeInkDisplay/src/lut/Uc8279X3Luts.h), and
[grayscale API guide](grayscale-capabilities.md). Run the
[display host tests](testing.md#display-drivers) for protocol regression coverage;
physical tone separation and refresh quality still require hardware checks.

## Useful UC8279 features not yet wired

- **AUTO (0x17)**: `PON→DRF→POF(→DSLP)` as one command — could shave host
  round-trips on sleepy ESL-style updates.
- **PBC (0x44)**: panel-break check via the CHKGI/CHKGO wire loop, if the
  module bonds it.
- **CRC (0x72)**: MTP integrity check over `0x000–0xFFF`.
- On-chip temperature readback (**TSC 0x40**) if the consumer ever wants it.
