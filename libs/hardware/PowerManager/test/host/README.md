# Battery log core test

`test_batterylog_core.cpp` checks the pure rules of the Silkscreen battery sleep/wake log (`BatteryLogCore.h`): the
CRC-16, the 53-column CSV and its row formatter, the ADC burst arithmetic, the sleep and awake intervals, the session
ledger and the RTC record's state machine. Every check is a `static_assert`, so the test needs no host compiler to
run, only one that can parse it. The log exists only in the `silkscreen-batlog` image: the whole file sits inside
`#if SILKSCREEN_BATLOG`, which only that env's build flags define, and without the macro it compiles to nothing.

From the SDK root, with the xtensa g++ of the PlatformIO toolchain or any C++17 compiler:

```sh
~/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++ -std=gnu++17 -fsyntax-only -Wall -Wextra \
  -DSILKSCREEN_BATLOG=1 -Ilibs/hardware/PowerManager/include \
  libs/hardware/PowerManager/test/host/test_batterylog_core.cpp
```

A story function returns 0 when its checks hold, or the source line of the first one that does not. GCC prints it in
the note under the failed assertion: `the comparison reduces to '(805 == 0)'` means line 805. The header has no
hardware dependency, so no stubs and no include paths beyond `PowerManager/include` are needed. The hardware layer
(`BatteryLog.cpp`: the RTC memory, the ADC burst under a bounded lock, the clocks) is not covered here; it is checked
by compiling it with the SDK's include paths, and by the bench tests of the design.

## What it covers

- CRC-16/CCITT-FALSE: the check value 0x29B1 of "123456789", the empty string, one byte, a continued pass.
- The integer square root, and the burst mean, spread and range, including zero conversions (the ADC's silent
  failure), the clamps and the extremes of the 64-bit steps.
- The interval arithmetic with a missing stamp, a counter that went backwards and an interval that does not fit 32 bits.
- The session ledger: ticks, counts, compaction to the record's form, saturation.
- The header text (53 columns, 272 bytes, no trailing LF), the `# fw` line, and the row bound: a row with every field
  at its widest fits the 384-byte writer buffer, and the column table is tight to six characters of it.
- The design's example row (section 4.4), built through the state machine from the 64 conversions behind it: exact text,
  the CRC `c59d`, and the flush. The row has 205 bytes and 53 fields. Its raw sums agree with its millivolt means at the
  calibrated 0.8693 mV per count (2299.0 and 2304.4 counts, 1998.5 and 2003.25 mV at the pin), and its spreads (5.2 and
  4.7 mV, ranges 24 and 22 mV at P+) are what 64 conversions of 2.6 and 2.35 mV sigma give. The conversions, their
  sums and the row were produced by an independent model (Python, `binascii.crc_hqx`) that the formatter agrees with.
- Every row of the state table below, an entry staged on each record state, the full story (entry, ghost, ghost,
  wake, flush), a Ready row followed by a silent restart, a restart with nothing carried, a panic and a watchdog.
- `utc` empty until the clock-synced flag, and below the 2025-01-01 floor even with it.
- The lock-timeout, no-pin, no-valid, staged, charge-ladder and file-cap flag bits, with their values.
- An unflushed row followed by an entry, a failed write, a slow one, and the SD back-off: bad, bad, 63 skipped wakes,
  a good probe, a bad probe, a slow probe, a restart inside a skipped wake.
- A crash count (`ab`, `ab_r`) that stays through a write that failed and through a row dropped by the next entry,
  and goes only with a row that reached the card; the `w_ui` latch of the row before an orphan row, which a flush
  leaves set, does not shut the orphan's seed out.
- A negative control (below).

Mutating the core (the skip count, the CRC polynomial, the drop on a lost row, the boots count, the utc rule, what
makes a flush good, the spread's divisor, what a demote clears, what a flush clears, when a sleep interval is printed)
makes this test fail; each of those was tried.

## The state table

The record is Lost (fresh, or it failed its magic, version, build id and CRC-32), Idle (no cycle pending), Entry
(asleep, or in a chain of early re-sleeps) or Ready (a row waits for the SD card; k = S for a cycle, k = B for an
orphan boot with no recorded entry). The log has no setting, so there is no Disarmed or Unknown-mode state.

| Event | From | To | What it does |
|---|---|---|---|
| `tail` | any | Entry | A Ready row is counted lost (`drop`, its number consumed). Stamps, takes the staged intent and the ended session's ledger, clears the early and wake halves. `ab`/`abR` are kept |
| `early` | Entry | Entry | `g_n` (ghost) or `g_o` (cutoff) up, `g_ms` summed |
| `early` | Ready (S) | Entry | Demote: the wake burst becomes `g_mv`/`g_t`, the wake half is cleared, then as above |
| `early` | Ready (B), Idle, Lost | same | Nothing |
| `onWake` | Entry | Ready (S) | Deep-sleep reset: wake burst, sleep interval, new session. Other reset: an aborted sleep or a crash inside it, no burst, no interval, `fl_w` bit 2 |
| `onWake` | Idle, Ready, reset other than deep sleep | same | A restart of the running session: the wake half, `sleep_ms` and `w_ui` stay, `boots` counts, the carried ledger comes back (partial when there is none or the reset was abnormal) |
| `onWake` | Ready, deep-sleep reset | Ready (B) | The old row is counted lost, the wake is an orphan |
| `onWake` | Idle, deep-sleep reset | Ready (B) | A sleep nothing recorded: an orphan |
| `onWake` | Lost, any | Ready (B) | Row 1 (or the number the record had); a restart reason makes the session partial |
| `flushed` | Ready | Idle | Row number consumed; `drop` and `ab`/`ab_r` reset on a write that reached the card; the back-off counts bad attempts |
| `mergeUiSeed`, `noteClockSynced`, `carryLedger` | | | `w_ui` once per row and only while a row waits; the synced flag; the running ledger into the record |

## Where the core departs from the design text

The battery log design (sections 4.3-4.6) has a setting that arms and disarms the log; the build has none.
Besides dropping `Disarmed`, `onSettingsLoaded()`, `discard()` and the Unknown mode, it differs in these places, each
found while writing the tests:

- The running session needs its own carried ledger in the record. With one ledger group, a silent restart while a row is
  Ready would overwrite that row's ledger with the new session's. The record keeps both (`ended` and `carry`) and
  `carry.boots` is the running boot count, so panics keep counting boots too.
- `ab`/`abR` count from one flushed row to the next, so they survive an entry: a panic in the session reaches the
  next row, and a lost row's crashes are not lost with it.
- `sleep_ms` is computed from the two stamps when the row is formatted (17 digits fit any u64 microsecond count)
  instead of being stored in 32 bits, which would wrap after 49.7 days; a stamp of 0 means "not taken".
- An orphan row carries no `cfg`, `tmo` or ledger columns: the boot that makes it has not seen the settings.
- The SD back-off's skip is a record flag (`SKIP_NOW`) set by the wake that consumed one of the 63, so a restart inside
  a skipped wake stays skipped and exactly 63 wakes are skipped before the probe.
- `rf` stays empty until the first refresh is counted, so a build that never calls `noteRefresh()` shows no column
  of zeros.

## The negative control

The block at the end of the test, `#if 0 // NEGATIVE CONTROL`, holds one assertion that is wrong on purpose: the
example row with its last CRC digit changed. To see the test fail, compile a scratch copy with the block enabled:

```sh
sed 's|^#if 0  // NEGATIVE CONTROL|#if 1  // NEGATIVE CONTROL|' libs/hardware/PowerManager/test/host/test_batterylog_core.cpp > /tmp/neg.cpp
```

then compile `/tmp/neg.cpp` as above: it must stop with `static assertion failed: NEGATIVE CONTROL`, and nothing else.
Keep the block disabled in the file.

# Driver-free panel sleep

`PowerManager::sleepPanelWithoutDriver()` puts a panel on an always-on rail into deep sleep with no driver: a reset
pulse, a bounded wait for BUSY, then the controller's own command bit-banged. `PowerManager.cpp` cannot be compiled on
a host (it needs the ESP-IDF sleep, RTC and ADC APIs), so `run_sleep_panel.py` cuts the function and its two helpers
out of the production file, builds them with a recording GPIO model, and runs `test_sleep_panel.cpp` once for the
4.26" board and once for the 3.68" variant. It stops if the file no longer holds exactly that function. It needs a
C++17 compiler, `CXX` or `c++`:

```sh
python3 libs/hardware/PowerManager/test/host/run_sleep_panel.py
```

What it checks: the bytes (SSD1677 `0x10`/`0x03`, an UltraChip `0x07`/`0xA5`, the 3.68" variant's UC8253 included), the
pull on BUSY while it runs, that the 4.26" board settles 10 ms after the reset releases as it always did and leaves a
UC8253 alone (no driver is linked for one there), that the variant settles 60 ms (what `Uc8253X3Driver::begin()` and
`Uc8279Driver::begin()` give their controllers), and the 100 ms bound on a BUSY_N that stays asserted. `SLEEP_PANEL_SOURCE`
names another copy of `PowerManager.cpp` to cut from: with the file as it was before the 60 ms settle the variant run
fails, which is the negative control.
