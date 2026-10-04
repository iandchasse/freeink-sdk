#if SILKSCREEN_BATLOG
#pragma once

// FreeInk SDK - the Silkscreen battery sleep/wake log, built only into the silkscreen-batlog image.
//
// One CSV row per completed sleep/wake cycle: a loaded ADC burst at the head of PowerManager::deepSleep(), a partly
// settled one right after the wake decision, the early re-sleeps in between as counters, the session ledger and the
// flags that explain them. The state lives in one RTC_NOINIT record (it survives deep sleep, a silent restart, a panic
// and a watchdog; a power loss or a new image loses it, and the next row says so). The rules are in BatteryLogCore.h;
// this layer owns the memory, the ADC and the clocks. The SDK formats a row into the caller's buffer and never opens a
// file: the app writes it, once per verified wake, after the first paint, and reports the outcome with flushed().
//
// Nothing here adds a pin call, pull, hold, timer, task or wake source, and nothing waits without a bound between the
// sleep decision and the deep-sleep entry: the ADC lock is taken with a 20 ms bound, a miss is skipped and flagged.
// Everything runs on the Arduino loop task except noteRefresh(), noteUiSeed() and noteClockSynced(), which may be called
// from any task.

#include <BoardConfig.h>

#if !FREEINK_DEVICE_SILKSCREEN || !FREEINK_CAP_POWER_POLICY
#error "SILKSCREEN_BATLOG is for the Silkscreen device only (FREEINK_DEVICE_SILKSCREEN, FREEINK_CAP_POWER_POLICY)"
#endif

#include <cstddef>
#include <cstdint>

#include "BatteryLogCore.h"

namespace freeink {
namespace batterylog {

using core::EarlyKind;
using core::Path;
constexpr uint32_t FILE_CAP_BYTES = core::FILE_CAP_BYTES;
constexpr size_t ROW_BUFFER_BYTES = core::ROW_BUFFER_BYTES;
constexpr uint32_t SLOW_FLUSH_MS = core::SLOW_FLUSH_MS;  // a flush over this is a bad attempt for the SD back-off

// What the app stages at the trigger, in RAM only.
struct Intent {
  Path path = Path::Manual;     // the default end-of-session path; a tagPath() call before beginEntry() overrides it
  uint8_t usb = core::ABSENT8;  // USB present: 0, 1
  char chg = 0;                 // charge state: '?' unknown, 'b' battery, 'c' charging, 'f' full, 'x' idle, 'n' no cell
  uint16_t uiTenths = 0;        // the battery value the last render showed, in 0.1 %; 0 when it has none yet
  uint32_t idleMs = 0;          // time since the last user input
  bool lightOn = false;
  bool radioOn = false;
  bool settingsLoaded = false;  // false on the SD-error boot: cfg and tmoMin are then left out of the row
  uint8_t cfg = 0;              // bit 0 Restore Light on Wake, 1 click-wake rule, 2 touch, 3 front light, 4 Fading Fix
  uint8_t tmoMin = 0;           // the sleep timeout setting, in minutes
};

enum class Flag : uint8_t { TouchNotAsleep, SdaStuck };

struct WakeInfo {
  bool holdVerified = false;  // the power key was held through the wake verify
  bool clickWake = false;     // accepted with no hold, under the click-wake rule
};

struct TickIn {
  bool lit = false;
  uint8_t litPct = 0;  // front light brightness, percent
  bool radioOn = false;
  bool fullClock = false;
  bool usb = false;
};

// What the writer knows when it flushes.
struct FlushCtx {
  uint8_t usb = core::ABSENT8;  // USB present now: 0, 1
  char chg = 0;                 // charge state now, as in Intent
  bool chargeUndecoded = false;
  bool fileCapReached = false;  // this row brings the file to FILE_CAP_BYTES: it is the last one written
};

struct Pending {
  bool ready = false;   // a row waits for the SD card
  bool orphan = false;  // ... and it is a k = B row
};

struct Status {
  bool valid = false;  // the record passed its checks at the last change
  uint8_t state = 0;   // core::State
  uint32_t nextRow = 0;
  uint16_t drop = 0;
  uint8_t sdBad = 0;
  uint8_t sdSkip = 0;
};

// S0, at the top of the app's sleep entry: stage the intent in RAM. Does nothing else.
void beginEntry(const Intent& intent);
// Sets the path for the next beginEntry(), for the call sites that know why the sleep was ordered.
void tagPath(Path path);
// RAM notes for the staged entry, from the HAL: the touch controller did not accept hibernate, the I2C bus is stuck.
void noteFlag(Flag flag);
// E, the head of PowerManager::deepSleep(): the entry burst, the stamp, the record. Returns at a flag compare when
// no entry is staged. `releaseArmed`: the sleep wakes on the power key's release, so the key is still held.
void tail(bool releaseArmed);
// An early re-sleep after the wake decision: folds into the pending entry. `awakeMs` is millis() at the decision.
void early(EarlyKind kind, uint32_t awakeMs);
// W, after the wake decision and before the touch probe: classifies the reset and, on a deep-sleep wake, takes the
// wake burst.
void onWake(const WakeInfo& info);
// The first battery percentage after the wake, in 0.1 %, from any task; merged into the row once.
void noteUiSeed(uint16_t tenths);
// The system clock was set by NTP, from any task.
void noteClockSynced();
// Every loop pass: the session ledger, and the notes the other tasks left.
void tick(const TickIn& in);
// A loop pass with user input; a display refresh (any task).
void noteInput();
void noteRefresh();
// Before a silent restart: keep the running session's accumulators across it.
void carry();

// A row waits, and whether it may be written at this wake: the SD back-off skips the flush on most wakes after two
// bad attempts in a row. These read RAM only.
Pending pending();
bool sdBackoff();
uint8_t sdBadCount();

// The pending row as written to the file: a LF, 53 fields, a CRC. Returns its length, or 0 when nothing is pending or
// `cap` (at least ROW_BUFFER_BYTES) is too small.
size_t formatRow(char* buf, size_t cap, const FlushCtx& ctx);
// The header line (no trailing LF) and the "# fw" comment line (after a LF), for a new file and before an orphan row.
size_t formatHeader(char* buf, size_t cap);
size_t formatFwLine(char* buf, size_t cap, const char* version, uint16_t bootWaitMs, bool serialLog);
// The writer's result. `ms` is the whole open-write-close sequence.
void flushed(bool ok, uint32_t ms);

// The nine hex characters of this image's ELF hash: the build id the record is bound to.
const char* buildId();
Status status();

}  // namespace batterylog
}  // namespace freeink

#endif  // SILKSCREEN_BATLOG
