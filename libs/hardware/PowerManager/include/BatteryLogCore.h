#if SILKSCREEN_BATLOG
#pragma once

// FreeInk SDK - the pure rules of the Silkscreen battery sleep/wake log (the silkscreen-batlog build), free of
// Arduino and ESP-IDF so a host test can check them at compile time:
//   - CRC-16/CCITT-FALSE, the 53-column CSV table and the row formatter into a caller buffer;
//   - the ADC burst arithmetic (mean, spread, range) in integers, with an integer square root;
//   - the sleep and awake interval arithmetic, including an RTC counter that went backwards;
//   - the session ledger and the RTC record's state machine (tail, early, onWake, flushed ...).
// BatteryLog.cpp owns the RTC memory, the ADC and the clocks and feeds these functions plain values.
//
// One record holds one cycle: the entry half (taken at the head of PowerManager::deepSleep()), the early re-sleeps
// folded into it, and the wake half. State: Lost (fresh, or it failed validation), Idle (no cycle pending), Entry
// (asleep, or in a chain of early re-sleeps), Ready (a row waits for the SD card: k = S for a cycle, k = B for an
// orphan boot with no recorded entry). The transitions are tail, early, onWake and flushed below; the table is in
// test/host/README.md and every row of it is checked by test/host/test_batterylog_core.cpp.
// Every function is constexpr and takes the record by reference: no heap, no static state.

#include <cstddef>
#include <cstdint>

namespace freeink {
namespace batterylog {
namespace core {

// --- Constants ----------------------------------------------------------------------------------------------------
constexpr uint8_t SCHEMA_VERSION = 1;
constexpr uint8_t METHOD_ID = 1;  // bumped whenever a burst or its placement changes
constexpr uint8_t BURST_WARMUP = 2;
constexpr uint8_t BURST_COUNT = 64;
constexpr uint8_t BURST_RAW = 16;
constexpr uint32_t LOCK_WAIT_MS = 20;
constexpr uint16_t MAX_PIN_MV = 4095;  // a conversion above this is clamped, so the sums cannot overflow
constexpr uint16_t MAX_MUL_X10 = 40;   // the divider multiplier, times ten, is clamped to this
constexpr uint32_t ABSENT32 = 0xFFFFFFFFu;
constexpr uint16_t ABSENT16 = 0xFFFFu;
constexpr uint8_t ABSENT8 = 0xFFu;
constexpr uint32_t SLOW_FLUSH_MS = 250;
constexpr uint8_t SD_BAD_LIMIT = 2;
constexpr uint8_t SD_SKIP_WAKES = 63;
constexpr uint32_t FILE_CAP_BYTES = 2u * 1024u * 1024u;
constexpr uint32_t UTC_FLOOR = 1735689600u;  // 2025-01-01T00:00:00Z; an earlier time() is not a synced clock
constexpr size_t ROW_BUFFER_BYTES = 384;

// The character of the end-of-session path column.
enum class Path : uint8_t {
  None = 0,  // no tag set
  Manual = 'm',
  Auto = 'a',
  Ceiling = 'k',
  InputCap = 'i',
  Unattended = 'w',
  Cutoff = 'b',
  SdError = 'e',
  QuickLock = 'q',
  Unknown = 'x',
};
enum class EarlyKind : uint8_t { Ghost, Cutoff };
enum class Reset : uint8_t { DeepSleep, Software, Abnormal, Other };
enum class State : uint8_t { Lost = 0, Idle = 1, Entry = 2, Ready = 3 };

// fl_e, the entry flags (append only).
namespace entryflag {
constexpr uint16_t LIT = 1u << 0;                  // front light lit at the trigger
constexpr uint16_t RADIO_ON = 1u << 1;             // radio on at the trigger
constexpr uint16_t TOUCH_NOT_ASLEEP = 1u << 2;     // touch controller did not accept hibernate
constexpr uint16_t SDA_LOW = 1u << 3;              // I2C SDA still low after bus recovery
constexpr uint16_t KEY_HELD = 1u << 4;             // power key held after the release wait: a release wake is armed
constexpr uint16_t NO_VALID = 1u << 5;             // the entry burst has no valid conversion
constexpr uint16_t NO_ADC_PIN = 1u << 6;           // no battery ADC pin: no burst
constexpr uint16_t LEDGER_PARTIAL = 1u << 7;       // session ledger incomplete
constexpr uint16_t LOCK_TIMEOUT = 1u << 8;         // burst skipped: the ADC lock was not got within LOCK_WAIT_MS
constexpr uint16_t SETTINGS_NOT_LOADED = 1u << 9;  // cfg and tmo are empty
constexpr uint16_t STAGED_MASK = LIT | RADIO_ON | TOUCH_NOT_ASLEEP | SDA_LOW | KEY_HELD;
}  // namespace entryflag

// fl_w, the wake flags (append only).
namespace wakeflag {
constexpr uint16_t ORPHAN = 1u << 0;            // k = B
constexpr uint16_t RTC_BACKWARDS = 1u << 1;     // the RTC counter went backwards: sleep_ms unknown
constexpr uint16_t NOT_DEEP_SLEEP = 1u << 2;    // the reset was not a deep-sleep wake
constexpr uint16_t CHARGE_UNDECODED = 1u << 3;  // the charge ladder was not decoded at the flush
constexpr uint16_t NO_VALID = 1u << 4;          // the wake burst was not taken or has no valid conversion
constexpr uint16_t NOT_EXT1 = 1u << 5;          // deep-sleep wake, cause not the power-key line
constexpr uint16_t PREV_FLUSH_BAD = 1u << 6;    // the previous flush failed or took over SLOW_FLUSH_MS
constexpr uint16_t LOCK_TIMEOUT = 1u << 7;      // burst skipped: the ADC lock was not got in time
constexpr uint16_t BACKOFF_SKIPPED = 1u << 8;   // flushes were skipped by the SD back-off since the previous row
constexpr uint16_t FILE_CAP = 1u << 9;          // the file reached its cap with this row: the last one written
}  // namespace wakeflag

// The record's own flag byte.
namespace recflag {
constexpr uint8_t CARRIED = 1u << 0;       // the carried ledger holds a restart's accumulators
constexpr uint8_t RUN_PARTIAL = 1u << 1;   // the running session's ledger is incomplete
constexpr uint8_t KIND_B = 1u << 2;        // the Ready row is an orphan
constexpr uint8_t CLOCK_SYNCED = 1u << 3;  // an NTP sync happened since the record began
constexpr uint8_t U_LATCHED = 1u << 4;     // w_ui is set for this row
constexpr uint8_t PREV_BAD = 1u << 5;      // the previous flush was bad
constexpr uint8_t SKIPPED = 1u << 6;       // a flush was skipped since the last row that reached the card
constexpr uint8_t SKIP_NOW = 1u << 7;      // this wake's flush is skipped (the SD back-off)
}  // namespace recflag

// --- Small integer helpers -----------------------------------------------------------------------------------------
constexpr uint8_t satInc8(const uint8_t v) { return v == 0xFFu ? v : static_cast<uint8_t>(v + 1); }
constexpr uint16_t satInc16(const uint16_t v) { return v == 0xFFFFu ? v : static_cast<uint16_t>(v + 1); }
constexpr uint32_t satInc32(const uint32_t v) { return v == 0xFFFFFFFFu ? v : v + 1; }
constexpr uint32_t satAdd32(const uint32_t a, const uint32_t b) {
  return b > 0xFFFFFFFFu - a ? 0xFFFFFFFFu : a + b;
}
constexpr uint16_t clamp16(const uint64_t v) { return v > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(v); }

// Floor of the square root, bit by bit: no floating point, so a static_assert can check it.
constexpr uint64_t isqrt(const uint64_t v) {
  uint64_t result = 0;
  uint64_t bit = 1ull << 62;
  uint64_t rest = v;
  while (bit > rest) bit >>= 2;
  while (bit != 0) {
    if (rest >= result + bit) {
      rest -= result + bit;
      result = (result >> 1) + bit;
    } else {
      result >>= 1;
    }
    bit >>= 2;
  }
  return result;
}

constexpr uint8_t digits(uint64_t v) {
  uint8_t n = 1;
  while (v >= 10) {
    v /= 10;
    ++n;
  }
  return n;
}

// --- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor; check value 0x29B1) ---------------
constexpr uint16_t crc16(const char* data, const size_t len, uint16_t crc = 0xFFFFu) {
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8));
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// --- The ADC burst: 2 discarded, 64 counted, 16 raw conversions (see BatteryLog.cpp) -------------------------------
// Conversions are calibrated millivolts at the divider pin. A zero is the ADC's silent failure: it is counted in
// `zeros` and kept out of the sums. n + zeros == BURST_COUNT for a burst that was taken; both 0 for one that was not.
struct Burst {
  uint32_t sum;
  uint32_t sumsq;
  uint16_t mn;
  uint16_t mx;
  uint16_t raw16;  // sum of the 16 raw conversions: independent of the calibration
  uint8_t n;
  uint8_t zeros;
};

constexpr bool taken(const Burst& b) { return b.n + b.zeros != 0; }
constexpr bool valid(const Burst& b) { return b.n != 0; }

constexpr void addConversion(Burst& b, uint32_t mv) {
  if (mv == 0) {
    b.zeros = satInc8(b.zeros);
    return;
  }
  if (mv > MAX_PIN_MV) mv = MAX_PIN_MV;
  const uint16_t v = static_cast<uint16_t>(mv);
  if (b.n == 0 || v < b.mn) b.mn = v;
  if (b.n == 0 || v > b.mx) b.mx = v;
  b.sum += v;
  b.sumsq += static_cast<uint32_t>(v) * v;
  b.n = satInc8(b.n);
}

// Mean P+ in 0.1 mV: the pin mean times the divider multiplier (times ten), rounded. Needs valid(b).
constexpr uint32_t meanTenths(const Burst& b, const uint16_t mulX10) {
  return static_cast<uint32_t>((static_cast<uint64_t>(b.sum) * mulX10 + b.n / 2u) / b.n);
}

// Sample standard deviation of the conversions, P+ in 0.1 mV, rounded. Needs valid(b); 0 for a single conversion.
constexpr uint32_t sdTenths(const Burst& b, const uint16_t mulX10) {
  if (b.n < 2) return 0;
  const uint64_t n = b.n;
  const uint64_t spread = n * b.sumsq - static_cast<uint64_t>(b.sum) * b.sum;  // n * (n-1) * variance
  const uint64_t hundredths = isqrt(spread * mulX10 * mulX10 * 10000u / (n * (n - 1)));
  return static_cast<uint32_t>((hundredths + 50) / 100);
}

// Largest minus smallest conversion, P+ in whole mV, rounded. Needs valid(b).
constexpr uint32_t rangeMv(const Burst& b, const uint16_t mulX10) {
  return (static_cast<uint32_t>(b.mx - b.mn) * mulX10 + 5u) / 10u;
}

// The sums fit their fields and the 64-bit steps above cannot overflow, at the clamps' extremes.
static_assert(static_cast<uint64_t>(BURST_COUNT) * MAX_PIN_MV * MAX_PIN_MV <= 0xFFFFFFFFull, "sumsq fits u32");
static_assert(static_cast<uint64_t>(BURST_RAW) * 4095u <= 0xFFFFull, "raw16 fits u16");
static_assert(static_cast<uint64_t>(BURST_COUNT) * (static_cast<uint64_t>(BURST_COUNT) * MAX_PIN_MV * MAX_PIN_MV) <=
                  0xFFFFFFFFFFFFFFFFull / (static_cast<uint64_t>(MAX_MUL_X10) * MAX_MUL_X10 * 10000u),
              "the spread step fits u64");

// --- Intervals ------------------------------------------------------------------------------------------------------
// A stamp is the RTC counter in microseconds; 0 means "not taken".
enum class IntervalStatus : uint8_t { Ok, Unknown, Backwards };
struct Interval {
  IntervalStatus status;
  uint64_t ms;
};

constexpr Interval intervalMs(const uint64_t fromUs, const uint64_t toUs) {
  if (fromUs == 0 || toUs == 0) return {IntervalStatus::Unknown, 0};
  if (toUs < fromUs) return {IntervalStatus::Backwards, 0};
  return {IntervalStatus::Ok, (toUs - fromUs) / 1000u};
}

// Session length, entry minus the previous wake stamp; ABSENT32 when a stamp is missing, the counter went
// backwards or the length does not fit.
constexpr uint32_t awakeMsBetween(const uint64_t wakeUs, const uint64_t entryUs) {
  const Interval iv = intervalMs(wakeUs, entryUs);
  return iv.status == IntervalStatus::Ok && iv.ms < ABSENT32 ? static_cast<uint32_t>(iv.ms) : ABSENT32;
}

// --- Session ledger ---------------------------------------------------------------------------------------------
// The running session's accumulators, in RAM, advanced by tick() from the top of loop().
struct Ledger {
  uint32_t litMs;
  uint64_t litPctMs;  // brightness percent times milliseconds: its mean over litMs is lit_pct
  uint32_t wifiMs;
  uint32_t fullMs;
  uint32_t usbMs;
  uint16_t inp;
  uint16_t rf;  // ABSENT16 until the first refresh is counted
  uint8_t boots;
};

// What the RTC record keeps of a ledger: the weighted brightness collapses to its mean.
struct LedgerRec {
  uint32_t litMs;
  uint32_t wifiMs;
  uint32_t fullMs;
  uint32_t usbMs;
  uint16_t inp;
  uint16_t rf;
  uint8_t litPct;
  uint8_t boots;
};

constexpr Ledger newLedger(const uint8_t boots = 1) {
  Ledger l{};
  l.rf = ABSENT16;
  l.boots = boots;
  return l;
}

constexpr LedgerRec newLedgerRec(const uint8_t boots = 1) {
  LedgerRec l{};
  l.rf = ABSENT16;
  l.boots = boots;
  return l;
}

// One loop pass of `dtMs`. The light counts only above 0 %.
constexpr void tick(Ledger& l, const uint32_t dtMs, const bool lit, const uint8_t litPct, const bool radio,
                    const bool full, const bool usb) {
  if (lit && litPct > 0) {
    l.litMs = satAdd32(l.litMs, dtMs);
    l.litPctMs += static_cast<uint64_t>(dtMs) * (litPct > 100 ? 100 : litPct);
  }
  if (radio) l.wifiMs = satAdd32(l.wifiMs, dtMs);
  if (full) l.fullMs = satAdd32(l.fullMs, dtMs);
  if (usb) l.usbMs = satAdd32(l.usbMs, dtMs);
}

constexpr void noteInput(Ledger& l) { l.inp = satInc16(l.inp); }
constexpr void noteRefresh(Ledger& l) {
  if (l.rf == ABSENT16) {
    l.rf = 1;
  } else if (l.rf < 0xFFFEu) {
    ++l.rf;
  }
}

constexpr LedgerRec compact(const Ledger& l) {
  LedgerRec r{};
  r.litMs = l.litMs;
  r.wifiMs = l.wifiMs;
  r.fullMs = l.fullMs;
  r.usbMs = l.usbMs;
  r.inp = l.inp;
  r.rf = l.rf;
  const uint64_t pct = l.litMs == 0 ? 0 : (l.litPctMs + l.litMs / 2u) / l.litMs;
  r.litPct = static_cast<uint8_t>(pct > 100 ? 100 : pct);
  r.boots = l.boots;
  return r;
}

constexpr Ledger expand(const LedgerRec& r) {
  Ledger l{};
  l.litMs = r.litMs;
  l.litPctMs = static_cast<uint64_t>(r.litPct) * r.litMs;
  l.wifiMs = r.wifiMs;
  l.fullMs = r.fullMs;
  l.usbMs = r.usbMs;
  l.inp = r.inp;
  l.rf = r.rf;
  l.boots = r.boots;
  return l;
}

// --- The record -------------------------------------------------------------------------------------------------
// Plain data, no initialisers: BatteryLog.cpp keeps it in RTC_NOINIT memory behind a magic, version, build id and
// CRC-32. Members are ordered by size, so the only padding is the tails of the ledgers and of the record.
struct Record {
  uint64_t eRtcUs;   // RTC counter at the entry
  uint64_t wRtcUs;   // RTC counter at the wake (also the start of the running session)
  uint32_t nextRow;  // number of the next row; a lost row consumes its number
  uint32_t wUtc;     // time() at the wake, 0 when not synced
  uint32_t awakeMs;  // session length ended by the entry, ABSENT32 unknown
  uint32_t idleMs;   // idle tail at the trigger
  uint32_t gMs;      // summed millis() of the early re-sleeps
  uint32_t gT;       // ms from entry to the last early wake that took a sample, ABSENT32 when none
  uint32_t wDt;      // millis() at the wake sample
  LedgerRec ended;   // the session the entry ended
  LedgerRec carry;   // the running session across a silent restart; carry.boots is always the running count
  Burst eBurst;
  Burst wBurst;
  uint16_t drop;   // rows consumed without reaching the card since the last good row
  uint16_t sdwMs;  // duration of the previous flush, ABSENT16 none yet
  uint16_t eFl;
  uint16_t eUi;  // smoothed UI value at the trigger, 0.1 %, ABSENT16 unknown
  uint16_t eEntMs;
  uint16_t wUi;  // first battery percentage after the wake, 0.1 %, ABSENT16 not yet
  uint16_t wFl;
  uint16_t gMv;  // P+ in 0.1 mV at the last early wake that took a sample, 0 none
  uint8_t state;
  uint8_t flags;
  uint8_t sdBad;   // consecutive bad flushes
  uint8_t sdSkip;  // wakes still to skip
  uint8_t ePath;
  uint8_t eUsb;  // 0 / 1, ABSENT8 unknown
  uint8_t eChg;  // charge status character, 0 unknown
  uint8_t cfg;
  uint8_t tmo;
  uint8_t gN;
  uint8_t gO;
  uint8_t wRst;
  uint8_t wWk;
  uint8_t ab;   // abnormal resets since the previous row
  uint8_t abR;  // the last abnormal reset reason
};
static_assert(sizeof(Record) <= 160, "the record's share of the RTC budget");

constexpr Record freshRecord() {
  Record r{};
  r.nextRow = 1;
  r.awakeMs = ABSENT32;
  r.gT = ABSENT32;
  r.ended = newLedgerRec();
  r.carry = newLedgerRec();
  r.sdwMs = ABSENT16;
  r.eUi = ABSENT16;
  r.wUi = ABSENT16;
  r.eUsb = ABSENT8;
  r.state = static_cast<uint8_t>(State::Lost);
  return r;
}

constexpr State stateOf(const Record& r) { return static_cast<State>(r.state); }
constexpr bool pendingReady(const Record& r) { return stateOf(r) == State::Ready; }
constexpr bool pendingOrphan(const Record& r) { return pendingReady(r) && (r.flags & recflag::KIND_B) != 0; }
// True for the whole boot of a wake whose flush the SD back-off skips.
constexpr bool skipThisWake(const Record& r) { return (r.flags & recflag::SKIP_NOW) != 0; }

constexpr void clearEarly(Record& r) {
  r.gN = 0;
  r.gO = 0;
  r.gMs = 0;
  r.gMv = 0;
  r.gT = ABSENT32;
}

// The wake half. ab and abR are not part of it: they count from one flushed row to the next.
constexpr void clearWake(Record& r) {
  r.wRtcUs = 0;
  r.wUtc = 0;
  r.wDt = 0;
  r.wBurst = Burst{};
  r.wUi = ABSENT16;
  r.wFl = 0;
  r.wRst = 0;
  r.wWk = 0;
}

constexpr void clearEntry(Record& r) {
  r.eRtcUs = 0;
  r.awakeMs = ABSENT32;
  r.idleMs = 0;
  r.ended = newLedgerRec();
  r.eBurst = Burst{};
  r.eFl = 0;
  r.eUi = ABSENT16;
  r.eEntMs = 0;
  r.ePath = 0;
  r.eUsb = ABSENT8;
  r.eChg = 0;
  r.cfg = 0;
  r.tmo = 0;
}

// --- Events -------------------------------------------------------------------------------------------------------
struct EntryIn {
  uint64_t rtcUs;  // the stamp, taken after the burst
  Burst burst;
  bool lockTimeout;  // the burst was skipped: the ADC lock was not got in time
  bool noAdcPin;
  uint8_t path;  // a Path character
  uint8_t usb;
  uint8_t chg;
  uint16_t uiTenths;  // ABSENT16 unknown
  uint16_t entMs;     // trigger to the tail
  uint32_t idleMs;
  uint8_t cfg;
  uint8_t tmoMin;
  bool settingsLoaded;
  uint16_t noteFlags;  // entryflag bits staged by the app and the HAL
  LedgerRec ledger;    // the session this entry ends
};

constexpr void tail(Record& r, const EntryIn& in) {
  if (stateOf(r) == State::Ready) {
    r.drop = satInc16(r.drop);
    r.nextRow = satInc32(r.nextRow);
  }
  const bool lost = stateOf(r) == State::Lost;
  r.awakeMs = r.wRtcUs != 0 ? awakeMsBetween(r.wRtcUs, in.rtcUs) : ABSENT32;
  uint16_t fl = in.noteFlags & entryflag::STAGED_MASK;
  if (!valid(in.burst)) fl |= entryflag::NO_VALID;
  if (in.noAdcPin) fl |= entryflag::NO_ADC_PIN;
  if (in.lockTimeout) fl |= entryflag::LOCK_TIMEOUT;
  if (lost || r.awakeMs == ABSENT32 || (r.flags & recflag::RUN_PARTIAL) != 0) fl |= entryflag::LEDGER_PARTIAL;
  if (!in.settingsLoaded) fl |= entryflag::SETTINGS_NOT_LOADED;
  r.eRtcUs = in.rtcUs;
  r.eBurst = in.burst;
  r.ePath = in.path;
  r.eUsb = in.usb;
  r.eChg = in.chg;
  r.eUi = in.uiTenths;
  r.eEntMs = in.entMs;
  r.idleMs = in.idleMs;
  r.cfg = in.cfg;
  r.tmo = in.tmoMin;
  r.ended = in.ledger;
  r.eFl = fl;
  clearEarly(r);
  clearWake(r);
  r.flags = static_cast<uint8_t>(r.flags & ~(recflag::KIND_B | recflag::U_LATCHED));
  r.state = static_cast<uint8_t>(State::Entry);
}

// Folds an early re-sleep into the pending entry. `awakeMs` is millis() at the re-sleep decision. Returns whether
// the record changed.
constexpr bool early(Record& r, const EarlyKind kind, const uint32_t awakeMs, const uint16_t mulX10) {
  if (pendingReady(r) && !pendingOrphan(r)) {
    if (valid(r.wBurst)) {
      r.gMv = clamp16(meanTenths(r.wBurst, mulX10));
      const Interval iv = intervalMs(r.eRtcUs, r.wRtcUs);
      r.gT = iv.status == IntervalStatus::Ok && iv.ms < ABSENT32 ? static_cast<uint32_t>(iv.ms) : ABSENT32;
    }
    clearWake(r);
    r.flags = static_cast<uint8_t>(r.flags & ~recflag::U_LATCHED);
    r.state = static_cast<uint8_t>(State::Entry);
  }
  if (stateOf(r) != State::Entry) return false;
  if (kind == EarlyKind::Ghost) {
    r.gN = satInc8(r.gN);
  } else {
    r.gO = satInc8(r.gO);
  }
  r.gMs = satAdd32(r.gMs, awakeMs);
  return true;
}

struct WakeIn {
  Reset reset;
  uint8_t rst;    // esp_reset_reason()
  bool extWake;   // the wake cause is the power-key line
  uint64_t rtcUs;
  uint32_t utc;   // time(); kept only once the clock is synced
  uint32_t dtMs;  // millis() at the wake sample
  Burst burst;    // empty unless the reset was a deep-sleep wake
  bool lockTimeout;
  char wk;  // 'h' held through the verify, 'c' accepted under the click-wake rule, 'o' other
};

namespace detail {

constexpr uint32_t syncedUtc(const Record& r, const uint32_t utc) {
  return (r.flags & recflag::CLOCK_SYNCED) != 0 && utc >= UTC_FLOOR ? utc : 0;
}

// The wake half of the row this wake starts (everything but the sleep interval).
constexpr void captureWake(Record& r, const WakeIn& in) {
  clearWake(r);
  r.wRtcUs = in.rtcUs;
  r.wUtc = syncedUtc(r, in.utc);
  r.wDt = in.dtMs;
  r.wBurst = in.burst;
  r.wRst = in.rst;
  r.wWk = static_cast<uint8_t>(in.wk);
  uint16_t fl = 0;
  if (!valid(in.burst)) fl |= wakeflag::NO_VALID;
  if (in.lockTimeout) fl |= wakeflag::LOCK_TIMEOUT;
  if (in.reset != Reset::DeepSleep) fl |= wakeflag::NOT_DEEP_SLEEP;
  if (in.reset == Reset::DeepSleep && !in.extWake) fl |= wakeflag::NOT_EXT1;
  r.wFl = fl;
}

constexpr void countAbnormal(Record& r, const WakeIn& in) {
  if (in.reset != Reset::Abnormal) return;
  r.ab = satInc8(r.ab);
  r.abR = in.rst;
}

// A deep-sleep wake consumes one wake of the SD back-off.
constexpr void skipBookkeeping(Record& r) {
  if (r.sdSkip > 0) {
    --r.sdSkip;
    r.flags |= recflag::SKIP_NOW | recflag::SKIPPED;
  } else {
    r.flags = static_cast<uint8_t>(r.flags & ~recflag::SKIP_NOW);
  }
}

// A new session starts at this wake: a fresh ledger, nothing carried.
constexpr Ledger newSession(Record& r, const bool partial) {
  r.carry = newLedgerRec();
  r.flags = static_cast<uint8_t>(r.flags & ~(recflag::CARRIED | recflag::RUN_PARTIAL));
  if (partial) r.flags |= recflag::RUN_PARTIAL;
  return newLedger();
}

}  // namespace detail

// The wake point, after the wake decision. Returns the ledger the session starts from.
constexpr Ledger onWake(Record& r, const WakeIn& in) {
  const State was = stateOf(r);
  const bool deep = in.reset == Reset::DeepSleep;
  if (!deep && (was == State::Idle || was == State::Ready)) {
    // A restart of the running session (a silent restart, a panic, a watchdog): the wake half, the sleep interval
    // and w_ui stay as captured; the carried ledger comes back, partial when there is none or the reset was abnormal.
    r.carry.boots = satInc8(r.carry.boots);
    Ledger run = (r.flags & recflag::CARRIED) != 0 ? expand(r.carry) : newLedger();
    run.boots = r.carry.boots;
    if ((r.flags & recflag::CARRIED) == 0 || in.reset == Reset::Abnormal) r.flags |= recflag::RUN_PARTIAL;
    r.flags = static_cast<uint8_t>(r.flags & ~recflag::CARRIED);
    detail::countAbnormal(r, in);
    return run;
  }
  if (was == State::Entry) {
    detail::captureWake(r, in);
    if (deep) {
      const Interval iv = intervalMs(r.eRtcUs, r.wRtcUs);
      if (iv.status == IntervalStatus::Backwards) r.wFl |= wakeflag::RTC_BACKWARDS;
      detail::skipBookkeeping(r);
    }
    detail::countAbnormal(r, in);
    r.state = static_cast<uint8_t>(State::Ready);
    return detail::newSession(r, false);
  }
  // Lost, or a wake from a sleep nothing recorded (Idle), or a Ready row that never got flushed: an orphan.
  if (was == State::Ready) {
    r.drop = satInc16(r.drop);
    r.nextRow = satInc32(r.nextRow);
  }
  clearEntry(r);
  clearEarly(r);
  detail::captureWake(r, in);
  r.wFl |= wakeflag::ORPHAN;
  r.flags |= recflag::KIND_B;
  if (deep) detail::skipBookkeeping(r);
  detail::countAbnormal(r, in);
  r.state = static_cast<uint8_t>(State::Ready);
  return detail::newSession(r, was == State::Lost && (in.reset == Reset::Software || in.reset == Reset::Abnormal));
}

// The first battery percentage after the wake, once per row.
constexpr bool mergeUiSeed(Record& r, const uint16_t tenths) {
  if (!pendingReady(r) || (r.flags & recflag::U_LATCHED) != 0) return false;
  r.wUi = tenths;
  r.flags |= recflag::U_LATCHED;
  return true;
}

constexpr void noteClockSynced(Record& r) { r.flags |= recflag::CLOCK_SYNCED; }

// A silent restart is about to happen: keep the running session's accumulators.
constexpr void carryLedger(Record& r, const LedgerRec& l) {
  r.carry = l;
  r.flags |= recflag::CARRIED;
}

// The writer's result for the Ready row. A bad attempt failed or took over SLOW_FLUSH_MS; two in a row skip the
// next SD_SKIP_WAKES wakes, then one probe: a good one clears the count, a bad one skips again.
constexpr void flushed(Record& r, const bool ok, const uint32_t ms) {
  if (!pendingReady(r)) return;
  const bool good = ok && ms <= SLOW_FLUSH_MS;
  r.sdwMs = ms >= ABSENT16 ? static_cast<uint16_t>(ABSENT16 - 1) : static_cast<uint16_t>(ms);
  r.nextRow = satInc32(r.nextRow);
  if (ok) {
    r.drop = 0;
    r.flags = static_cast<uint8_t>(r.flags & ~recflag::SKIPPED);
  } else {
    r.drop = satInc16(r.drop);
  }
  if (good) {
    r.sdBad = 0;
    r.flags = static_cast<uint8_t>(r.flags & ~recflag::PREV_BAD);
  } else {
    r.sdBad = satInc8(r.sdBad);
    if (r.sdBad >= SD_BAD_LIMIT) r.sdSkip = SD_SKIP_WAKES;
    r.flags |= recflag::PREV_BAD;
  }
  r.ab = 0;
  r.abR = 0;
  r.flags = static_cast<uint8_t>(r.flags & ~recflag::KIND_B);
  r.state = static_cast<uint8_t>(State::Idle);
}

// --- The CSV ----------------------------------------------------------------------------------------------------
// Columns are only ever appended. maxWidth is the widest the formatter can print the column.
struct Column {
  const char* name;
  uint8_t maxWidth;
};

namespace width {
constexpr uint8_t U8 = 3;
constexpr uint8_t U16 = 5;
constexpr uint8_t U32 = 10;
constexpr uint8_t MS_OF_US64 = digits(0xFFFFFFFFFFFFFFFFull / 1000u);  // a u64 microsecond count, in ms
constexpr uint8_t HEX4 = 4;
constexpr uint8_t CHAR = 1;
constexpr uint8_t FLAG = 1;
constexpr uint8_t SCHEMA = digits(SCHEMA_VERSION);
constexpr uint8_t METHOD = digits(METHOD_ID);
constexpr uint8_t FW = 9;
// A tenths column: the mean or spread of a burst at the clamps' extremes, or a u16 of tenths, with the point.
constexpr uint8_t MEAN_TENTHS = digits((static_cast<uint64_t>(MAX_PIN_MV) * MAX_MUL_X10) / 10) + 2;
constexpr uint8_t U16_TENTHS = digits(0xFFFFu / 10) + 2;
// The sample sd of conversions inside 1..MAX_PIN_MV is under half that span times n / (n - 1), at most 1 % more.
constexpr uint32_t SD_TENTHS_MAX = (MAX_PIN_MV / 2 + 1u) * MAX_MUL_X10 * 101u / 100u;
constexpr uint8_t SD_TENTHS = digits(SD_TENTHS_MAX / 10) + 2;
constexpr uint8_t RANGE_MV = digits((static_cast<uint64_t>(MAX_PIN_MV) * MAX_MUL_X10 + 5) / 10);
}  // namespace width

constexpr Column COLUMNS[] = {
    {"v", width::SCHEMA},
    {"row", width::U32},
    {"k", width::CHAR},
    {"fw", width::FW},
    {"m", width::METHOD},
    {"rtc_ms", width::MS_OF_US64},
    {"utc", width::U32},
    {"sleep_ms", width::MS_OF_US64},
    {"awake_ms", width::U32},
    {"path", width::CHAR},
    {"e_mv", width::MEAN_TENTHS},
    {"e_sd", width::SD_TENTHS},
    {"e_rg", width::RANGE_MV},
    {"e_z", width::U8},
    {"e_raw", width::U16},
    {"e_ui", width::U16},
    {"usb_e", width::FLAG},
    {"chg_e", width::CHAR},
    {"fl_e", width::HEX4},
    {"ent_ms", width::U16},
    {"g_n", width::U8},
    {"g_o", width::U8},
    {"g_ms", width::U32},
    {"g_mv", width::U16_TENTHS},
    {"g_t", width::U32},
    {"w_mv", width::MEAN_TENTHS},
    {"w_sd", width::SD_TENTHS},
    {"w_rg", width::RANGE_MV},
    {"w_z", width::U8},
    {"w_raw", width::U16},
    {"w_ui", width::U16},
    {"w_dt", width::U32},
    {"usb_w", width::FLAG},
    {"chg_w", width::CHAR},
    {"fl_w", width::HEX4},
    {"rst", width::U8},
    {"wk", width::CHAR},
    {"ab", width::U8},
    {"ab_r", width::U8},
    {"cfg", 2},
    {"tmo", width::U8},
    {"idle_ms", width::U32},
    {"boots", width::U8},
    {"lit_ms", width::U32},
    {"lit_pct", width::U8},
    {"wifi_ms", width::U32},
    {"full_ms", width::U32},
    {"inp", width::U16},
    {"rf", width::U16},
    {"usb_ms", width::U32},
    {"sdw_ms", width::U16},
    {"drop", width::U16},
    {"crc", width::HEX4},
};
constexpr size_t COLUMN_COUNT = sizeof(COLUMNS) / sizeof(COLUMNS[0]);

constexpr size_t cstrLen(const char* s) {
  size_t n = 0;
  while (s[n] != '\0') ++n;
  return n;
}

constexpr size_t headerBytes() {
  size_t n = COLUMN_COUNT - 1;  // the commas
  for (size_t i = 0; i < COLUMN_COUNT; ++i) n += cstrLen(COLUMNS[i].name);
  return n;
}

// The widest row: the leading LF, every column at its widest, the commas, and the terminating NUL.
constexpr size_t worstCaseRowBytes() {
  size_t n = 1 + (COLUMN_COUNT - 1) + 1;
  for (size_t i = 0; i < COLUMN_COUNT; ++i) n += COLUMNS[i].maxWidth;
  return n;
}
static_assert(COLUMN_COUNT == 53, "the schema has 53 columns");
static_assert(worstCaseRowBytes() <= ROW_BUFFER_BYTES, "the widest row fits the writer's buffer");

// Output cursor over a caller buffer. A write that does not fit sets ok = false; n keeps counting.
struct Out {
  char* buf;
  size_t cap;
  size_t n;
  bool ok;
};

constexpr void put(Out& o, const char c) {
  if (o.n < o.cap) {
    o.buf[o.n] = c;
  } else {
    o.ok = false;
  }
  ++o.n;
}

constexpr void putText(Out& o, const char* s) {
  for (size_t i = 0; s[i] != '\0'; ++i) put(o, s[i]);
}

constexpr void putDec(Out& o, uint64_t v) {
  char tmp[20] = {};
  uint8_t len = 0;
  do {
    tmp[len++] = static_cast<char>('0' + v % 10);
    v /= 10;
  } while (v != 0);
  while (len != 0) put(o, tmp[--len]);
}

constexpr void putTenths(Out& o, const uint32_t v) {
  putDec(o, v / 10);
  put(o, '.');
  put(o, static_cast<char>('0' + v % 10));
}

constexpr void putHex(Out& o, const uint16_t v, const uint8_t nibbles) {
  for (uint8_t i = nibbles; i != 0; --i) put(o, "0123456789abcdef"[(v >> (4 * (i - 1))) & 0xF]);
}

// One field, then its comma. An absent value is an empty field.
constexpr void fieldDec(Out& o, const bool present, const uint64_t v) {
  if (present) putDec(o, v);
  put(o, ',');
}
constexpr void fieldTenths(Out& o, const bool present, const uint32_t v) {
  if (present) putTenths(o, v);
  put(o, ',');
}
constexpr void fieldChar(Out& o, const char c) {
  if (c != '\0') put(o, c);
  put(o, ',');
}
constexpr void fieldHex(Out& o, const uint16_t v, const uint8_t nibbles) {
  putHex(o, v, nibbles);
  put(o, ',');
}

// What the writer knows at the flush.
struct RowCtx {
  uint8_t usbW;  // 0 / 1, ABSENT8 unknown
  char chgW;     // '?', 'b', 'c', 'f', 'x', 'n'; 0 unknown
  bool chargeUndecoded;
  bool capReached;  // this row brings the file to its cap: the last one written
  uint16_t mulX10;  // the divider multiplier, times ten
};

// The 16-bit fl_w of the Ready row, with what the flush adds.
constexpr uint16_t wakeFlags(const Record& r, const RowCtx& c) {
  uint16_t fl = r.wFl;
  if ((r.flags & recflag::KIND_B) != 0) fl |= wakeflag::ORPHAN;
  if ((r.flags & recflag::PREV_BAD) != 0) fl |= wakeflag::PREV_FLUSH_BAD;
  if ((r.flags & recflag::SKIPPED) != 0) fl |= wakeflag::BACKOFF_SKIPPED;
  if (c.chargeUndecoded) fl |= wakeflag::CHARGE_UNDECODED;
  if (c.capReached) fl |= wakeflag::FILE_CAP;
  return fl;
}

// The CSV header line: no trailing LF. Returns its length, or 0 when `cap` is too small (it needs one more for a NUL).
constexpr size_t formatHeader(char* buf, const size_t cap) {
  Out o{buf, cap, 0, true};
  for (size_t i = 0; i < COLUMN_COUNT; ++i) {
    if (i != 0) put(o, ',');
    putText(o, COLUMNS[i].name);
  }
  if (!o.ok || o.n >= cap) return 0;
  buf[o.n] = '\0';
  return o.n;
}

// "# fw <build id> <app version> m<method> w<boot wait ms> ser<0|1>", after a LF like every line of the file.
constexpr size_t formatFwLine(char* buf, const size_t cap, const char* fw, const char* version, const uint16_t waitMs,
                              const bool serial) {
  Out o{buf, cap, 0, true};
  put(o, '\n');
  putText(o, "# fw ");
  putText(o, fw);
  put(o, ' ');
  putText(o, version);
  putText(o, " m");
  putDec(o, METHOD_ID);
  putText(o, " w");
  putDec(o, waitMs);
  putText(o, " ser");
  put(o, serial ? '1' : '0');
  if (!o.ok || o.n >= cap) return 0;
  buf[o.n] = '\0';
  return o.n;
}

// The Ready row as written to the file: a LF, the 53 fields, the CRC-16 of the text between the LF and the CRC
// (first field through the last comma) as four lowercase hex digits, and a NUL that is not counted. Returns the
// length, or 0 when it does not fit `cap`. `fw` is the build id (9 hex characters).
constexpr size_t formatRow(char* buf, const size_t cap, const Record& r, const char* fw, const RowCtx& c) {
  Out o{buf, cap, 0, true};
  const bool b = (r.flags & recflag::KIND_B) != 0;
  const uint16_t mul = c.mulX10;
  const uint16_t eFl = r.eFl;
  const bool noSettings = (eFl & entryflag::SETTINGS_NOT_LOADED) != 0;
  const Interval sleep = intervalMs(r.eRtcUs, r.wRtcUs);
  const bool sleepKnown = !b && sleep.status == IntervalStatus::Ok && (r.wFl & wakeflag::NOT_DEEP_SLEEP) == 0;
  const bool eTaken = !b && taken(r.eBurst);
  const bool eValid = !b && valid(r.eBurst);
  const bool wTaken = taken(r.wBurst);
  const bool wValid = valid(r.wBurst);
  const bool gSample = !b && r.gMv != 0;

  put(o, '\n');
  fieldDec(o, true, SCHEMA_VERSION);                                         // 1 v
  fieldDec(o, true, r.nextRow);                                              // 2 row
  fieldChar(o, b ? 'B' : 'S');                                               // 3 k
  putText(o, fw);                                                            // 4 fw
  put(o, ',');
  fieldDec(o, true, METHOD_ID);                                              // 5 m
  fieldDec(o, true, r.wRtcUs / 1000u);                                       // 6 rtc_ms
  fieldDec(o, r.wUtc != 0, r.wUtc);                                          // 7 utc
  fieldDec(o, sleepKnown, sleep.ms);                                         // 8 sleep_ms
  fieldDec(o, !b && r.awakeMs != ABSENT32, r.awakeMs);                       // 9 awake_ms
  fieldChar(o, b ? '\0' : static_cast<char>(r.ePath));                       // 10 path
  fieldTenths(o, eValid, eValid ? meanTenths(r.eBurst, mul) : 0);            // 11 e_mv
  fieldTenths(o, eValid, eValid ? sdTenths(r.eBurst, mul) : 0);              // 12 e_sd
  fieldDec(o, eValid, eValid ? rangeMv(r.eBurst, mul) : 0);                  // 13 e_rg
  fieldDec(o, eTaken, r.eBurst.zeros);                                       // 14 e_z
  fieldDec(o, eTaken, r.eBurst.raw16);                                       // 15 e_raw
  fieldDec(o, !b && r.eUi != ABSENT16, r.eUi);                               // 16 e_ui
  fieldDec(o, !b && r.eUsb != ABSENT8, r.eUsb);                              // 17 usb_e
  fieldChar(o, b ? '\0' : static_cast<char>(r.eChg));                        // 18 chg_e
  if (b) {                                                                   // 19 fl_e
    put(o, ',');
  } else {
    fieldHex(o, eFl, 4);
  }
  fieldDec(o, !b, r.eEntMs);                                                 // 20 ent_ms
  fieldDec(o, !b, r.gN);                                                     // 21 g_n
  fieldDec(o, !b, r.gO);                                                     // 22 g_o
  fieldDec(o, !b, r.gMs);                                                    // 23 g_ms
  fieldTenths(o, gSample, r.gMv);                                            // 24 g_mv
  fieldDec(o, gSample && r.gT != ABSENT32, r.gT);                            // 25 g_t
  fieldTenths(o, wValid, wValid ? meanTenths(r.wBurst, mul) : 0);            // 26 w_mv
  fieldTenths(o, wValid, wValid ? sdTenths(r.wBurst, mul) : 0);              // 27 w_sd
  fieldDec(o, wValid, wValid ? rangeMv(r.wBurst, mul) : 0);                  // 28 w_rg
  fieldDec(o, wTaken, r.wBurst.zeros);                                       // 29 w_z
  fieldDec(o, wTaken, r.wBurst.raw16);                                       // 30 w_raw
  fieldDec(o, r.wUi != ABSENT16, r.wUi);                                     // 31 w_ui
  fieldDec(o, wTaken, r.wDt);                                                // 32 w_dt
  fieldDec(o, c.usbW != ABSENT8, c.usbW);                                    // 33 usb_w
  fieldChar(o, c.chgW);                                                      // 34 chg_w
  fieldHex(o, wakeFlags(r, c), 4);                                           // 35 fl_w
  fieldDec(o, true, r.wRst);                                                 // 36 rst
  fieldChar(o, static_cast<char>(r.wWk));                                    // 37 wk
  fieldDec(o, true, r.ab);                                                   // 38 ab
  fieldDec(o, r.ab != 0, r.abR);                                             // 39 ab_r
  if (b || noSettings) {                                                     // 40 cfg
    put(o, ',');
  } else {
    fieldHex(o, r.cfg, 2);
  }
  fieldDec(o, !b && !noSettings, r.tmo);                                     // 41 tmo
  fieldDec(o, !b, r.idleMs);                                                 // 42 idle_ms
  fieldDec(o, !b, r.ended.boots);                                            // 43 boots
  fieldDec(o, !b, r.ended.litMs);                                            // 44 lit_ms
  fieldDec(o, !b, r.ended.litPct);                                           // 45 lit_pct
  fieldDec(o, !b, r.ended.wifiMs);                                           // 46 wifi_ms
  fieldDec(o, !b, r.ended.fullMs);                                           // 47 full_ms
  fieldDec(o, !b, r.ended.inp);                                              // 48 inp
  fieldDec(o, !b && r.ended.rf != ABSENT16, r.ended.rf);                     // 49 rf
  fieldDec(o, !b, r.ended.usbMs);                                            // 50 usb_ms
  fieldDec(o, r.sdwMs != ABSENT16, r.sdwMs);                                 // 51 sdw_ms
  fieldDec(o, true, r.drop);                                                 // 52 drop
  if (!o.ok) return 0;
  putHex(o, crc16(buf + 1, o.n - 1), 4);                                     // 53 crc
  if (!o.ok || o.n >= cap) return 0;
  buf[o.n] = '\0';
  return o.n;
}

}  // namespace core
}  // namespace batterylog
}  // namespace freeink

#endif  // SILKSCREEN_BATLOG
