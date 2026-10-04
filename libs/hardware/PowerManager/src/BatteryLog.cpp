#if SILKSCREEN_BATLOG

#include <Arduino.h>
#include <SharedAdc.h>
#include <esp_app_desc.h>
#include <esp_attr.h>
#include <esp_rom_crc.h>
#include <esp_rtc_time.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <ctime>

#include "BatteryLog.h"

namespace freeink {
namespace batterylog {
namespace {

constexpr uint32_t MAGIC = 0x42544c47;  // 'BTLG'
constexpr uint8_t LAYOUT_VERSION = 1;   // bump with the layout of Rec or core::Record
constexpr size_t BUILD_ID_CHARS = 9;    // CONFIG_APP_RETRIEVE_LEN_ELF_SHA

// The RTC_NOINIT record. The CRC-32 covers everything after the crc field, padding included, and is rewritten after
// every change, so a write cut short by a brown-out reads as a lost record rather than as a half-updated one.
struct Rec {
  uint32_t magic;
  uint32_t crc;
  uint8_t ver;
  char fw[BUILD_ID_CHARS + 1];
  core::Record r;
};
static_assert(sizeof(Rec) <= 192, "the battery log's RTC budget");

RTC_NOINIT_ATTR Rec rec;

// --- RAM state, gone at every reset ---------------------------------------------------------------------------------
bool stagedValid = false;
Intent stagedIntent;
Path pathHint = Path::None;
uint16_t stagedNotes = 0;  // entryflag bits from the app's intent and the HAL's notes
uint32_t stagedStartMs = 0;
core::LedgerRec stagedLedger = core::newLedgerRec();

core::Ledger runLedger = core::newLedger();  // the running session
bool ticked = false;
uint32_t lastTickMs = 0;

// Written from other tasks, merged by the loop task.
std::atomic<uint32_t> refreshes{0};
std::atomic<bool> clockSynced{false};
constexpr uint32_t UI_SEED_PENDING = 1u << 16;
std::atomic<uint32_t> uiSeed{0};

// What the loop reads every pass, kept in step with the record by seal().
struct Cache {
  bool ready = false;
  bool orphan = false;
  bool skip = false;
  bool valid = false;
  uint8_t state = 0;
  uint8_t sdBad = 0;
  uint8_t sdSkip = 0;
  uint16_t drop = 0;
  uint32_t nextRow = 0;
};
Cache cache;

// --- The record --------------------------------------------------------------------------------------------------
const char* elfHash() {
  static char id[BUILD_ID_CHARS + 1];
  static bool have = false;
  if (!have) {
    esp_app_get_elf_sha256(id, sizeof(id));
    have = true;
  }
  return id;
}

uint32_t crcOfRecord() {
  return esp_rom_crc32_le(0, reinterpret_cast<const uint8_t*>(&rec.ver), sizeof(Rec) - offsetof(Rec, ver));
}

void refreshCache() {
  const core::Record& r = rec.r;
  cache.valid = true;
  cache.state = r.state;
  cache.ready = core::pendingReady(r);
  cache.orphan = core::pendingOrphan(r);
  cache.skip = core::skipThisWake(r);
  cache.sdBad = r.sdBad;
  cache.sdSkip = r.sdSkip;
  cache.drop = r.drop;
  cache.nextRow = r.nextRow;
}

// Call after every change to the record.
void seal() {
  rec.magic = MAGIC;
  rec.ver = LAYOUT_VERSION;
  rec.crc = crcOfRecord();
  refreshCache();
}

bool recordValid() {
  return rec.magic == MAGIC && rec.ver == LAYOUT_VERSION && std::memcmp(rec.fw, elfHash(), sizeof(rec.fw)) == 0 &&
         rec.crc == crcOfRecord();
}

// A record that failed its checks (power loss, a new image, garbage) starts again at row 1, in the Lost state.
void startFresh() {
  std::memset(&rec, 0, sizeof(rec));
  std::strncpy(rec.fw, elfHash(), BUILD_ID_CHARS);
  rec.r = core::freshRecord();
  seal();
}

void openRecord() {
  if (!recordValid()) startFresh();
}

// --- Measurements -----------------------------------------------------------------------------------------------
// The divider multiplier times ten, within the range the integer arithmetic is proven for.
uint16_t dividerMulX10() {
  const long v = std::lround(BoardConfig::ACTIVE.batteryDividerMultiplier * 10.0f);
  return static_cast<uint16_t>(v < 10 ? 10 : (v > core::MAX_MUL_X10 ? core::MAX_MUL_X10 : v));
}

// Two discarded conversions, 64 counted and 16 raw, under one ADC lock taken with a bound. Never lights the front
// light, never touches a pin mode: IO8 is read the way every status-bar read reads it. On a lock miss nothing is read.
core::Burst takeBurst(bool& lockTimeout, bool& noAdcPin) {
  core::Burst b{};
  const int8_t pin = BoardConfig::ACTIVE.batteryAdc;
  if (pin < 0) {
    noAdcPin = true;
    return b;
  }
  const adc::Lock lock(pdMS_TO_TICKS(core::LOCK_WAIT_MS));
  if (!lock.held()) {
    lockTimeout = true;
    return b;
  }
  const uint8_t adcPin = static_cast<uint8_t>(pin);
  for (uint8_t i = 0; i < core::BURST_WARMUP; ++i) analogReadMilliVolts(adcPin);
  for (uint8_t i = 0; i < core::BURST_COUNT; ++i) core::addConversion(b, analogReadMilliVolts(adcPin));
  uint32_t raw = 0;
  for (uint8_t i = 0; i < core::BURST_RAW; ++i) raw += static_cast<uint32_t>(analogRead(adcPin));
  b.raw16 = core::clamp16(raw);
  return b;
}

core::Reset classify(const esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_DEEPSLEEP:
      return core::Reset::DeepSleep;
    case ESP_RST_SW:
      return core::Reset::Software;
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
    case ESP_RST_CPU_LOCKUP:
      return core::Reset::Abnormal;
    default:
      return core::Reset::Other;
  }
}

uint32_t systemSeconds() {
  const time_t now = time(nullptr);
  return now > 0 && now <= static_cast<time_t>(0xFFFFFFFFu) ? static_cast<uint32_t>(now) : 0;
}

// --- Notes left by other tasks ---------------------------------------------------------------------------------
void foldRefreshes() {
  uint32_t n = refreshes.exchange(0, std::memory_order_relaxed);
  if (n > 0xFFFEu) n = 0xFFFEu;
  while (n-- != 0) core::noteRefresh(runLedger);
}

// The clock flag and the first battery percentage go into the record, which only the loop task writes.
void mergeNotes() {
  foldRefreshes();
  if (!clockSynced.load(std::memory_order_relaxed) && (uiSeed.load(std::memory_order_relaxed) & UI_SEED_PENDING) == 0) {
    return;
  }
  if (!recordValid()) return;
  if (clockSynced.exchange(false, std::memory_order_relaxed)) core::noteClockSynced(rec.r);
  const uint32_t seed = uiSeed.exchange(0, std::memory_order_relaxed);
  if ((seed & UI_SEED_PENDING) != 0) core::mergeUiSeed(rec.r, static_cast<uint16_t>(seed & 0xFFFFu));
  seal();
}

// The clock flag alone, for the two places that seal the record with no loop pass before them: the sleep tail and a
// silent restart. A sync that lands after the last tick() (CrossPoint joins Wi-Fi and syncs inside its sleep entry,
// while no loop pass runs) would otherwise die with the RAM flag at the reset. The UI seed stays for tick(): it belongs
// to the row being worked on, and the tail is about to drop that row. The caller seals. RAM only, no wait.
void mergeClockFlag() {
  if (clockSynced.exchange(false, std::memory_order_relaxed)) core::noteClockSynced(rec.r);
}

}  // namespace

void beginEntry(const Intent& intent) {
  foldRefreshes();
  stagedIntent = intent;
  if (pathHint != Path::None) stagedIntent.path = pathHint;
  pathHint = Path::None;
  stagedLedger = core::compact(runLedger);
  stagedStartMs = millis();
  stagedNotes = static_cast<uint16_t>((intent.lightOn ? core::entryflag::LIT : 0) |
                                      (intent.radioOn ? core::entryflag::RADIO_ON : 0));
  stagedValid = true;
}

void tagPath(const Path path) { pathHint = path; }

void noteFlag(const Flag flag) {
  stagedNotes |= flag == Flag::TouchNotAsleep ? core::entryflag::TOUCH_NOT_ASLEEP : core::entryflag::SDA_LOW;
}

void tail(const bool releaseArmed) {
  if (!stagedValid) return;
  stagedValid = false;
  core::EntryIn in{};
  in.burst = takeBurst(in.lockTimeout, in.noAdcPin);
  in.rtcUs = esp_rtc_get_time_us();
  in.path = static_cast<uint8_t>(stagedIntent.path);
  in.usb = stagedIntent.usb;
  in.chg = static_cast<uint8_t>(stagedIntent.chg);
  in.uiTenths = stagedIntent.uiTenths == 0 ? core::ABSENT16 : stagedIntent.uiTenths;
  const uint32_t sinceTrigger = millis() - stagedStartMs;
  in.entMs = core::clamp16(sinceTrigger);
  in.idleMs = stagedIntent.idleMs;
  in.cfg = stagedIntent.cfg;
  in.tmoMin = stagedIntent.tmoMin;
  in.settingsLoaded = stagedIntent.settingsLoaded;
  in.noteFlags = static_cast<uint16_t>(stagedNotes | (releaseArmed ? core::entryflag::KEY_HELD : 0));
  in.ledger = stagedLedger;
  openRecord();
  mergeClockFlag();
  core::tail(rec.r, in);
  seal();
}

void early(const EarlyKind kind, const uint32_t awakeMs) {
  if (!recordValid()) return;
  if (core::early(rec.r, kind, awakeMs, dividerMulX10())) seal();
}

void onWake(const WakeInfo& info) {
  const esp_reset_reason_t reason = esp_reset_reason();
  core::WakeIn in{};
  in.reset = classify(reason);
  in.rst = static_cast<uint8_t>(reason);
  const bool deep = in.reset == core::Reset::DeepSleep;
  in.extWake = deep && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
  in.dtMs = millis();
  if (deep) {
    bool noAdcPin = false;
    in.burst = takeBurst(in.lockTimeout, noAdcPin);
  }
  in.rtcUs = esp_rtc_get_time_us();
  in.utc = systemSeconds();
  in.wk = in.extWake ? (info.holdVerified ? 'h' : (info.clickWake ? 'c' : 'o')) : 'o';
  openRecord();
  runLedger = core::onWake(rec.r, in);
  seal();
  ticked = false;
  refreshes.store(0, std::memory_order_relaxed);
}

void noteUiSeed(const uint16_t tenths) { uiSeed.store(UI_SEED_PENDING | tenths, std::memory_order_relaxed); }

void noteClockSynced() { clockSynced.store(true, std::memory_order_relaxed); }

void tick(const TickIn& in) {
  const uint32_t now = millis();
  mergeNotes();
  if (!ticked) {
    ticked = true;
    lastTickMs = now;
    return;
  }
  const uint32_t dt = now - lastTickMs;
  lastTickMs = now;
  core::tick(runLedger, dt, in.lit, in.litPct, in.radioOn, in.fullClock, in.usb);
}

void noteInput() { core::noteInput(runLedger); }

void noteRefresh() { refreshes.fetch_add(1, std::memory_order_relaxed); }

void carry() {
  foldRefreshes();
  if (!recordValid()) return;
  mergeClockFlag();
  core::carryLedger(rec.r, core::compact(runLedger));
  seal();
}

Pending pending() { return {cache.ready, cache.orphan}; }

bool sdBackoff() { return cache.skip; }

uint8_t sdBadCount() { return cache.sdBad; }

size_t formatRow(char* buf, const size_t cap, const FlushCtx& ctx) {
  if (!cache.ready || !recordValid()) return 0;
  core::RowCtx c{};
  c.usbW = ctx.usb;
  c.chgW = ctx.chg;
  c.chargeUndecoded = ctx.chargeUndecoded;
  c.capReached = ctx.fileCapReached;
  c.mulX10 = dividerMulX10();
  return core::formatRow(buf, cap, rec.r, rec.fw, c);
}

size_t formatHeader(char* buf, const size_t cap) { return core::formatHeader(buf, cap); }

size_t formatFwLine(char* buf, const size_t cap, const char* version, const uint16_t bootWaitMs, const bool serialLog) {
  return core::formatFwLine(buf, cap, elfHash(), version, bootWaitMs, serialLog);
}

void flushed(const bool ok, const uint32_t ms) {
  if (!recordValid()) {
    // The record stopped validating while a row waited, so formatRow() wrote nothing. Start again in Lost: that also
    // refreshes the cache the loop reads, so the writer's one attempt is the last of this wake.
    startFresh();
    return;
  }
  core::flushed(rec.r, ok, ms);
  seal();
}

const char* buildId() { return elfHash(); }

Status status() {
  Status s;
  s.valid = cache.valid;
  s.state = cache.state;
  s.nextRow = cache.nextRow;
  s.drop = cache.drop;
  s.sdBad = cache.sdBad;
  s.sdSkip = cache.sdSkip;
  return s;
}

}  // namespace batterylog
}  // namespace freeink

#endif  // SILKSCREEN_BATLOG
