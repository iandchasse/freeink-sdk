#if SILKSCREEN_BATLOG
// Compile-time test of the battery log's pure core (BatteryLogCore.h): every check is a static_assert, so it needs
// no host compiler to run, only one that can parse it (the xtensa g++ with -fsyntax-only does). A story function
// returns 0 when all its checks hold, or the source line of the first one that does not; GCC prints that line in the
// "comparison reduces to" note of the failed static_assert.
#include <BatteryLogCore.h>

#include <cstddef>
#include <cstdint>

namespace core = freeink::batterylog::core;
using core::Burst;
using core::EarlyKind;
using core::EntryIn;
using core::Ledger;
using core::LedgerRec;
using core::Record;
using core::Reset;
using core::RowCtx;
using core::State;
using core::WakeIn;
namespace entryflag = core::entryflag;
namespace wakeflag = core::wakeflag;
namespace recflag = core::recflag;

namespace {

#define CHECK(condition) \
  do {                   \
    if (!(condition)) {  \
      return __LINE__;   \
    }                    \
  } while (0)

// --- Text helpers -------------------------------------------------------------------------------------------------
struct Text {
  char c[core::ROW_BUFFER_BYTES];
  size_t n;
};

constexpr const char* FW = "f3a9c01d2";

constexpr Text rowText(const Record& r, const RowCtx& ctx) {
  Text t{};
  t.n = core::formatRow(t.c, sizeof(t.c), r, FW, ctx);
  return t;
}

constexpr bool same(const char* a, const char* b) {
  size_t i = 0;
  while (a[i] != '\0' && a[i] == b[i]) ++i;
  return a[i] == b[i];
}

template <size_t N>
constexpr bool bufIs(const char* buf, const size_t n, const char (&want)[N]) {
  if (n != N - 1) return false;
  for (size_t i = 0; i < n; ++i) {
    if (buf[i] != want[i]) return false;
  }
  return buf[n] == '\0';
}

template <size_t N>
constexpr bool textIs(const Text& t, const char (&want)[N]) {
  return bufIs(t.c, t.n, want);
}

// The text of column `name`, compared with `want`.
constexpr bool fieldIs(const Text& t, const char* name, const char* want) {
  size_t index = core::COLUMN_COUNT;
  for (size_t i = 0; i < core::COLUMN_COUNT; ++i) {
    if (same(core::COLUMNS[i].name, name)) index = i;
  }
  size_t field = 0;
  size_t start = 1;  // after the LF
  for (size_t i = 1; i <= t.n; ++i) {
    if (i == t.n || t.c[i] == ',') {
      if (field == index) {
        size_t k = 0;
        while (start + k < i && want[k] != '\0' && t.c[start + k] == want[k]) ++k;
        return start + k == i && want[k] == '\0';
      }
      ++field;
      start = i + 1;
    }
  }
  return false;
}

constexpr int hexValue(const char c) { return c >= 'a' ? c - 'a' + 10 : c - '0'; }

// The row's last four characters are the CRC-16 of everything between the LF and them.
constexpr bool crcOk(const Text& t) {
  if (t.n < 6 || t.c[0] != '\n' || t.c[t.n - 5] != ',') return false;
  uint16_t shown = 0;
  for (size_t i = t.n - 4; i < t.n; ++i) shown = static_cast<uint16_t>(shown * 16 + hexValue(t.c[i]));
  return core::crc16(t.c + 1, t.n - 5) == shown;
}

constexpr size_t commas(const Text& t) {
  size_t n = 0;
  for (size_t i = 0; i < t.n; ++i) n += t.c[i] == ',' ? 1 : 0;
  return n;
}

// --- Fixtures -----------------------------------------------------------------------------------------------------
// 64 calibrated conversions at the divider pin behind the design's example row: means 1998.5 and 2003.25 mV, so P+
// reads 3997.0 and 4006.5 mV. Their sample sd is 2.6 and 2.35 mV and their ranges 12 and 11 mV, which is what 64
// conversions of that noise look like (about 4.7 sd of range), i.e. 5.2 / 4.7 and 24 / 22 mV at P+.
constexpr uint16_t E_SAMPLES[64] = {2001, 2000, 2002, 2001, 1998, 1998, 2003, 2000, 2001, 2002, 1996, 1996, 2001,
                                    1997, 1996, 1997, 1999, 2000, 1996, 2000, 1999, 1996, 1993, 2002, 1996, 1996,
                                    1996, 1993, 2000, 2003, 2000, 1999, 1997, 1997, 1997, 1999, 1998, 2001, 1995,
                                    1996, 1996, 1999, 2003, 1998, 1995, 1998, 2000, 2000, 2004, 2002, 1999, 1998,
                                    1999, 1996, 1997, 1996, 1998, 1999, 2000, 2001, 1998, 1997, 1992, 2002};
constexpr uint16_t W_SAMPLES[64] = {2004, 1999, 2001, 2003, 2007, 2003, 1999, 2004, 2000, 2006, 2003, 2007, 2003,
                                    2001, 2000, 2002, 2004, 2006, 2004, 2002, 2004, 2000, 2004, 2001, 2007, 2006,
                                    2004, 2001, 2005, 2004, 2002, 2002, 2006, 2002, 2004, 2007, 2000, 2007, 2006,
                                    2002, 2001, 2000, 2005, 1997, 2005, 2004, 2002, 2002, 2002, 2003, 2005, 2001,
                                    2006, 2006, 2003, 2005, 2002, 2003, 2004, 2008, 2003, 2003, 2003, 2003};
// The 16 raw conversions at the calibrated local slope of 0.8693 mV per count: 2299.0 and 2304.4 counts on average.
constexpr uint16_t E_RAW16 = 36784;
constexpr uint16_t W_RAW16 = 36871;

constexpr Burst burstFrom(const uint16_t (&samples)[64], const uint16_t raw16) {
  Burst b{};
  for (size_t i = 0; i < 64; ++i) core::addConversion(b, samples[i]);
  b.raw16 = raw16;
  return b;
}

// 64 identical conversions: P+ is exactly twice `mv`.
constexpr Burst flatBurst(const uint16_t mv) {
  Burst b{};
  for (size_t i = 0; i < 64; ++i) core::addConversion(b, mv);
  b.raw16 = static_cast<uint16_t>(16u * mv * 100u / 87u);
  return b;
}

// 64 conversions that all came back 0: taken, nothing valid.
constexpr Burst zeroBurst() {
  Burst b{};
  for (size_t i = 0; i < 64; ++i) core::addConversion(b, 0);
  b.raw16 = 1234;
  return b;
}

constexpr uint16_t MUL = 20;          // Silkscreen's 1 MOhm / 1 MOhm divider
constexpr uint64_t SEC = 1000000ull;  // microseconds

constexpr EntryIn entryIn(const uint64_t rtcUs, const Burst& burst = flatBurst(1900)) {
  EntryIn in{};
  in.rtcUs = rtcUs;
  in.burst = burst;
  in.path = static_cast<uint8_t>(core::Path::Manual);
  in.usb = 0;
  in.chg = 'b';
  in.uiTenths = 800;
  in.entMs = 2000;
  in.idleMs = 1000;
  in.cfg = 0x0d;
  in.tmoMin = 10;
  in.settingsLoaded = true;
  in.ledger = core::newLedgerRec();
  return in;
}

constexpr WakeIn wakeIn(const Reset reset, const uint64_t rtcUs, const Burst& burst = flatBurst(2000)) {
  WakeIn in{};
  in.reset = reset;
  in.rst = reset == Reset::DeepSleep ? 8 : (reset == Reset::Software ? 3 : (reset == Reset::Abnormal ? 4 : 1));
  in.extWake = reset == Reset::DeepSleep;
  in.rtcUs = rtcUs;
  in.dtMs = 300;
  in.burst = reset == Reset::DeepSleep ? burst : Burst{};
  in.wk = reset == Reset::DeepSleep ? 'h' : 'o';
  return in;
}

constexpr RowCtx rowCtx() {
  RowCtx c{};
  c.usbW = 0;
  c.chgW = 'b';
  c.mulX10 = MUL;
  return c;
}

// A record that has been through a flush: Idle, the last wake stamp known, the clock synced.
constexpr Record idleRecord(const uint32_t nextRow = 10, const uint64_t wakeUs = 1000 * SEC) {
  Record r = core::freshRecord();
  r.state = static_cast<uint8_t>(State::Idle);
  r.nextRow = nextRow;
  r.wRtcUs = wakeUs;
  r.flags = recflag::CLOCK_SYNCED;
  return r;
}

// An entry at `entryAt` and a deep-sleep wake at `wakeAt`: the record is Ready.
constexpr void cycle(Record& r, const uint64_t entryAt, const uint64_t wakeAt) {
  core::tail(r, entryIn(entryAt));
  core::onWake(r, wakeIn(Reset::DeepSleep, wakeAt));
}

// --- CRC-16, square root, burst arithmetic ------------------------------------------------------------------------
constexpr int crcStory() {
  CHECK(core::crc16("123456789", 9) == 0x29B1);  // the CCITT-FALSE check value
  CHECK(core::crc16("", 0) == 0xFFFF);
  CHECK(core::crc16("A", 1) == 0xB915);
  CHECK(core::crc16("56789", 5, core::crc16("1234", 4)) == 0x29B1);  // continuing equals one pass
  return 0;
}
static_assert(crcStory() == 0, "CRC-16/CCITT-FALSE: check value 0x29B1, empty, one byte, continued");

constexpr int sqrtStory() {
  CHECK(core::isqrt(0) == 0);
  CHECK(core::isqrt(1) == 1);
  CHECK(core::isqrt(2) == 1);
  CHECK(core::isqrt(3) == 1);
  CHECK(core::isqrt(4) == 2);
  CHECK(core::isqrt(15) == 3);
  CHECK(core::isqrt(16) == 4);
  CHECK(core::isqrt(17) == 4);
  CHECK(core::isqrt(1000000000000ull) == 1000000);
  CHECK(core::isqrt(999999999999ull) == 999999);
  CHECK(core::isqrt(1ull << 62) == (1ull << 31));
  CHECK(core::isqrt(0xFFFFFFFFFFFFFFFFull) == 0xFFFFFFFFull);
  return 0;
}
static_assert(sqrtStory() == 0, "isqrt is the floor of the square root, up to the 64-bit limit");

constexpr int burstStory() {
  const Burst e = burstFrom(E_SAMPLES, E_RAW16);
  CHECK(e.n == 64 && e.zeros == 0 && core::taken(e) && core::valid(e));
  CHECK(e.sum == 127904 && e.sumsq == 255616576 && e.mn == 1992 && e.mx == 2004);
  CHECK(core::meanTenths(e, MUL) == 39970);  // 3997.0 mV at P+
  CHECK(core::sdTenths(e, MUL) == 52);       // 5.2 mV
  CHECK(core::rangeMv(e, MUL) == 24);
  const Burst w = burstFrom(W_SAMPLES, W_RAW16);
  CHECK(w.sum == 128208 && w.sumsq == 256833018 && w.mn == 1997 && w.mx == 2008);
  CHECK(core::meanTenths(w, MUL) == 40065 && core::sdTenths(w, MUL) == 47 && core::rangeMv(w, MUL) == 22);
  // The raw sums agree with the calibrated means at the 0.8693 mV per count slope, to the millivolt.
  CHECK(E_RAW16 * 8693u / 160000u == 1998 && W_RAW16 * 8693u / 160000u == 2003);
  CHECK(W_RAW16 - E_RAW16 == 87);  // 5.4 counts: the 9.5 mV the two P+ means differ by

  // A zero conversion is the ADC's silent failure: counted, kept out of the sums and the extremes.
  Burst z{};
  core::addConversion(z, 0);
  CHECK(core::taken(z) && !core::valid(z) && z.zeros == 1 && z.n == 0);
  core::addConversion(z, 1500);
  core::addConversion(z, 0);
  core::addConversion(z, 1504);
  CHECK(z.n == 2 && z.zeros == 2 && z.sum == 3004 && z.mn == 1500 && z.mx == 1504);
  CHECK(core::meanTenths(z, MUL) == 30040);
  CHECK(!core::taken(Burst{}) && !core::valid(Burst{}));
  // The mean rounds to the nearest 0.1 mV: a pin mean of 1000.25 is P+ 2000.5.
  Burst h{};
  const uint16_t quarter[4] = {1000, 1000, 1000, 1001};
  for (const uint16_t mv : quarter) core::addConversion(h, mv);
  CHECK(core::meanTenths(h, MUL) == 20005);
  // One conversion has no spread; a reading above the clamp is held at it.
  Burst one{};
  core::addConversion(one, 40000);
  CHECK(one.sum == core::MAX_PIN_MV && core::sdTenths(one, MUL) == 0 && core::rangeMv(one, MUL) == 0);
  // At the clamps' extremes every step stays inside its type: half the conversions at 1, half at 4095.
  Burst x{};
  for (size_t i = 0; i < 64; ++i) core::addConversion(x, i % 2 == 0 ? 1 : 4095);
  CHECK(x.n == 64 && x.sumsq == 32u + 32u * 4095u * 4095u && x.sum == 131072);
  CHECK(core::sdTenths(x, core::MAX_MUL_X10) > 80000 && core::sdTenths(x, core::MAX_MUL_X10) < 99990);
  CHECK(core::rangeMv(x, core::MAX_MUL_X10) == 16376);
  return 0;
}
static_assert(burstStory() == 0, "burst mean, spread and range in integers; zeros and clamps");

constexpr int intervalStory() {
  CHECK(core::intervalMs(1000000, 3500000).status == core::IntervalStatus::Ok);
  CHECK(core::intervalMs(1000000, 3500000).ms == 2500);
  CHECK(core::intervalMs(1000000, 3500999).ms == 2500);  // floors
  CHECK(core::intervalMs(5000000, 4999999).status == core::IntervalStatus::Backwards);
  CHECK(core::intervalMs(0, 4999999).status == core::IntervalStatus::Unknown);
  CHECK(core::intervalMs(5000000, 0).status == core::IntervalStatus::Unknown);
  CHECK(core::awakeMsBetween(1000000, 2000000) == 1000);
  CHECK(core::awakeMsBetween(2000000, 1000000) == core::ABSENT32);  // backwards
  CHECK(core::awakeMsBetween(0, 1000000) == core::ABSENT32);        // no wake stamp
  // A session of 2^32 ms or more does not fit: unknown, not wrapped.
  CHECK(core::awakeMsBetween(1, 1 + 4294967296ull * 1000u) == core::ABSENT32);
  CHECK(core::awakeMsBetween(1, 1 + 4294967294ull * 1000u) == 4294967294u);
  return 0;
}
static_assert(intervalStory() == 0, "sleep and awake arithmetic, including a counter that went backwards");

constexpr int ledgerStory() {
  Ledger l = core::newLedger();
  CHECK(l.boots == 1 && l.rf == core::ABSENT16);
  core::tick(l, 1000, false, 0, false, true, false);  // full clock only
  core::tick(l, 2000, true, 0, true, true, true);     // a light at 0 % is not lit
  core::tick(l, 4000, true, 50, false, false, true);
  core::tick(l, 4000, true, 100, false, false, false);
  CHECK(l.litMs == 8000 && l.litPctMs == 4000u * 50 + 4000u * 100);
  CHECK(l.wifiMs == 2000 && l.fullMs == 3000 && l.usbMs == 6000);
  core::noteInput(l);
  core::noteInput(l);
  core::noteRefresh(l);
  core::noteRefresh(l);
  CHECK(l.inp == 2 && l.rf == 2);
  const LedgerRec r = core::compact(l);
  CHECK(r.litMs == 8000 && r.litPct == 75 && r.wifiMs == 2000 && r.fullMs == 3000 && r.usbMs == 6000);
  CHECK(r.inp == 2 && r.rf == 2 && r.boots == 1);
  const Ledger back = core::expand(r);
  CHECK(back.litMs == 8000 && back.litPctMs == 75u * 8000 && back.boots == 1 && back.rf == 2);
  // The mean brightness rounds to the nearest percent: 1 ms at 100 % and 1 ms at 1 % is 50.5.
  Ledger m = core::newLedger();
  core::tick(m, 1, true, 100, false, false, false);
  core::tick(m, 1, true, 1, false, false, false);
  CHECK(core::compact(m).litPct == 51 && core::compact(core::newLedger()).litPct == 0);
  // Counters stop at their maximum, and a refresh count never reaches the "absent" value.
  Ledger s = core::newLedger();
  s.wifiMs = 0xFFFFFFF0u;
  core::tick(s, 100, false, 0, true, false, false);
  CHECK(s.wifiMs == 0xFFFFFFFFu);
  s.inp = 0xFFFF;
  core::noteInput(s);
  CHECK(s.inp == 0xFFFF);
  s.rf = 0xFFFE;
  core::noteRefresh(s);
  CHECK(s.rf == 0xFFFE);
  CHECK(core::compact(core::newLedger()).rf == core::ABSENT16);  // never counted: an empty field in the row
  return 0;
}
static_assert(ledgerStory() == 0, "the session ledger: ticks, counts, compaction and saturation");

// --- The CSV table, the header, the bound -------------------------------------------------------------------------
constexpr int headerStory() {
  CHECK(core::COLUMN_COUNT == 53);
  Text t{};
  t.n = core::formatHeader(t.c, sizeof(t.c));
  CHECK(t.n == 272 && t.n == core::headerBytes());
  CHECK(t.c[t.n - 1] != '\n');
  CHECK(bufIs(t.c, t.n,
              "v,row,k,fw,m,rtc_ms,utc,sleep_ms,awake_ms,path,e_mv,e_sd,e_rg,e_z,e_raw,e_ui,usb_e,chg_e,fl_e,ent_ms,"
              "g_n,g_o,g_ms,g_mv,g_t,w_mv,w_sd,w_rg,w_z,w_raw,w_ui,w_dt,usb_w,chg_w,fl_w,rst,wk,ab,ab_r,cfg,tmo,"
              "idle_ms,boots,lit_ms,lit_pct,wifi_ms,full_ms,inp,rf,usb_ms,sdw_ms,drop,crc"));
  // The header needs room for its NUL too.
  char small[272] = {};
  CHECK(core::formatHeader(small, sizeof(small)) == 0);
  return 0;
}
static_assert(headerStory() == 0, "the header: 53 columns, 272 bytes, no trailing LF, the design's order");

// Every field at its widest in a k = S row.
constexpr Record widestRecord() {
  Record r = core::freshRecord();
  r.state = static_cast<uint8_t>(State::Ready);
  r.nextRow = 0xFFFFFFFFu;
  r.eRtcUs = 1;
  r.wRtcUs = 0xFFFFFFFFFFFFFFFFull;
  r.wUtc = 0xFFFFFFFFu;
  r.awakeMs = 0xFFFFFFFEu;
  r.idleMs = 0xFFFFFFFFu;
  r.gMs = 0xFFFFFFFFu;
  r.gT = 0xFFFFFFFEu;
  r.wDt = 0xFFFFFFFFu;
  r.ended.litMs = 0xFFFFFFFFu;
  r.ended.wifiMs = 0xFFFFFFFFu;
  r.ended.fullMs = 0xFFFFFFFFu;
  r.ended.usbMs = 0xFFFFFFFFu;
  r.ended.inp = 0xFFFE;
  r.ended.rf = 0xFFFE;
  r.ended.litPct = 100;
  r.ended.boots = 0xFF;
  r.eBurst = flatBurst(core::MAX_PIN_MV);
  r.wBurst = flatBurst(core::MAX_PIN_MV);
  r.eBurst.mn = 1;
  r.wBurst.mn = 1;
  r.eBurst.zeros = 0xFF;
  r.wBurst.zeros = 0xFF;
  r.eBurst.raw16 = 0xFFFF;
  r.wBurst.raw16 = 0xFFFF;
  r.drop = 0xFFFF;
  r.sdwMs = 0xFFFE;
  r.eFl = static_cast<uint16_t>(0xFFFF & ~entryflag::SETTINGS_NOT_LOADED);
  r.eUi = 0xFFFE;
  r.eEntMs = 0xFFFF;
  r.wUi = 0xFFFE;
  r.wFl = static_cast<uint16_t>(0xFFFF & ~wakeflag::NOT_DEEP_SLEEP);  // a deep-sleep wake has a sleep_ms
  r.gMv = 0xFFFF;
  r.ePath = 'x';
  r.eUsb = 1;
  r.eChg = 'x';
  r.cfg = 0xFF;
  r.tmo = 0xFF;
  r.gN = 0xFF;
  r.gO = 0xFF;
  r.wRst = 0xFF;
  r.wWk = 'o';
  r.ab = 0xFF;
  r.abR = 0xFF;
  return r;
}

constexpr int boundStory() {
  RowCtx ctx = rowCtx();
  ctx.usbW = 1;
  ctx.chgW = 'x';
  ctx.mulX10 = core::MAX_MUL_X10;
  const Text t = rowText(widestRecord(), ctx);
  CHECK(t.n != 0 && crcOk(t) && commas(t) == 52);
  // The bound counts the NUL and the widest spread, which a burst whose mean is at its widest does not have (flat
  // bursts: "0.0" in two columns of six). Everything else is at its widest here, so the bound is tight to those six.
  CHECK(t.n + 1 <= core::worstCaseRowBytes());
  CHECK(t.n + 1 + 6 == core::worstCaseRowBytes());
  CHECK(core::worstCaseRowBytes() <= core::ROW_BUFFER_BYTES);
  // A buffer one byte short of the row, which also needs a NUL, is refused rather than truncated.
  char buf[core::ROW_BUFFER_BYTES] = {};
  CHECK(core::formatRow(buf, t.n, widestRecord(), FW, ctx) == 0);
  CHECK(core::formatRow(buf, t.n + 1, widestRecord(), FW, ctx) == t.n);
  return 0;
}
static_assert(boundStory() == 0, "the widest row stays inside the 384-byte writer buffer, and the table is tight");

constexpr int fwLineStory() {
  Text t{};
  t.n = core::formatFwLine(t.c, sizeof(t.c), FW, "1.6.5-batlog", 0, true);
  CHECK(textIs(t, "\n# fw f3a9c01d2 1.6.5-batlog m1 w0 ser1"));
  t.n = core::formatFwLine(t.c, sizeof(t.c), FW, "1.6.5-batlog", 250, false);
  CHECK(textIs(t, "\n# fw f3a9c01d2 1.6.5-batlog m1 w250 ser0"));
  char small[8] = {};
  CHECK(core::formatFwLine(small, sizeof(small), FW, "1.6.5-batlog", 0, true) == 0);
  return 0;
}
static_assert(fwLineStory() == 0, "the '# fw' comment line");

// --- The example row ----------------------------------------------------------------------------------------------
// The design's example: a 25.6 min session ended by the 10 min auto-sleep, two early ghost wakes, a 9.0 h sleep, the
// wake and the flush. Built through the state machine, not filled in by hand.
constexpr Record exampleRecord() {
  Record r = idleRecord(57, 378397333250ull);
  r.sdwMs = 41;

  Ledger session = core::newLedger();
  for (int i = 0; i < 720; ++i) core::tick(session, 1000, true, 40, false, false, false);
  for (int i = 0; i < 410; ++i) core::tick(session, 1000, false, 0, false, true, false);
  for (int i = 0; i < 118; ++i) core::noteInput(session);

  EntryIn in = entryIn(379933333456ull, burstFrom(E_SAMPLES, E_RAW16));
  in.path = static_cast<uint8_t>(core::Path::Auto);
  in.uiTenths = 826;
  in.entMs = 2310;
  in.idleMs = 600000;
  in.ledger = core::compact(session);
  core::tail(r, in);
  core::early(r, EarlyKind::Ghost, 310, MUL);
  core::early(r, EarlyKind::Ghost, 330, MUL);

  WakeIn w = wakeIn(Reset::DeepSleep, 412345678901ull, burstFrom(W_SAMPLES, W_RAW16));
  w.utc = 1791234567;
  w.dtMs = 312;
  core::onWake(r, w);
  core::mergeUiSeed(r, 818);
  return r;
}

constexpr int exampleStory() {
  Record r = exampleRecord();
  CHECK(core::stateOf(r) == State::Ready && !core::pendingOrphan(r));
  const Text t = rowText(r, rowCtx());
  CHECK(t.n == 206);  // 205 bytes of row and the leading LF
  CHECK(crcOk(t) && commas(t) == 52);
  CHECK(core::crc16(t.c + 1, t.n - 5) == 0xC59D);
  CHECK(textIs(t,
               "\n1,57,S,f3a9c01d2,1,412345678,1791234567,32412345,1536000,a,3997.0,5.2,24,0,36784,826,0,b,0000,"
               "2310,2,0,640,,,4006.5,4.7,22,0,36871,818,312,0,b,0000,8,h,0,,0d,10,600000,1,720000,40,0,410000,"
               "118,,0,41,0,c59d"));
  // What the example narrates: the screen showed 83 at the trigger and 82 after the wake, while the wake burst reads
  // 9.5 mV above the loaded entry burst.
  CHECK(fieldIs(t, "e_ui", "826") && fieldIs(t, "w_ui", "818"));
  CHECK(fieldIs(t, "e_mv", "3997.0") && fieldIs(t, "w_mv", "4006.5"));
  CHECK(fieldIs(t, "g_n", "2") && fieldIs(t, "g_mv", "") && fieldIs(t, "g_t", "") && fieldIs(t, "rf", ""));
  // The flush: the row number is consumed, the record is Idle, the duration is what the writer measured.
  core::flushed(r, true, 52);
  CHECK(core::stateOf(r) == State::Idle && r.nextRow == 58 && r.drop == 0 && r.sdwMs == 52 && r.sdBad == 0);
  CHECK(!core::pendingReady(r));
  return 0;
}
static_assert(exampleStory() == 0, "the formatter reproduces the example row, CRC c59d, from a story");

// --- Early re-sleeps ----------------------------------------------------------------------------------------------
constexpr int earlyStory() {
  // Entry -> Entry: ghost and cutoff counters, summed awake time.
  Record r = idleRecord(5, 100 * SEC);
  core::tail(r, entryIn(200 * SEC));
  CHECK(core::early(r, EarlyKind::Ghost, 280, MUL) && core::early(r, EarlyKind::Ghost, 300, MUL));
  CHECK(core::early(r, EarlyKind::Cutoff, 500, MUL));
  CHECK(core::stateOf(r) == State::Entry && r.gN == 2 && r.gO == 1 && r.gMs == 1080 && r.gMv == 0);
  // Idle and Lost: nothing.
  Record idle = idleRecord();
  CHECK(!core::early(idle, EarlyKind::Ghost, 280, MUL) && core::stateOf(idle) == State::Idle && idle.gN == 0);
  Record lost = core::freshRecord();
  CHECK(!core::early(lost, EarlyKind::Ghost, 280, MUL) && core::stateOf(lost) == State::Lost && lost.gN == 0);
  // Ready (S) -> Entry: a wake that took a sample and then re-slept (late ghost, SD-fail ghost, boot cutoff). The wake
  // burst becomes g_mv, its time since the entry g_t, and the wake half is gone.
  Record s = idleRecord(7, 100 * SEC);
  core::tail(s, entryIn(200 * SEC));
  core::onWake(s, wakeIn(Reset::DeepSleep, 500 * SEC, flatBurst(1950)));  // P+ 3900.0 mV, 300 s after the entry
  core::mergeUiSeed(s, 700);
  CHECK(core::stateOf(s) == State::Ready);
  CHECK(core::early(s, EarlyKind::Ghost, 410, MUL));
  CHECK(core::stateOf(s) == State::Entry && s.gN == 1 && s.gMs == 410);
  CHECK(s.gMv == 39000 && s.gT == 300000);
  CHECK(s.wRtcUs == 0 && !core::taken(s.wBurst) && s.wUi == core::ABSENT16 && (s.flags & recflag::U_LATCHED) == 0);
  CHECK(s.eRtcUs == 200 * SEC && core::valid(s.eBurst));  // the entry half is untouched
  // The next real wake makes the row, and the early sample is in it.
  CHECK(core::early(s, EarlyKind::Cutoff, 390, MUL) && s.gO == 1 && s.gMs == 800);
  core::onWake(s, wakeIn(Reset::DeepSleep, 900 * SEC, flatBurst(1990)));
  const Text t = rowText(s, rowCtx());
  CHECK(crcOk(t) && fieldIs(t, "g_n", "1") && fieldIs(t, "g_o", "1") && fieldIs(t, "g_ms", "800"));
  CHECK(fieldIs(t, "g_mv", "3900.0") && fieldIs(t, "g_t", "300000") && fieldIs(t, "w_mv", "3980.0"));
  CHECK(fieldIs(t, "sleep_ms", "700000"));
  // Ready (S) whose wake took no burst (an aborted sleep): demoted with no sample.
  Record a = idleRecord(7, 100 * SEC);
  core::tail(a, entryIn(200 * SEC));
  core::onWake(a, wakeIn(Reset::Software, 210 * SEC));
  CHECK(core::early(a, EarlyKind::Ghost, 100, MUL) && a.gMv == 0 && a.gT == core::ABSENT32);
  // Ready (B): nothing, an orphan is not a cycle.
  Record b = core::freshRecord();
  core::onWake(b, wakeIn(Reset::Other, 5 * SEC));
  CHECK(core::pendingOrphan(b) && !core::early(b, EarlyKind::Ghost, 100, MUL) && core::pendingOrphan(b));
  // Counters stop at their maximum.
  Record sat = idleRecord();
  core::tail(sat, entryIn(2000 * SEC));
  sat.gN = 255;
  sat.gMs = 0xFFFFFFF0u;
  core::early(sat, EarlyKind::Ghost, 100, MUL);
  CHECK(sat.gN == 255 && sat.gMs == 0xFFFFFFFFu);
  return 0;
}
static_assert(earlyStory() == 0, "early re-sleeps: fold, demote, nothing on Idle, Lost and orphans");

// --- A story: entry, ghost, ghost, wake, flush, and the next cycle ---------------------------------------------------
constexpr int fullStory() {
  Record r = idleRecord(1, 100 * SEC);
  core::tail(r, entryIn(160 * SEC));
  CHECK(core::stateOf(r) == State::Entry && r.awakeMs == 60000 && r.eFl == 0);
  core::early(r, EarlyKind::Ghost, 250, MUL);
  core::early(r, EarlyKind::Ghost, 270, MUL);
  CHECK(r.gN == 2 && r.gMs == 520);
  const Ledger run = core::onWake(r, wakeIn(Reset::DeepSleep, 3760 * SEC));
  CHECK(core::stateOf(r) == State::Ready && run.boots == 1 && r.carry.boots == 1);
  CHECK(core::pendingReady(r) && !core::pendingOrphan(r) && !core::skipThisWake(r));
  Text t = rowText(r, rowCtx());
  CHECK(crcOk(t) && fieldIs(t, "row", "1") && fieldIs(t, "k", "S") && fieldIs(t, "sleep_ms", "3600000"));
  CHECK(fieldIs(t, "awake_ms", "60000") && fieldIs(t, "rtc_ms", "3760000") && fieldIs(t, "utc", ""));
  CHECK(fieldIs(t, "rst", "8") && fieldIs(t, "wk", "h") && fieldIs(t, "w_dt", "300") && fieldIs(t, "fl_w", "0000"));
  CHECK(fieldIs(t, "ab", "0") && fieldIs(t, "ab_r", "") && fieldIs(t, "drop", "0") && fieldIs(t, "sdw_ms", ""));
  core::flushed(r, true, 33);
  CHECK(core::stateOf(r) == State::Idle && r.nextRow == 2 && r.sdwMs == 33);
  // The next entry takes its awake time from this wake, and the next row carries the flush time.
  core::tail(r, entryIn(3760 * SEC + 90 * SEC));
  CHECK(r.awakeMs == 90000 && r.drop == 0 && r.nextRow == 2);
  core::onWake(r, wakeIn(Reset::DeepSleep, 3760 * SEC + 90 * SEC + 7200 * SEC));
  t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "row", "2") && fieldIs(t, "sleep_ms", "7200000") && fieldIs(t, "awake_ms", "90000"));
  CHECK(fieldIs(t, "sdw_ms", "33") && fieldIs(t, "g_n", "0"));
  return 0;
}
static_assert(fullStory() == 0, "entry, ghost, ghost, wake, flush, and the next cycle");

// --- A Ready row followed by a silent restart -----------------------------------------------------------------------
constexpr int restartStory() {
  Record r = idleRecord(10, 100 * SEC);
  core::tail(r, entryIn(200 * SEC));
  core::onWake(r, wakeIn(Reset::DeepSleep, 5000 * SEC));
  CHECK(core::mergeUiSeed(r, 819));
  const uint64_t wRtc = r.wRtcUs;
  const Burst wBurst = r.wBurst;
  // The session so far, carried by the restart funnel.
  Ledger session = core::newLedger();
  core::tick(session, 5000, true, 30, false, true, false);
  core::noteInput(session);
  core::carryLedger(r, core::compact(session));
  CHECK((r.flags & recflag::CARRIED) != 0);
  // The restart boot: the Ready row keeps its wake half, the sleep interval and w_ui; boots counts; the ledger is back.
  Ledger run = core::onWake(r, wakeIn(Reset::Software, 5002 * SEC));
  CHECK(core::stateOf(r) == State::Ready && r.wRtcUs == wRtc && r.wBurst.sum == wBurst.sum && r.wUi == 819);
  CHECK(run.boots == 2 && r.carry.boots == 2 && run.litMs == 5000 && run.litPctMs == 5000u * 30);
  CHECK(run.fullMs == 5000 && run.inp == 1);
  CHECK((r.flags & recflag::CARRIED) == 0 && (r.flags & recflag::RUN_PARTIAL) == 0 && r.ab == 0);
  CHECK(!core::mergeUiSeed(r, 777) && r.wUi == 819);  // the new boot's first seed does not overwrite it
  // A second silent restart with the ledger carried again: three boots.
  core::tick(run, 1000, false, 0, true, false, false);
  core::carryLedger(r, core::compact(run));
  run = core::onWake(r, wakeIn(Reset::Software, 5004 * SEC));
  CHECK(run.boots == 3 && run.wifiMs == 1000 && run.litMs == 5000);
  // The Ready row, flushed after the restarts, is as it was at the wake.
  const Text t = rowText(r, rowCtx());
  CHECK(crcOk(t) && fieldIs(t, "row", "10") && fieldIs(t, "sleep_ms", "4800000") && fieldIs(t, "rtc_ms", "5000000"));
  CHECK(fieldIs(t, "w_ui", "819") && fieldIs(t, "w_dt", "300") && fieldIs(t, "rst", "8") && fieldIs(t, "ab", "0"));
  core::flushed(r, true, 40);
  // The running session goes on: its entry shows three boots, the carried time and no partial flag.
  core::tick(run, 2000, false, 0, false, true, false);
  EntryIn in = entryIn(6000 * SEC);
  in.ledger = core::compact(run);
  core::tail(r, in);
  CHECK(r.ended.boots == 3 && r.ended.litMs == 5000 && r.ended.litPct == 30 && r.ended.fullMs == 7000);
  CHECK((r.eFl & entryflag::LEDGER_PARTIAL) == 0 && r.awakeMs == 1000000);  // 6000 s minus the wake at 5000 s
  return 0;
}
static_assert(restartStory() == 0, "a Ready row and a silent restart: the wake half survives, boots counts");

// A restart with nothing carried, a panic or a watchdog: counted, with its reason, and the ledger partial.
constexpr int uncarriedStory() {
  Record r = idleRecord(3, 100 * SEC);
  core::tail(r, entryIn(200 * SEC));
  core::onWake(r, wakeIn(Reset::DeepSleep, 300 * SEC));
  core::flushed(r, true, 20);
  Ledger run = core::onWake(r, wakeIn(Reset::Software, 310 * SEC));  // Idle, restart, nothing carried
  CHECK(core::stateOf(r) == State::Idle && run.boots == 2 && run.litMs == 0 && (r.flags & recflag::RUN_PARTIAL) != 0);
  CHECK(r.ab == 0 && r.nextRow == 4);
  WakeIn panic = wakeIn(Reset::Abnormal, 320 * SEC);
  panic.rst = 4;
  run = core::onWake(r, panic);
  CHECK(run.boots == 3 && r.ab == 1 && r.abR == 4);
  WakeIn wdt = wakeIn(Reset::Abnormal, 330 * SEC);
  wdt.rst = 7;
  run = core::onWake(r, wdt);
  CHECK(run.boots == 4 && r.ab == 2 && r.abR == 7);
  // A panic after a carry restores the ledger but stays partial.
  Ledger session = core::newLedger(4);
  core::tick(session, 900, false, 0, true, false, false);
  core::carryLedger(r, core::compact(session));
  run = core::onWake(r, panic);
  CHECK(run.boots == 5 && run.wifiMs == 900 && (r.flags & recflag::RUN_PARTIAL) != 0 && r.ab == 3);
  // The entry shows the partial ledger; the abnormal resets count from one row to the next, so they reach this row.
  EntryIn in = entryIn(400 * SEC);
  in.ledger = core::compact(run);
  core::tail(r, in);
  CHECK((r.eFl & entryflag::LEDGER_PARTIAL) != 0 && r.ab == 3 && r.abR == 4);
  core::onWake(r, wakeIn(Reset::DeepSleep, 500 * SEC));
  const Text t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "ab", "3") && fieldIs(t, "ab_r", "4") && fieldIs(t, "fl_e", "0080") && fieldIs(t, "boots", "5"));
  core::flushed(r, true, 20);
  CHECK(r.ab == 0 && r.abR == 0);
  return 0;
}
static_assert(uncarriedStory() == 0, "restarts with no carried ledger, panics and watchdogs: partial, counted, kept");

// A crash count goes with the row that carried it to the card: a write that failed (a slow, full or capped card, where
// crashes tend to occur too) leaves it for the next row.
constexpr int lostRowCrashStory() {
  Record r = idleRecord(3, 100 * SEC);
  core::tail(r, entryIn(200 * SEC));
  core::onWake(r, wakeIn(Reset::DeepSleep, 300 * SEC));
  WakeIn panic = wakeIn(Reset::Abnormal, 310 * SEC);
  panic.rst = 4;
  core::onWake(r, panic);  // a panic before the first flush: the row is still Ready
  CHECK(core::stateOf(r) == State::Ready && r.ab == 1 && r.abR == 4);
  core::flushed(r, false, 40);  // the append failed
  CHECK(core::stateOf(r) == State::Idle && r.drop == 1 && r.ab == 1 && r.abR == 4);
  core::tail(r, entryIn(400 * SEC));
  CHECK(r.ab == 1 && r.abR == 4);
  core::onWake(r, wakeIn(Reset::DeepSleep, 500 * SEC));
  const Text t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "ab", "1") && fieldIs(t, "ab_r", "4") && fieldIs(t, "drop", "1"));
  core::flushed(r, true, 20);  // this one reached the card
  CHECK(r.ab == 0 && r.abR == 0);
  // A row dropped by the next entry keeps them too.
  r.ab = 2;
  r.abR = 7;
  cycle(r, 600 * SEC, 700 * SEC);
  cycle(r, 800 * SEC, 900 * SEC);
  CHECK(r.drop == 1 && r.ab == 2 && r.abR == 7);
  return 0;
}
static_assert(lostRowCrashStory() == 0, "a row that never reached the card leaves its crash count for the next one");

// --- An entry staged on each record state -------------------------------------------------------------------------
constexpr int entryOnEachStateStory() {
  // Lost: a fresh record. The session was not seen from its start.
  Record lost = core::freshRecord();
  core::tail(lost, entryIn(50 * SEC));
  CHECK(core::stateOf(lost) == State::Entry && lost.awakeMs == core::ABSENT32 && lost.nextRow == 1 && lost.drop == 0);
  CHECK((lost.eFl & entryflag::LEDGER_PARTIAL) != 0 && lost.eRtcUs == 50 * SEC);
  // Idle: the awake time comes from the last wake stamp.
  Record idle = idleRecord(9, 20 * SEC);
  core::tail(idle, entryIn(50 * SEC));
  CHECK(core::stateOf(idle) == State::Entry && idle.awakeMs == 30000 && idle.nextRow == 9 && idle.drop == 0);
  CHECK(idle.eFl == 0 && idle.wRtcUs == 0);
  // Entry: a second entry with no wake between. The wake stamp was cleared, so the awake time is unknown.
  core::tail(idle, entryIn(60 * SEC));
  CHECK(core::stateOf(idle) == State::Entry && idle.awakeMs == core::ABSENT32 && idle.nextRow == 9 && idle.drop == 0);
  // Ready (S): the unflushed row is counted lost and its number consumed.
  Record ready = idleRecord(9, 20 * SEC);
  cycle(ready, 50 * SEC, 80 * SEC);
  CHECK(core::pendingReady(ready) && ready.drop == 0 && ready.nextRow == 9);
  core::tail(ready, entryIn(100 * SEC));
  CHECK(core::stateOf(ready) == State::Entry && ready.drop == 1 && ready.nextRow == 10 && ready.awakeMs == 20000);
  CHECK(ready.wRtcUs == 0 && !core::taken(ready.wBurst) && ready.gN == 0);
  // Ready (B): the same.
  Record orphan = core::freshRecord();
  core::onWake(orphan, wakeIn(Reset::Other, 5 * SEC));
  CHECK(core::pendingOrphan(orphan) && orphan.nextRow == 1);
  core::tail(orphan, entryIn(65 * SEC));
  CHECK(core::stateOf(orphan) == State::Entry && orphan.drop == 1 && orphan.nextRow == 2 && orphan.awakeMs == 60000);
  CHECK((orphan.flags & recflag::KIND_B) == 0);
  // A counter that went backwards: the awake time is unknown and the ledger partial.
  Record back = idleRecord(9, 500 * SEC);
  core::tail(back, entryIn(100 * SEC));
  CHECK(back.awakeMs == core::ABSENT32 && (back.eFl & entryflag::LEDGER_PARTIAL) != 0);
  // The SD-error boot: the settings were never loaded, so the row has no cfg and tmo, and says why.
  Record sd = idleRecord(9, 20 * SEC);
  EntryIn in = entryIn(50 * SEC);
  in.settingsLoaded = false;
  in.path = static_cast<uint8_t>(core::Path::SdError);
  core::tail(sd, in);
  core::onWake(sd, wakeIn(Reset::DeepSleep, 90 * SEC));
  const Text t = rowText(sd, rowCtx());
  CHECK(crcOk(t) && fieldIs(t, "cfg", "") && fieldIs(t, "tmo", "") && fieldIs(t, "fl_e", "0200"));
  CHECK(fieldIs(t, "path", "e"));
  return 0;
}
static_assert(entryOnEachStateStory() == 0, "an entry staged on Lost, Idle, Entry, Ready and an orphan");

// --- onWake on every state and reset ----------------------------------------------------------------------------
constexpr int wakeTableStory() {
  // Entry + deep sleep, wake cause not the power key: flagged. And a counter that went backwards, and a missing stamp.
  Record r = idleRecord(4, 10 * SEC);
  core::tail(r, entryIn(20 * SEC));
  WakeIn noExt = wakeIn(Reset::DeepSleep, 120 * SEC);
  noExt.extWake = false;
  noExt.wk = 'o';
  core::onWake(r, noExt);
  Text t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "fl_w", "0020") && fieldIs(t, "wk", "o") && fieldIs(t, "sleep_ms", "100000"));
  Record back = idleRecord(4, 10 * SEC);
  core::tail(back, entryIn(500 * SEC));
  core::onWake(back, wakeIn(Reset::DeepSleep, 100 * SEC));
  t = rowText(back, rowCtx());
  CHECK(fieldIs(t, "fl_w", "0002") && fieldIs(t, "sleep_ms", "") && fieldIs(t, "rtc_ms", "100000"));
  CHECK(fieldIs(t, "w_mv", "4000.0"));  // the burst itself is fine
  Record noStamp = idleRecord(4, 10 * SEC);
  core::tail(noStamp, entryIn(0));
  core::onWake(noStamp, wakeIn(Reset::DeepSleep, 100 * SEC));
  t = rowText(noStamp, rowCtx());
  CHECK(fieldIs(t, "fl_w", "0000") && fieldIs(t, "sleep_ms", ""));

  // Entry + any other reset: an aborted sleep, or a crash inside it. No burst, no interval, no new-session partial.
  const Reset others[3] = {Reset::Software, Reset::Abnormal, Reset::Other};
  for (const Reset reset : others) {
    Record o = idleRecord(4, 10 * SEC);
    core::tail(o, entryIn(20 * SEC));
    WakeIn w = wakeIn(reset, 25 * SEC);
    w.rst = reset == Reset::Abnormal ? 5 : (reset == Reset::Software ? 3 : 1);
    const Ledger run = core::onWake(o, w);
    CHECK(core::stateOf(o) == State::Ready && !core::pendingOrphan(o));
    CHECK(run.boots == 1 && (o.flags & recflag::RUN_PARTIAL) == 0 && o.carry.boots == 1);
    t = rowText(o, rowCtx());
    CHECK(crcOk(t) && fieldIs(t, "k", "S") && fieldIs(t, "sleep_ms", "") && fieldIs(t, "fl_w", "0014"));
    CHECK(fieldIs(t, "w_mv", "") && fieldIs(t, "w_z", "") && fieldIs(t, "w_dt", "") && fieldIs(t, "w_ui", ""));
    CHECK(fieldIs(t, "rtc_ms", "25000") && fieldIs(t, "awake_ms", "10000") && fieldIs(t, "e_mv", "3800.0"));
    const bool crashed = reset == Reset::Abnormal;
    CHECK(fieldIs(t, "ab", crashed ? "1" : "0") && fieldIs(t, "ab_r", crashed ? "5" : ""));
  }

  // Idle or Ready + a reset that is not a deep-sleep wake: a restart of the running session, state kept.
  Record idleOther = idleRecord(4, 10 * SEC);
  const Ledger fromOther = core::onWake(idleOther, wakeIn(Reset::Other, 20 * SEC));
  CHECK(core::stateOf(idleOther) == State::Idle && fromOther.boots == 2);
  CHECK((idleOther.flags & recflag::RUN_PARTIAL) != 0);
  Record readyPanic = idleRecord(4, 10 * SEC);
  cycle(readyPanic, 20 * SEC, 90 * SEC);
  WakeIn panic = wakeIn(Reset::Abnormal, 91 * SEC);
  panic.rst = 9;
  core::onWake(readyPanic, panic);
  CHECK(core::stateOf(readyPanic) == State::Ready && readyPanic.ab == 1 && readyPanic.abR == 9);
  t = rowText(readyPanic, rowCtx());  // the panic came after the wake and before the flush: it is in this row
  CHECK(fieldIs(t, "ab", "1") && fieldIs(t, "ab_r", "9") && fieldIs(t, "rst", "8") && fieldIs(t, "sleep_ms", "70000"));

  // Ready + deep sleep: nothing ended the cycle. The old row is counted lost and the wake is an orphan, with the
  // burst it took, none of the entry half and none of the early folds.
  Record lostRow = idleRecord(4, 10 * SEC);
  core::tail(lostRow, entryIn(20 * SEC));
  core::early(lostRow, EarlyKind::Ghost, 250, MUL);
  core::onWake(lostRow, wakeIn(Reset::DeepSleep, 90 * SEC));
  core::onWake(lostRow, wakeIn(Reset::DeepSleep, 200 * SEC, flatBurst(1970)));
  CHECK(core::pendingOrphan(lostRow) && lostRow.drop == 1 && lostRow.nextRow == 5);
  CHECK(lostRow.eRtcUs == 0 && !core::taken(lostRow.eBurst) && lostRow.ended.litMs == 0 && lostRow.gN == 0);
  t = rowText(lostRow, rowCtx());
  CHECK(crcOk(t) && fieldIs(t, "row", "5") && fieldIs(t, "k", "B") && fieldIs(t, "fl_w", "0001"));
  CHECK(fieldIs(t, "w_mv", "3940.0") && fieldIs(t, "sleep_ms", "") && fieldIs(t, "e_mv", ""));
  CHECK(fieldIs(t, "drop", "1"));

  // Idle + deep sleep: a sleep nothing recorded. An orphan, and nothing was lost.
  Record idleDeep = idleRecord(4, 10 * SEC);
  core::onWake(idleDeep, wakeIn(Reset::DeepSleep, 90 * SEC));
  CHECK(core::pendingOrphan(idleDeep) && idleDeep.drop == 0 && idleDeep.nextRow == 4);

  // Lost + every reset: an orphan, row 1. A restart reason means the session began before the record did.
  const Reset all[4] = {Reset::DeepSleep, Reset::Software, Reset::Abnormal, Reset::Other};
  for (const Reset reset : all) {
    Record l = core::freshRecord();
    const Ledger run = core::onWake(l, wakeIn(reset, 7 * SEC));
    const bool restart = reset == Reset::Software || reset == Reset::Abnormal;
    CHECK(core::pendingOrphan(l) && l.nextRow == 1 && l.drop == 0 && run.boots == 1 && l.carry.boots == 1);
    CHECK(((l.flags & recflag::RUN_PARTIAL) != 0) == restart);
    CHECK(l.ab == (reset == Reset::Abnormal ? 1 : 0));
  }
  return 0;
}
static_assert(wakeTableStory() == 0, "onWake: every state against every kind of reset");

// An orphan boot after a power-on: only what the boot knows is in the row.
constexpr int orphanRowStory() {
  Record r = core::freshRecord();
  WakeIn w = wakeIn(Reset::Other, 5230000);
  w.rst = 1;
  const Ledger run = core::onWake(r, w);
  CHECK(core::pendingOrphan(r) && run.boots == 1 && (r.flags & recflag::RUN_PARTIAL) == 0);
  const Text t = rowText(r, rowCtx());
  CHECK(crcOk(t) && commas(t) == 52);
  CHECK(fieldIs(t, "v", "1") && fieldIs(t, "row", "1") && fieldIs(t, "k", "B") && fieldIs(t, "fw", FW));
  CHECK(fieldIs(t, "m", "1") && fieldIs(t, "rtc_ms", "5230") && fieldIs(t, "usb_w", "0") && fieldIs(t, "chg_w", "b"));
  CHECK(fieldIs(t, "fl_w", "0015") && fieldIs(t, "rst", "1") && fieldIs(t, "wk", "o") && fieldIs(t, "ab", "0"));
  CHECK(fieldIs(t, "drop", "0"));
  const char* const empty[] = {"utc",    "sleep_ms", "awake_ms", "path",    "e_mv",    "e_sd",   "e_rg",    "e_z",
                               "e_raw",  "e_ui",     "usb_e",   "chg_e",   "fl_e",    "ent_ms", "g_n",     "g_o",
                               "g_ms",   "g_mv",     "g_t",     "w_mv",    "w_sd",    "w_rg",   "w_z",     "w_raw",
                               "w_ui",   "w_dt",     "ab_r",    "cfg",     "tmo",     "idle_ms", "boots",  "lit_ms",
                               "lit_pct", "wifi_ms", "full_ms", "inp",     "rf",      "usb_ms", "sdw_ms"};
  for (const char* name : empty) CHECK(fieldIs(t, name, ""));
  // The flush ends the orphan: Idle, its row number consumed, and the next entry is an ordinary one.
  Record f = r;
  core::flushed(f, true, 30);
  CHECK(core::stateOf(f) == State::Idle && f.nextRow == 2 && (f.flags & recflag::KIND_B) == 0);
  core::tail(f, entryIn(65 * SEC));
  CHECK(f.awakeMs == 59770 && f.drop == 0);  // from the orphan's wake at 5.23 s
  return 0;
}
static_assert(orphanRowStory() == 0, "an orphan row after a power-on");

// --- The clock: utc is empty until a sync -----------------------------------------------------------------------
constexpr int utcStory() {
  Record r = core::freshRecord();
  core::onWake(r, wakeIn(Reset::Other, 5 * SEC));
  core::flushed(r, true, 20);
  // time() reads a plausible 2026 (CrossPoint's restored floor, say) but no sync happened in this domain: not written.
  WakeIn w = wakeIn(Reset::DeepSleep, 500 * SEC);
  w.utc = 1791234567;
  core::tail(r, entryIn(100 * SEC));
  core::onWake(r, w);
  Text t = rowText(r, rowCtx());
  CHECK(r.wUtc == 0 && fieldIs(t, "utc", ""));
  // The sync happens during this session: the row of the boot that did it still has no utc (the wake was before it).
  core::noteClockSynced(r);
  CHECK((r.flags & recflag::CLOCK_SYNCED) != 0 && fieldIs(rowText(r, rowCtx()), "utc", ""));
  core::flushed(r, true, 20);
  // The next wake has it, written as the clock read.
  core::tail(r, entryIn(600 * SEC));
  w = wakeIn(Reset::DeepSleep, 900 * SEC);
  w.utc = 1791234567;
  core::onWake(r, w);
  t = rowText(r, rowCtx());
  CHECK(r.wUtc == 1791234567 && fieldIs(t, "utc", "1791234567") && crcOk(t));
  // A synced flag does not make a time before 2025-01-01 a date: only the floor and later are written.
  core::flushed(r, true, 20);
  core::tail(r, entryIn(1000 * SEC));
  w = wakeIn(Reset::DeepSleep, 1300 * SEC);
  w.utc = core::UTC_FLOOR - 1;
  core::onWake(r, w);
  CHECK(r.wUtc == 0 && fieldIs(rowText(r, rowCtx()), "utc", ""));
  core::flushed(r, true, 20);
  core::tail(r, entryIn(1400 * SEC));
  w = wakeIn(Reset::DeepSleep, 1700 * SEC);
  w.utc = core::UTC_FLOOR;
  core::onWake(r, w);
  CHECK(fieldIs(rowText(r, rowCtx()), "utc", "1735689600"));
  // A restart keeps the captured utc; it is not read again.
  WakeIn restart = wakeIn(Reset::Software, 1701 * SEC);
  restart.utc = 1735690000;
  core::onWake(r, restart);
  CHECK(r.wUtc == 1735689600);
  return 0;
}
static_assert(utcStory() == 0, "utc stays empty until the clock-synced flag is set, and below the 2025 floor");

// --- Flag bits: lock timeout, no pin, no valid conversion, staged flags, the file cap ------------------------
constexpr int flagBitsStory() {
  // E skipped on the ADC lock: bits 8 and 5, and the burst columns are empty.
  Record r = idleRecord(1, 10 * SEC);
  EntryIn in = entryIn(20 * SEC, Burst{});
  in.lockTimeout = true;
  core::tail(r, in);
  CHECK(r.eFl == (entryflag::LOCK_TIMEOUT | entryflag::NO_VALID) && r.eFl == 0x0120);
  WakeIn w = wakeIn(Reset::DeepSleep, 90 * SEC, Burst{});
  w.lockTimeout = true;
  core::onWake(r, w);
  CHECK(r.wFl == (wakeflag::LOCK_TIMEOUT | wakeflag::NO_VALID) && r.wFl == 0x0090);
  RowCtx ctx = rowCtx();
  Text t = rowText(r, ctx);
  CHECK(fieldIs(t, "fl_e", "0120") && fieldIs(t, "fl_w", "0090") && crcOk(t));
  CHECK(fieldIs(t, "e_mv", "") && fieldIs(t, "e_sd", "") && fieldIs(t, "e_rg", "") && fieldIs(t, "e_z", "") &&
        fieldIs(t, "e_raw", ""));
  CHECK(fieldIs(t, "w_mv", "") && fieldIs(t, "w_z", "") && fieldIs(t, "w_raw", "") && fieldIs(t, "w_dt", ""));
  // The flush adds the charge-ladder bit and, on the last row the file takes, the cap bit.
  ctx.chargeUndecoded = true;
  ctx.capReached = true;
  t = rowText(r, ctx);
  CHECK(fieldIs(t, "fl_w", "0298") && crcOk(t));
  ctx = rowCtx();
  ctx.capReached = true;
  CHECK(fieldIs(rowText(r, ctx), "fl_w", "0290"));
  // No battery ADC pin: bit 6 and 5.
  Record np = idleRecord(1, 10 * SEC);
  in = entryIn(20 * SEC, Burst{});
  in.noAdcPin = true;
  core::tail(np, in);
  CHECK(np.eFl == 0x0060);
  // Zero conversions all 64 times: taken, so the zero count and the raw sum are there, but no mean.
  Record zr = idleRecord(1, 10 * SEC);
  core::tail(zr, entryIn(20 * SEC, zeroBurst()));
  core::onWake(zr, wakeIn(Reset::DeepSleep, 90 * SEC, zeroBurst()));
  t = rowText(zr, rowCtx());
  CHECK(zr.eFl == entryflag::NO_VALID && zr.wFl == wakeflag::NO_VALID);
  CHECK(fieldIs(t, "e_mv", "") && fieldIs(t, "e_z", "64") && fieldIs(t, "e_raw", "1234") && fieldIs(t, "w_z", "64"));
  CHECK(fieldIs(t, "w_raw", "1234") && fieldIs(t, "w_mv", "") && fieldIs(t, "w_dt", "300"));
  // The staged flags pass through (a key held means a release wake is armed); other bits are not the app's to set.
  Record sf = idleRecord(1, 10 * SEC);
  in = entryIn(20 * SEC);
  in.noteFlags = 0xFFFF;
  core::tail(sf, in);
  CHECK(sf.eFl == (entryflag::LIT | entryflag::RADIO_ON | entryflag::TOUCH_NOT_ASLEEP | entryflag::SDA_LOW |
                   entryflag::KEY_HELD));
  CHECK(sf.eFl == 0x001F);
  // The bit values are the design's.
  CHECK(entryflag::LIT == 1 && entryflag::RADIO_ON == 2 && entryflag::TOUCH_NOT_ASLEEP == 4 && entryflag::SDA_LOW == 8);
  CHECK(entryflag::KEY_HELD == 16 && entryflag::NO_VALID == 32 && entryflag::NO_ADC_PIN == 64);
  CHECK(entryflag::LEDGER_PARTIAL == 128 && entryflag::LOCK_TIMEOUT == 256 && entryflag::SETTINGS_NOT_LOADED == 512);
  CHECK(wakeflag::ORPHAN == 1 && wakeflag::RTC_BACKWARDS == 2 && wakeflag::NOT_DEEP_SLEEP == 4);
  CHECK(wakeflag::CHARGE_UNDECODED == 8 && wakeflag::NO_VALID == 16 && wakeflag::NOT_EXT1 == 32);
  CHECK(wakeflag::PREV_FLUSH_BAD == 64 && wakeflag::LOCK_TIMEOUT == 128 && wakeflag::BACKOFF_SKIPPED == 256);
  CHECK(wakeflag::FILE_CAP == 512);
  return 0;
}
static_assert(flagBitsStory() == 0, "the lock-timeout, no-pin, no-valid, staged and file-cap flag bits");

// --- An unflushed row, a failed write ---------------------------------------------------------------------------
constexpr int dropStory() {
  Record r = idleRecord(20, 10 * SEC);
  cycle(r, 100 * SEC, 200 * SEC);  // row 20 is Ready and never flushed
  cycle(r, 300 * SEC, 400 * SEC);  // the next entry counts it lost; row 21 is Ready
  CHECK(r.drop == 1 && r.nextRow == 21);
  Text t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "row", "21") && fieldIs(t, "drop", "1"));  // the gap in `row` and `drop` say so
  core::flushed(r, true, 25);
  CHECK(r.drop == 0 && r.nextRow == 22);
  // A write that failed: the row is lost, the number consumed, and the next row says the previous flush was bad.
  cycle(r, 500 * SEC, 600 * SEC);
  core::flushed(r, false, 40);
  CHECK(core::stateOf(r) == State::Idle && r.nextRow == 23 && r.drop == 1 && r.sdBad == 1 && r.sdSkip == 0);
  CHECK((r.flags & recflag::PREV_BAD) != 0 && r.sdwMs == 40);
  cycle(r, 700 * SEC, 800 * SEC);
  t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "row", "23") && fieldIs(t, "drop", "1") && fieldIs(t, "fl_w", "0040") && fieldIs(t, "sdw_ms", "40"));
  core::flushed(r, true, 20);
  CHECK(r.drop == 0 && r.sdBad == 0 && (r.flags & recflag::PREV_BAD) == 0);
  // Written but slow (over 250 ms) is a bad attempt that still reached the card.
  cycle(r, 900 * SEC, 1000 * SEC);
  core::flushed(r, true, 300);
  CHECK(r.drop == 0 && r.sdBad == 1 && (r.flags & recflag::PREV_BAD) != 0 && r.nextRow == 25);
  cycle(r, 1100 * SEC, 1200 * SEC);
  core::flushed(r, true, 250);  // exactly 250 ms is still good
  CHECK(r.sdBad == 0 && (r.flags & recflag::PREV_BAD) == 0);
  // The duration saturates below the "none yet" value.
  cycle(r, 1300 * SEC, 1400 * SEC);
  core::flushed(r, true, 100000);
  CHECK(r.sdwMs == 0xFFFE);
  // flushed() on a record that is not Ready does nothing.
  Record idle = idleRecord(5);
  core::flushed(idle, false, 999);
  CHECK(idle.nextRow == 5 && idle.sdBad == 0 && idle.drop == 0);
  return 0;
}
static_assert(dropStory() == 0, "an unflushed row, a failed write and a slow one");

// --- The SD back-off: bad, bad, 63 skipped wakes, a probe -----------------------------------------------------------
constexpr void nextCycle(Record& r, uint64_t& at) {
  cycle(r, at + 60 * SEC, at + 3660 * SEC);
  at += 3660 * SEC;
}

constexpr int backoffStory() {
  Record r = idleRecord(1, 100 * SEC);
  uint64_t at = 1000 * SEC;
  // Two bad attempts in a row.
  nextCycle(r, at);
  CHECK(!core::skipThisWake(r));
  core::flushed(r, false, 30);
  CHECK(r.sdBad == 1 && r.sdSkip == 0);
  nextCycle(r, at);
  CHECK(!core::skipThisWake(r) && fieldIs(rowText(r, rowCtx()), "fl_w", "0040"));
  core::flushed(r, false, 30);
  CHECK(r.sdBad == 2 && r.sdSkip == core::SD_SKIP_WAKES && r.sdSkip == 63 && r.nextRow == 3);
  // The next 63 verified wakes skip the flush; each skipped row is counted lost by the next entry.
  for (uint8_t i = 1; i <= 63; ++i) {
    nextCycle(r, at);
    CHECK(core::skipThisWake(r) && r.sdSkip == 63 - i && core::pendingReady(r));
  }
  CHECK(r.sdSkip == 0 && r.sdBad == 2);
  // The wake after that probes.
  nextCycle(r, at);
  CHECK(!core::skipThisWake(r) && r.drop == 65 && r.nextRow == 66);  // two failed rows, 63 skipped
  const Text t = rowText(r, rowCtx());
  CHECK(fieldIs(t, "row", "66") && fieldIs(t, "drop", "65") && fieldIs(t, "fl_w", "0140"));  // skipped + previous bad
  // A good probe clears the count and the skip flags; the next wake is ordinary.
  Record good = r;
  core::flushed(good, true, 30);
  CHECK(good.sdBad == 0 && good.drop == 0 && (good.flags & (recflag::SKIPPED | recflag::PREV_BAD)) == 0);
  uint64_t at2 = at;
  nextCycle(good, at2);
  CHECK(!core::skipThisWake(good) && fieldIs(rowText(good, rowCtx()), "fl_w", "0000"));
  // A bad probe skips another 63 wakes, and the row it could not write is lost.
  Record bad = r;
  core::flushed(bad, false, 30);
  CHECK(bad.sdBad == 3 && bad.sdSkip == 63 && bad.drop == 66 && (bad.flags & recflag::SKIPPED) != 0);
  uint64_t at3 = at;
  nextCycle(bad, at3);
  CHECK(core::skipThisWake(bad) && bad.sdSkip == 62);
  // A probe that wrote the row but was slow (over 250 ms) also skips again, though nothing was lost.
  Record slow = r;
  core::flushed(slow, true, 400);
  CHECK(slow.sdBad == 3 && slow.sdSkip == 63 && slow.drop == 0);
  // One bad attempt between good ones never starts the back-off.
  Record once = idleRecord(1, 100 * SEC);
  uint64_t at4 = 1000 * SEC;
  nextCycle(once, at4);
  core::flushed(once, false, 30);
  nextCycle(once, at4);
  core::flushed(once, true, 30);
  nextCycle(once, at4);
  core::flushed(once, false, 30);
  CHECK(once.sdBad == 1 && once.sdSkip == 0);
  // A silent restart inside a skipped wake stays skipped: only a deep-sleep wake consumes one.
  Record rs = idleRecord(1, 100 * SEC);
  rs.sdSkip = 2;
  uint64_t at5 = 1000 * SEC;
  nextCycle(rs, at5);
  CHECK(core::skipThisWake(rs) && rs.sdSkip == 1);
  core::onWake(rs, wakeIn(Reset::Software, at5 + 2 * SEC));
  CHECK(core::skipThisWake(rs) && rs.sdSkip == 1);
  nextCycle(rs, at5);
  CHECK(core::skipThisWake(rs) && rs.sdSkip == 0);
  nextCycle(rs, at5);
  CHECK(!core::skipThisWake(rs));
  return 0;
}
static_assert(backoffStory() == 0, "the SD back-off: bad, bad, 63 skipped wakes, a good probe and a bad one");

// --- The late U latch, the clock flag, carry ---------------------------------------------------------------------
constexpr int latchStory() {
  Record r = idleRecord(1, 10 * SEC);
  CHECK(!core::mergeUiSeed(r, 800));  // Idle: no row waiting for it
  core::tail(r, entryIn(20 * SEC));
  CHECK(!core::mergeUiSeed(r, 800));  // Entry: nor here
  core::onWake(r, wakeIn(Reset::DeepSleep, 90 * SEC));
  CHECK(r.wUi == core::ABSENT16 && fieldIs(rowText(r, rowCtx()), "w_ui", ""));
  CHECK(core::mergeUiSeed(r, 0) && r.wUi == 0 && fieldIs(rowText(r, rowCtx()), "w_ui", "0"));  // 0.0 % is a value
  CHECK(!core::mergeUiSeed(r, 123) && r.wUi == 0);
  core::flushed(r, true, 20);
  core::tail(r, entryIn(200 * SEC));  // the latch is per row
  core::onWake(r, wakeIn(Reset::DeepSleep, 300 * SEC));
  CHECK(core::mergeUiSeed(r, 555) && r.wUi == 555);
  return 0;
}
static_assert(latchStory() == 0, "w_ui is latched once per row and only while a row waits");

// The state a static_assert cannot see by value: sizes.
static_assert(sizeof(Record) <= 160, "the record's share of the RTC budget");
static_assert(sizeof(core::LedgerRec) == 24 && sizeof(Burst) == 16, "no padding in the bursts");
static_assert(core::ROW_BUFFER_BYTES == 384, "the writer's static row buffer");

#if 0  // NEGATIVE CONTROL: enable this block in a scratch copy; the compile must fail on it. Keep it disabled.
// The example row with its last CRC digit changed (c59d -> c59e): the comparison must reject it.
static_assert(textIs(rowText(exampleRecord(), rowCtx()),
                     "\n1,57,S,f3a9c01d2,1,412345678,1791234567,32412345,1536000,a,3997.0,5.2,24,0,36784,826,0,b,"
                     "0000,2310,2,0,640,,,4006.5,4.7,22,0,36871,818,312,0,b,0000,8,h,0,,0d,10,600000,1,720000,40,0,"
                     "410000,118,,0,41,0,c59e"),
              "NEGATIVE CONTROL: this assertion is wrong on purpose and must fail the compile");
#endif

}  // namespace

int main() { return 0; }
#endif  // SILKSCREEN_BATLOG
