#include "Ssd1677Driver.h"

#include <BoardConfig.h>

#include <algorithm>
#include <vector>

#include "../lut/Ssd1677Luts.h"

namespace freeink {
namespace {

// SSD1677 command set.
constexpr uint8_t CMD_SOFT_RESET = 0x12;
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x0C;
constexpr uint8_t CMD_DRIVER_OUTPUT_CONTROL = 0x01;
constexpr uint8_t CMD_BORDER_WAVEFORM = 0x3C;
constexpr uint8_t CMD_TEMP_SENSOR_CONTROL = 0x18;
constexpr uint8_t CMD_DATA_ENTRY_MODE = 0x11;
constexpr uint8_t CMD_SET_RAM_X_RANGE = 0x44;
constexpr uint8_t CMD_SET_RAM_Y_RANGE = 0x45;
constexpr uint8_t CMD_SET_RAM_X_COUNTER = 0x4E;
constexpr uint8_t CMD_SET_RAM_Y_COUNTER = 0x4F;
constexpr uint8_t CMD_WRITE_RAM_BW = 0x24;
constexpr uint8_t CMD_WRITE_RAM_RED = 0x26;
constexpr uint8_t CMD_AUTO_WRITE_BW_RAM = 0x46;
constexpr uint8_t CMD_AUTO_WRITE_RED_RAM = 0x47;
constexpr uint8_t CMD_DISPLAY_UPDATE_CTRL1 = 0x21;
constexpr uint8_t CMD_DISPLAY_UPDATE_CTRL2 = 0x22;
constexpr uint8_t CMD_MASTER_ACTIVATION = 0x20;
constexpr uint8_t CTRL1_NORMAL = 0x00;
constexpr uint8_t CTRL1_BYPASS_RED = 0x40;
constexpr uint8_t CMD_WRITE_LUT = 0x32;
constexpr uint8_t CMD_GATE_VOLTAGE = 0x03;
constexpr uint8_t CMD_SOURCE_VOLTAGE = 0x04;
constexpr uint8_t CMD_WRITE_VCOM = 0x2C;
constexpr uint8_t CMD_WRITE_TEMP = 0x1A;
constexpr uint8_t CMD_DEEP_SLEEP = 0x10;

constexpr uint8_t DRIVER_OUTPUT_SCAN = 0x02;  // SM=1 interlaced, TB=0 (base)

}  // namespace

const Ssd1677Config& ssd1677DefaultConfig() {
  // Xteink X4 / GDEQ0426T82 defaults. The stock X4 firmware's B/W update paths
  // use absolute SSD1677 sequences rather than the incremental 0x1C assembly:
  // INIT:  0C=AE C7 C3 C0 80, 3C=80
  // FULL:  3C=C0, 22=F7, 20, ~1800 ms
  // HALF:  3C=C0, 1A=5A, 22=D7, 20
  // PART:  3C=C0, 22=FC, 20, ~500 ms
  // Keep those as the default X4 path; weaker partial selection shows heavy
  // ghosting on some panels.
  static const Ssd1677Config cfg = {
      {0xAE, 0xC7, 0xC3, 0xC0, 0x80},  // booster soft-start
      DRIVER_OUTPUT_SCAN,
      0x80,  // borderWaveformInit: stock X4 init border
      0x5A,  // HALF refresh temperature
      lut_grayscale,
      0xF7,  // fullSeqOverride: stock X4 full update sequence
      0xFC,  // fastSeqOverride: stock X4 partial update sequence
      0xD7,  // halfSeqOverride: stock X4 warmed/full-clean update sequence
      0xC0,  // borderWaveformFull: stock X4 border
      0xC0,  // borderWaveformFast: stock X4 border
      0xC0,  // borderWaveformHalf: stock X4 border
      0xC0,  // borderWaveformGray: written explicitly with the external AA LUT (vendor
             // reference stage 2); same value the B/W paths leave in the register, so
             // the wire state is unchanged — just no longer relying on carry-over
  };
  return cfg;
}

// Seeed Sticky. Same SSD1677 controller, resolution, and RAM polarity as the X4
// (Seeed's driver uses "1bpp MSB, 0xFF=white", bit1=white — identical to the SDK
// framebuffer). The one real difference is the waveform: the X4's fast update
// sequence (0x1C) does NOT select this panel's partial/DU waveform — it runs the
// full OTP waveform every refresh (~1.7s, UI unusably slow). Seeed's own SSD1677
// driver uses 0x22 = 0xF7 (full) / 0xFF (partial); those load temperature and
// select the DU waveform. Supplied here as per-board sequences — tune here
// (booster, LUTs, sequences), never in the driver body.
static const Ssd1677Config& ssd1677StickyConfig() {
  static const Ssd1677Config cfg = {
      {0xAE, 0xC7, 0xC3, 0xC0, 0x80},  // booster soft-start (matches Seeed's panel driver)
      DRIVER_OUTPUT_SCAN,
      0x01,  // borderWaveformInit: vendor FULL/partial-clear border
      0x5A,  // halfRefreshTemp (unused once fullSeqOverride loads temperature itself)
      lut_grayscale_sticky,  // own copy: voltage tail is per-module, tune there
                             // (see Ssd1677Luts.h), never in the shared X4 LUT
      0xF7,  // fullSeqOverride: vendor FULL update sequence
      0xFF,  // fastSeqOverride: vendor PARTIAL/DU update sequence (the actual fast path)
      0x00,  // halfSeqOverride: use fullSeqOverride
      0x01,  // borderWaveformFull: vendor FULL/partial-clear border
      0x80,  // borderWaveformFast: vendor PARTIAL/DU border (stops the dark edge ring)
      0x00,  // borderWaveformHalf: use borderWaveformFull
      0x80,  // borderWaveformGray: hold at VCOM; follow-LUT (0x01) drives the border
             // black under the grayscale LUT (black frame on every AA/cover refresh)
      true,  // grayPowerUpFirst: vendor sequences power down after every refresh, so
             // settle the rails before the short gray LUT phases (see Ssd1677Config)
  };
  return cfg;
}

// de-link. Same GDEQ0426T82 panel as the X4, but behind a different flex wrapper,
// and that wrapper does not tolerate the X4's absolute FULL/HALF sequences. Values
// carried over from the board's own SDK fork (iandchasse/community-sdk-de-link),
// where each was arrived at against the hardware:
//
//   * Booster 5th byte 0xC0, not the X4's 0x80. Its author walked 0x40 -> 0x80 ->
//     0xC0, recording "shorter on-time led to weak refreshes on my hardware" and
//     "boost it up, better quality images". An under-driven charge pump shows up
//     exactly where demand is highest: FULL/HALF drive every pixel through a full
//     swing, so they smear and flicker while the short FAST/DU and gray-LUT
//     phases still look clean.
//
//   * FULL and HALF both on the incremental 0x34 path (overrides cleared), which
//     keeps the rails powered between updates instead of self-cycling them per
//     refresh like the X4's 0xF7/0xD7 sequences do. Re-ramping the booster on
//     every full-page refresh is visible flicker on this wrapper.
//
//   * HALF on the full waveform rather than the 0xD4 temperature spoof — the
//     fork's comment: "The FL wrapper's OTP waveform doesn't work well with the
//     temp trick."
//
// FAST keeps the X4's absolute 0xFC sequence: validated good on this board, and
// the incremental 0x1C path is exactly the one freeink warns can silently promote
// a DU refresh to a full waveform.
static const Ssd1677Config& ssd1677DeLinkConfig() {
  static const Ssd1677Config cfg = {
      {0xAE, 0xC7, 0xC3, 0xC0, 0xC0},  // booster soft-start: 5th byte boosted (see above)
      DRIVER_OUTPUT_SCAN,
      0x80,  // borderWaveformInit
      0x5A,  // halfRefreshTemp (unused: halfUsesFullWaveform skips the temp write)
      lut_grayscale,
      0x00,  // fullSeqOverride: 0 -> incremental 0x34, rails stay powered
      0xFC,  // fastSeqOverride: stock X4 partial sequence (validated on this board)
      0x00,  // halfSeqOverride: 0 -> incremental, and halfUsesFullWaveform picks 0x34
      0x00,  // borderWaveformFull: unused on the incremental path
      0xC0,  // borderWaveformFast: stock X4 border
      0x00,  // borderWaveformHalf: unused on the incremental path
      0x00,  // borderWaveformGray
      false, // grayPowerUpFirst: rails stay up between refreshes here
      true,  // halfUsesFullWaveform
  };
  return cfg;
}

// ── Reusable per-board waveform shortcuts ────────────────────────────────────
// Opt-in optimizations a board can layer onto a base Ssd1677Config when its
// specific panel is known to tolerate them. Each is a pure copy-and-tweak so a
// board picks only the shortcuts it has validated — apply like:
//   static const Ssd1677Config cfg = fastDuRefreshShortcut(ssd1677DefaultConfig());
// and add a `case Board::Xxx:` in ssd1677ActiveConfig() returning it.

// Fast-DU refresh shortcut (~77 ms/refresh). A fast BW refresh uses the
// incremental CTRL2=0x1C differential-update path (load LUT + display) instead
// of a 0xFC-style vendor sequence that also reloads temperature and re-cycles
// clock/analog on every refresh. SAFE ONLY on panels that honor 0x1C as a true
// partial update; on panels where 0x1C silently promotes to the FULL waveform
// this is a big regression, so leave fastSeqOverride != 0 there. Validated on
// the Xteink X4 (the CrossPoint community-sdk drives that exact panel with 0x1C).
static Ssd1677Config fastDuRefreshShortcut(Ssd1677Config base) {
  base.fastSeqOverride = 0;  // 0 -> incremental CTRL2=0x1C DU path
  return base;
}

// Xteink X4 with the fast-DU shortcut — OPT-IN via -DFREEINK_X4_FAST_DU_SHORTCUT.
// The X4 default stays the stock 0xFC absolute partial sequence: the 0x1C path
// skips the per-refresh temperature load and power sequencing, which is the
// community-sdk behavior the stock-parity work moved away from after ghosting /
// blotching reports on some panels ("weaker partial selection shows heavy
// ghosting", see ssd1677DefaultConfig). Enable the flag only after validating
// long reading sessions across temperatures on your panel (~85 ms/refresh win).
// A d2-class board that shares this panel can reuse ssd1677X4Config() or build
// its own base and layer the same shortcut(s).
#ifdef FREEINK_X4_FAST_DU_SHORTCUT
static const Ssd1677Config& ssd1677X4Config() {
  static const Ssd1677Config cfg = fastDuRefreshShortcut(ssd1677DefaultConfig());
  return cfg;
}
#endif

Ssd1677Driver::Ssd1677Driver(const Ssd1677Config& cfg)
    : _cfg(cfg),
      _w(BoardConfig::ACTIVE.displayWidth),
      _h(BoardConfig::ACTIVE.displayHeight),
      _wb(BoardConfig::ACTIVE.displayWidth / 8),
      _bufferSize(static_cast<uint32_t>(BoardConfig::ACTIVE.displayWidth / 8) * BoardConfig::ACTIVE.displayHeight),
#if defined(FREEINK_DISPLAY_FLIPPED) || defined(FLIPPED)
      // Legacy build-flag escape hatch: an upside-down mount, expressed as a
      // define instead of BoardProfile.orientation. Maps to the full 180.
      _rot180(true) {}
#else
      _rot180(BoardConfig::ACTIVE.orientation.mirrorX && BoardConfig::ACTIVE.orientation.mirrorY) {}
#endif

uint32_t Ssd1677Driver::spiHz() const {
  return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 40000000;
}

PanelGeometry Ssd1677Driver::geometry() const { return {_w, _h, _wb, _bufferSize}; }

void Ssd1677Driver::begin(EpdBus& bus) {
  bus.reset();
  initController(bus);
}

void Ssd1677Driver::initController(EpdBus& bus) {
  constexpr uint8_t TEMP_SENSOR_INTERNAL = 0x80;

  bus.cmd(CMD_SOFT_RESET);
  bus.waitBusy(" CMD_SOFT_RESET");

  bus.cmd(CMD_TEMP_SENSOR_CONTROL);
  bus.data(TEMP_SENSOR_INTERNAL);

  // Booster soft-start (device-tunable via config).
  bus.cmd(CMD_BOOSTER_SOFT_START);
  for (uint8_t b : _cfg.booster) {
    bus.data(b);
  }

  // Driver output control: display height + scan configuration. The scan byte is
  // NEVER modified for an upside-down mount: on the SSD1677 the TB bit is
  // documented "Option TB = 1 is reserved" (datasheet §8.1) — gate-scan reversal
  // does not exist on this controller, and setting the bit anyway produces
  // undefined scanning (observed on hardware as scrambled rows and unrefreshed
  // regions). The 180° mount is handled entirely in the data plane; see writeRam.
  bus.cmd(CMD_DRIVER_OUTPUT_CONTROL);
  bus.data((_h - 1) % 256);
  bus.data((_h - 1) / 256);
  bus.data(_cfg.driverOutputScan);

  bus.cmd(CMD_BORDER_WAVEFORM);
  bus.data(_cfg.borderWaveformInit);

  setRamArea(bus, 0, 0, _w, _h);

  bus.cmd(CMD_AUTO_WRITE_BW_RAM);
  bus.data(0xF7);
  bus.waitBusy(" CMD_AUTO_WRITE_BW_RAM");

  bus.cmd(CMD_AUTO_WRITE_RED_RAM);
  bus.data(0xF7);
  bus.waitBusy(" CMD_AUTO_WRITE_RED_RAM");

  _isScreenOn = false;
  // Override boards can't use _isScreenOn to detect a cold start (their fast
  // sequence powers down after every page), so arm an explicit one-shot full
  // refresh for the first paint — it clears the boot screen and seeds the baseline.
  _needsInitialFull = (_cfg.fullSeqOverride != 0);
}

void Ssd1677Driver::setRamArea(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // Addressing is ALWAYS stock X-increment / Y-decrement — the transaction shape
  // this panel has proven for every non-mirrored board. Do not be tempted to
  // express a mirror here: the SSD1677's X window registers are pixel-granular
  // (0..0x3BF) while the address counter steps once per data BYTE, and the
  // datasheet does not define that interaction for X-decrement; on hardware it
  // scrambles pixel order and leaves part of the window unwritten. The 180°
  // mount is a data-plane transform instead (see writeRam) — the window itself
  // is placed at mirrored coordinates by the callers that pass sub-frame rects.
  const uint8_t dataEntry = 0x01;  // X increment, Y decrement
  const uint16_t xStart = x;
  const uint16_t xEnd = x + w - 1;

  // Gates are physically reversed on this panel.
  y = _h - y - h;

  bus.cmd(CMD_DATA_ENTRY_MODE);
  bus.data(dataEntry);

  bus.cmd(CMD_SET_RAM_X_RANGE);
  bus.data(xStart % 256);
  bus.data(xStart / 256);
  bus.data(xEnd % 256);
  bus.data(xEnd / 256);

  bus.cmd(CMD_SET_RAM_Y_RANGE);
  bus.data((y + h - 1) % 256);
  bus.data((y + h - 1) / 256);
  bus.data(y % 256);
  bus.data(y / 256);

  bus.cmd(CMD_SET_RAM_X_COUNTER);
  bus.data(xStart % 256);
  bus.data(xStart / 256);

  bus.cmd(CMD_SET_RAM_Y_COUNTER);
  bus.data((y + h - 1) % 256);
  bus.data((y + h - 1) / 256);
}

namespace {
// Bit-reversal table for the 180-degree data-plane rotation. A lookup rather
// than a shift loop because it runs over every byte of every RAM write.
constexpr uint8_t kReverseByte[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48,
    0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4,
    0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C,
    0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2,
    0x32, 0xB2, 0x72, 0xF2, 0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A,
    0xFA, 0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E,
    0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01, 0x81, 0x41, 0xC1, 0x21,
    0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1, 0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9,
    0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55,
    0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD,
    0x7D, 0xFD, 0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 0x0B,
    0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07, 0x87, 0x47, 0xC7,
    0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F,
    0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};

// Chunk size for the mirrored write. Small enough to sit on the stack, large
// enough that the SPI transaction cost is amortised over a bulk transfer.
constexpr uint16_t kMirrorChunk = 256;
}  // namespace

void Ssd1677Driver::writeRam(EpdBus& bus, uint8_t ramCmd, const uint8_t* data, uint32_t size) {
  if (!_rot180) {
    bus.cmd(ramCmd);
    bus.data(data, static_cast<uint16_t>(size));
    return;
  }

  // The 180° mount, done entirely in the data plane. The controller offers no
  // help here — the TB gate-reversal bit is reserved on the SSD1677, and
  // X-decrement addressing is undefined for byte writes against its
  // pixel-granular window registers — so the flip must not involve addressing
  // at all. Instead the buffer is streamed back-to-front with each byte
  // bit-reversed. For a row-major 1bpp buffer that IS an exact 180: reversing
  // the flat byte order reverses both the row order and the byte order within
  // each row (index b = r·W + c maps to (H−1−r)·W + (W−1−c)), and the per-byte
  // bit reversal mirrors the last 8 pixels. The controller sees a completely
  // ordinary X-increment write.
  //
  // This is correct for any buffer whose rows are contiguous at the width the
  // RAM window expects: the full frame, an extracted display window, and a
  // grayscale strip alike (their own local 180 is what must land in the
  // mirrored window position — see the callers for the coordinate half).
  //
  // Every RAM write (BW, RED, grayscale planes, windowed update, baseline
  // resync) funnels through this method, so no path can be missed.
  //
  // The chunks go out inside ONE opened transaction rather than through
  // bus.data() per chunk, which would frame each with its own
  // beginTransaction/CS/endTransaction (see EpdBus::data): ~190 bus arbitrations
  // and CS edges for a 48KB frame instead of one.
  //
  // This is an efficiency measure, NOT a correctness fix. CS toggling mid-write
  // is tolerated here — the address counter lives in the controller and survives
  // it, and the proven unrotated path already splits its command and data across
  // two separate CS frames (EpdBus::cmd then EpdBus::data). It was briefly
  // suspected of causing grey/stuck pixels and flicker on FULL/HALF refreshes;
  // it was not. That was the booster strength and the FULL/HALF waveform
  // selection, fixed in the de-link board config.
  //
  // Safe to hold the bus this long only because the rotated path is per-board
  // and no such board shares this SPI bus with its SD card.
  bus.beginTxn();
  bus.rawCmd(ramCmd);
  uint8_t chunk[kMirrorChunk];
  uint32_t remaining = size;
  while (remaining > 0) {
    const uint16_t n = remaining < kMirrorChunk ? static_cast<uint16_t>(remaining) : kMirrorChunk;
    // Next n source bytes, walking backwards from the tail.
    const uint8_t* src = data + remaining - 1;
    for (uint16_t i = 0; i < n; i++) {
      chunk[i] = kReverseByte[*src--];
    }
    bus.rawWriteBytes(chunk, n);
    remaining -= n;
  }
  bus.endTxn();
}

void Ssd1677Driver::writeRamInverted(EpdBus& bus, uint8_t ramCmd, const uint8_t* data, uint32_t size) {
  uint8_t chunk[128];
  bus.cmd(ramCmd);
  for (uint32_t offset = 0; offset < size; offset += sizeof(chunk)) {
    const uint32_t n = std::min<uint32_t>(sizeof(chunk), size - offset);
    for (uint32_t i = 0; i < n; ++i) chunk[i] = static_cast<uint8_t>(~data[offset + i]);
    bus.data(chunk, static_cast<uint16_t>(n));
  }
}

void Ssd1677Driver::refresh(EpdBus& bus, RefreshMode mode, bool turnOff, bool async) {
#if defined(SSD1677_PROBE_DEBUG) && SSD1677_PROBE_DEBUG
  const uint32_t dbgStart = millis();
  const char* dbgMode = (mode == RefreshMode::Full) ? "FULL" : (mode == RefreshMode::Half) ? "HALF" : "FAST";
#endif
  bus.cmd(CMD_DISPLAY_UPDATE_CTRL1);
  bus.data((mode == RefreshMode::Fast) ? CTRL1_NORMAL : CTRL1_BYPASS_RED);

  // Per-board absolute update sequence (vendor 0x22 values). When set, it selects
  // the panel's waveform directly — including load-temperature and the partial/DU
  // display mode — and self-cycles power. The X4's incremental bit assembly below
  // doesn't trigger some panels' DU waveform (they then run the full waveform on
  // every "fast" refresh); these values fix that. Skipped while a custom grayscale
  // LUT is active (that path needs the 0x0C sequence with the loaded LUT).
  const uint8_t seqOverride = (mode == RefreshMode::Fast) ? _cfg.fastSeqOverride
                              : (mode == RefreshMode::Half && _cfg.halfSeqOverride != 0)
                                  ? _cfg.halfSeqOverride
                                  : _cfg.fullSeqOverride;
  if (seqOverride != 0 && !_customLutActive) {
    // Track the border waveform to the refresh mode (vendor parity): a partial/DU
    // (fast) refresh leaves the border driven dark if it keeps the full-refresh
    // border, producing a black ring around the page. 0 = leave the init value.
    const uint8_t border = (mode == RefreshMode::Fast) ? _cfg.borderWaveformFast
                           : (mode == RefreshMode::Half && _cfg.borderWaveformHalf != 0)
                               ? _cfg.borderWaveformHalf
                               : _cfg.borderWaveformFull;
    if (border != 0) {
      bus.cmd(CMD_BORDER_WAVEFORM);
      bus.data(border);
    }
    if (mode == RefreshMode::Half) {
      bus.cmd(CMD_WRITE_TEMP);
      bus.data(_cfg.halfRefreshTemp);
    }
    bus.cmd(CMD_DISPLAY_UPDATE_CTRL2);
    bus.data(seqOverride);
    bus.cmd(CMD_MASTER_ACTIVATION);
    if (!async) bus.waitRefreshComplete("refresh");
    // The sequence powered the panel down at the end, but keep the flag truthful
    // to intent: leave it "on" between active updates so display() doesn't force a
    // full HALF refresh next time (which would defeat fast refresh). turnOff marks
    // it off for the sleep path. The vendor sequences self-cycle power: if they
    // include the disable bits (0x03) the panel is OFF afterward — track that so the
    // next refresh (e.g. the custom-LUT grayscale path) powers it back on instead of
    // issuing a display command against a powered-down panel (which hangs BUSY).
    _isScreenOn = (seqOverride & 0x03) ? false : !turnOff;
#if defined(SSD1677_PROBE_DEBUG) && SSD1677_PROBE_DEBUG
    esp_rom_printf("[SSD1677] %s refresh %ums (ctrl2=0x%x, seq)\n", dbgMode, (unsigned)(millis() - dbgStart),
                   seqOverride);
#endif
    return;
  }

  uint8_t displayMode = 0x00;
  if (!_isScreenOn) {
    _isScreenOn = true;
    displayMode |= 0xC0;  // CLOCK_ON | ANALOG_ON
  }
  if (turnOff) {
    _isScreenOn = false;
    displayMode |= 0x03;  // ANALOG_OFF_PHASE | CLOCK_OFF
  }

  if (mode == RefreshMode::Full) {
    displayMode |= 0x34;
  } else if (mode == RefreshMode::Half) {
    if (_cfg.halfUsesFullWaveform) {
      // No temperature spoof: this panel's wrapper doesn't select a usable
      // waveform from a faked temperature, so take the full one (see the field).
      displayMode |= 0x34;
    } else {
      bus.cmd(CMD_WRITE_TEMP);
      bus.data(_cfg.halfRefreshTemp);
      displayMode |= 0xD4;
    }
  } else if (_customLutActive) {
    // External-LUT (AA grayscale) activation is the absolute 0xCC sequence per the
    // vendor reference — clock/analog enable + display, WITHOUT the OTP LUT reload
    // (0x10 bit clear). The enable bits are a no-op when the rails are already up
    // (the usual X4 case, where stage 1 left them on), and required when they are
    // not, so 0xCC is correct in both states. The production driver marks power OFF
    // after this pass; mirror that so the next refresh re-enables the rails.
    // DE-LINK WATCH: this replaces the pre-merge grayscale path — verify AA text +
    // covers on hardware; de-link's wrapper is picky about absolute sequences.
    displayMode = 0xCC;
    if (turnOff) displayMode |= 0x03;
    _isScreenOn = false;
  } else {  // Fast
    displayMode |= 0x1C;
  }

  bus.cmd(CMD_DISPLAY_UPDATE_CTRL2);
  bus.data(displayMode);
  bus.cmd(CMD_MASTER_ACTIVATION);
  if (!async) bus.waitRefreshComplete("refresh");
#if defined(SSD1677_PROBE_DEBUG) && SSD1677_PROBE_DEBUG
  // esp_rom_printf hits the always-on IDF console; Serial (HWCDC) drops on S3.
  esp_rom_printf("[SSD1677] %s refresh %ums (ctrl2=0x%x)\n", dbgMode, (unsigned)(millis() - dbgStart), displayMode);
#endif
}

void Ssd1677Driver::powerOn(EpdBus& bus) {
  if (_isScreenOn) return;
  bus.cmd(CMD_DISPLAY_UPDATE_CTRL2);
  bus.data(0xC0);  // CLOCK_ON | ANALOG_ON
  bus.cmd(CMD_MASTER_ACTIVATION);
  bus.waitBusy("gray power-on");
  _isScreenOn = true;
}

void Ssd1677Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  displayImpl(bus, fb, prev, mode, turnOff, /*async=*/false);
}

// Deferred refresh: fire the update and return; displayFinish() waits it out.
// Skips the single-buffer post-refresh baseline resync — the facade supplies
// `prev` (its shadow) on shadowed updates, and the no-shadow/grayscale flow
// re-seeds the baseline itself (cleanupGrayscaleBuffers).
bool Ssd1677Driver::displayStart(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode,
                                 bool turnOff) {
  displayImpl(bus, fb, prev, mode, turnOff, /*async=*/true);
  return true;
}

void Ssd1677Driver::displayFinish(EpdBus& bus, const uint8_t* fb) {
  (void)fb;  // X4 post-waveform needs nothing from the host frame
  bus.waitRefreshComplete("refresh");
}

void Ssd1677Driver::displayImpl(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff,
                                bool async) {
  // The first paint after boot/wake must be an absolute clean, not a partial/DU
  // refresh: a partial only drives pixels that differ from the RED "old" plane, so
  // it can't clear what is physically on the panel at boot (e.g. the sleep screen).
  // Stock parity: the OEM firmware's only clean primitive in normal operation is
  // the single-pass HALF (0xD7 "_updateFull"); it never runs the multi-flash OTP
  // full waveform (0xF7 is a dead fallback branch there). Promote to HALF where
  // the board has one; boards without (Sticky) promote to their vendor FULL.
  if (!turnOff) {
    if (_needsInitialFull) {
      // First paint after boot/wake must not be a differential FAST: it only drives
      // pixels that differ from the RED baseline, so it can't clear whatever is
      // physically on the panel (the boot screen) and that ghosts through. But a HALF
      // or FULL the caller already asked for is non-differential (BYPASS_RED) and
      // clears the panel + seeds the baseline on its own — honor it and just consume
      // the one-shot. Only upgrade a FAST request. This keeps the boot logo (a HALF)
      // from paying an extra multi-inversion FULL-waveform flash on top of its own.
      if (mode == RefreshMode::Fast) mode = RefreshMode::Full;
      _needsInitialFull = false;
      mode = (_cfg.halfSeqOverride != 0) ? RefreshMode::Half : RefreshMode::Full;
    } else if (!_isScreenOn && _cfg.fullSeqOverride == 0) {
      // X4-class cold start: panel asleep -> a (warmed) HALF full-clear. Override
      // boards skip this — their fast sequence self-powers, so _isScreenOn is false
      // every page and forcing HALF would make every page a slow full-waveform flash.
      mode = RefreshMode::Half;
    }
  }

  // Leaving grayscale content without the firmware's cleanup: stock parity — the
  // OEM firmware has NO revert waveform (its grayscale sequence just resyncs RED
  // afterwards). RED still holds the gray MSB plane here, so it can't serve as a
  // differential baseline; promote a Fast update to the single-pass HALF clean
  // (absolute waveform, both planes rewritten below), which is also how stock
  // erases AA residue (its periodic 0xD7 promote).
  if (_inGrayscaleMode) {
    _inGrayscaleMode = false;
    if (mode == RefreshMode::Fast) {
      mode = RefreshMode::Half;
    }
  }

  setRamArea(bus, 0, 0, _w, _h);

  if (mode != RefreshMode::Fast) {
    writeRam(bus, CMD_WRITE_RAM_BW, fb, _bufferSize);
    writeRam(bus, CMD_WRITE_RAM_RED, fb, _bufferSize);
  } else {
    writeRam(bus, CMD_WRITE_RAM_BW, fb, _bufferSize);
    if (_darkBackground) {
      // Inverted content: the DU compare idles unchanged pixels, so the light
      // residue of every white->black transition parks in the black background
      // and accumulates between absolute cleans. Write RED as the complement of
      // the target instead of the previous frame: every pixel then classifies
      // as changed-toward-target and is re-driven — re-blackening the
      // background (and re-whitening standing text) on every page turn, which
      // is optically invisible on pixels already at their endpoint. Scan time
      // is unchanged (the DU waveform length does not depend on how many
      // pixels drive). Works identically in single-buffer mode, where no host
      // copy of the previous frame exists. Same mechanism as the Paper Mono
      // driver's dark-background selector and its forceAll corrective.
      writeRamInverted(bus, CMD_WRITE_RAM_RED, fb, _bufferSize);
    } else if (prev != nullptr) {
      // Dual-buffer: RED holds the previous frame for the differential compare.
      // Single-buffer (prev == nullptr): RED already holds it from last refresh.
      writeRam(bus, CMD_WRITE_RAM_RED, prev, _bufferSize);
    }
  }

  refresh(bus, mode, turnOff, async);

  // Stock X4 syncs both controller RAM planes after activation. Do the same in
  // single-buffer mode so the next differential update starts from a matched
  // BW/RED baseline instead of assuming BW survived the refresh unchanged.
  // (Async updates always come with a facade-owned prev, so this never runs
  // while a refresh is still in flight.)
  if (prev == nullptr && !async) {
    setRamArea(bus, 0, 0, _w, _h);
    writeRam(bus, CMD_WRITE_RAM_BW, fb, _bufferSize);
    writeRam(bus, CMD_WRITE_RAM_RED, fb, _bufferSize);
  }
}

void Ssd1677Driver::displayWindow(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, uint16_t x, uint16_t y,
                                  uint16_t w, uint16_t h, bool turnOff) {
  if (x + w > _w || y + h > _h) return;
  if (x % 8 != 0 || w % 8 != 0) return;  // window must be byte-aligned
  if (!fb) return;

  // A windowed diff can't run against grayscale plane contents (RED holds the
  // gray MSB plane, not the previous frame). With no revert waveform (stock
  // parity), exit grayscale by painting the whole frame with the single-pass
  // HALF clean instead; the window content is part of fb, so nothing is lost.
  if (_inGrayscaleMode) {
    displayImpl(bus, fb, nullptr, RefreshMode::Half, turnOff, /*async=*/false);
    return;
  }

  const uint16_t windowWidthBytes = w / 8;
  const uint32_t windowBufferSize = static_cast<uint32_t>(windowWidthBytes) * h;

  std::vector<uint8_t> windowBuffer(windowBufferSize);
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t srcY = y + row;
    const uint32_t srcOffset = static_cast<uint32_t>(srcY) * _wb + (x / 8);
    const uint32_t dstOffset = static_cast<uint32_t>(row) * windowWidthBytes;
    memcpy(&windowBuffer[dstOffset], &fb[srcOffset], windowWidthBytes);
  }

  // 180° mount: the extraction above is in logical coordinates; writeRam rotates
  // the window's content, and the RAM window itself must sit at the mirrored
  // position so the rotated content lands where the logical rect appears on the
  // flipped panel. Alignment survives the transform: x, w and _w are all
  // multiples of 8, so _w−x−w is too.
  const uint16_t ramX = _rot180 ? static_cast<uint16_t>(_w - x - w) : x;
  const uint16_t ramY = _rot180 ? static_cast<uint16_t>(_h - y - h) : y;

  setRamArea(bus, ramX, ramY, w, h);
  writeRam(bus, CMD_WRITE_RAM_BW, windowBuffer.data(), windowBufferSize);

  if (_darkBackground) {
    // Inverted content: same as displayImpl()'s dark path — write the window's
    // "old" plane as the complement of its target so every pixel in the window
    // is re-driven toward its target. Without this, dismissing an overlay
    // diffs against the true previous frame, the window's black background
    // idles, and the overlay's light residue stays parked there (with nothing
    // repainting afterwards to fade it).
    std::vector<uint8_t> invertedWindow(windowBufferSize);
    for (uint32_t i = 0; i < windowBufferSize; i++) {
      invertedWindow[i] = static_cast<uint8_t>(~windowBuffer[i]);
    }
    writeRam(bus, CMD_WRITE_RAM_RED, invertedWindow.data(), windowBufferSize);
  } else if (prev != nullptr) {
    std::vector<uint8_t> previousWindow(windowBufferSize);
    for (uint16_t row = 0; row < h; row++) {
      const uint16_t srcY = y + row;
      const uint32_t srcOffset = static_cast<uint32_t>(srcY) * _wb + (x / 8);
      const uint32_t dstOffset = static_cast<uint32_t>(row) * windowWidthBytes;
      memcpy(&previousWindow[dstOffset], &prev[srcOffset], windowWidthBytes);
    }
    writeRam(bus, CMD_WRITE_RAM_RED, previousWindow.data(), windowBufferSize);
  }

  refresh(bus, RefreshMode::Fast, turnOff);

  if (prev == nullptr) {
    setRamArea(bus, ramX, ramY, w, h);  // same mirrored window as the display write
    writeRam(bus, CMD_WRITE_RAM_BW, windowBuffer.data(), windowBufferSize);
    writeRam(bus, CMD_WRITE_RAM_RED, windowBuffer.data(), windowBufferSize);
  }
}

void Ssd1677Driver::seedPreviousFrame(EpdBus& bus, const uint8_t* buf) {
  if (!buf) return;
  // Write the frame into RED (the differential "old frame" plane) with no refresh —
  // identical to the RED write display() does for `prev`, so the next prev==nullptr
  // fast refresh diffs the new frame against this baseline instead of a stale one.
  setRamArea(bus, 0, 0, _w, _h);
  writeRam(bus, CMD_WRITE_RAM_RED, buf, _bufferSize);
}

void Ssd1677Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  if (!lsb) return;
  setRamArea(bus, 0, 0, _w, _h);
  writeRam(bus, CMD_WRITE_RAM_BW, lsb, _bufferSize);
}

void Ssd1677Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  if (!msb) return;
  setRamArea(bus, 0, 0, _w, _h);
  writeRam(bus, CMD_WRITE_RAM_RED, msb, _bufferSize);
}

void Ssd1677Driver::writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                             uint16_t numRows) {
  if (!rows || numRows == 0) return;
  const uint16_t len = static_cast<uint16_t>(static_cast<uint32_t>(numRows) * _wb);
  const uint8_t ramCmd = (plane == GrayPlane::Lsb) ? CMD_WRITE_RAM_BW : CMD_WRITE_RAM_RED;
  // 180° mount: the strip's own content is rotated by writeRam (full-width rows,
  // so the strip-local reversal is exact), and the strip lands mirrored in Y.
  const uint16_t ramY = _rot180 ? static_cast<uint16_t>(_h - yStart - numRows) : yStart;
  setRamArea(bus, 0, ramY, _w, numRows);
  writeRam(bus, ramCmd, rows, len);
}

void Ssd1677Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                                bool factoryMode) {
  (void)fb;

  // Differential mode marks grayscale content on the panel (the next BW update
  // must not diff against the gray planes); factory absolute mode self-cleans.
  _inGrayscaleMode = !factoryMode;

  const unsigned char* selectedLut = lut;
  if (selectedLut == nullptr) {
    selectedLut = factoryMode ? lut_factory_quality : _cfg.grayLut;
  }
  setCustomLut(bus, true, selectedLut);

  if (factoryMode) {
    // Explicit, self-contained power cycle for 4-level absolute grayscale.
    // Reset CTRL1 to normal — a prior HALF leaves BYPASS_RED set, which would
    // ignore RED RAM and break 4-level grayscale.
    bus.cmd(CMD_DISPLAY_UPDATE_CTRL1);
    bus.data(CTRL1_NORMAL);
    bus.cmd(CMD_DISPLAY_UPDATE_CTRL2);
    bus.data(0xC7);  // CLOCK_ON|ANALOG_ON|DISPLAY_START|ANALOG_OFF|CLOCK_OFF
    bus.cmd(CMD_MASTER_ACTIVATION);
    bus.waitBusy("factory_gray");
    _isScreenOn = false;  // 0xC7 always powers down after the update
  } else {
    // Settled rails before the gray waveform (no-op where the panel is already
    // on, i.e. the X4's fast path). refresh() then runs the 0xCC external-LUT
    // sequence (its enable bits are a no-op on already-up rails).
    if (_cfg.grayPowerUpFirst) powerOn(bus);
    refresh(bus, RefreshMode::Fast, turnOff);
  }

  setCustomLut(bus, false, nullptr);
}

void Ssd1677Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  if (!bw) return;
  setRamArea(bus, 0, 0, _w, _h);
  writeRam(bus, CMD_WRITE_RAM_RED, bw, _bufferSize);
  // The restored BW frame in RED RAM *is* the clean differential baseline for the
  // next BW page turn, so nothing else is needed — clear the flag (stock parity:
  // the OEM firmware's grayscale sequence ends exactly this way, RED resync only;
  // it has no revert waveform at all).
  _inGrayscaleMode = false;
}

void Ssd1677Driver::setCustomLut(EpdBus& bus, bool enabled, const unsigned char* data) {
  if (!enabled) {
    _customLutActive = false;
    return;
  }

  // First 105 bytes: VS + TP/RP + frame rate.
  bus.cmd(CMD_WRITE_LUT);
  for (uint16_t i = 0; i < 105; i++) {
    bus.data(pgm_read_byte(&data[i]));
  }

  bus.cmd(CMD_GATE_VOLTAGE);  // VGH
  bus.data(pgm_read_byte(&data[105]));

  bus.cmd(CMD_SOURCE_VOLTAGE);  // VSH1, VSH2, VSL
  bus.data(pgm_read_byte(&data[106]));
  bus.data(pgm_read_byte(&data[107]));
  bus.data(pgm_read_byte(&data[108]));

  bus.cmd(CMD_WRITE_VCOM);  // VCOM
  bus.data(pgm_read_byte(&data[109]));

  // The border register keeps its last value across refreshes; a follow-LUT value
  // (Sticky's 0x01) would now track the loaded grayscale LUT and drive the border
  // black. Park it while the custom LUT is active; the next normal refresh restores
  // the per-mode border via the seqOverride path.
  if (_cfg.borderWaveformGray != 0) {
    bus.cmd(CMD_BORDER_WAVEFORM);
    bus.data(_cfg.borderWaveformGray);
  }

  _customLutActive = true;
}

void Ssd1677Driver::deepSleep(EpdBus& bus) {
  // Stock parity (_powerOff): park the border at its init value so it is not left
  // driven with the full-refresh waveform through deep sleep, then power down
  // analog/clock. Stock does not touch CTRL1 here.
  if (_isScreenOn) {
    bus.cmd(CMD_BORDER_WAVEFORM);
    bus.data(_cfg.borderWaveformInit);
    bus.cmd(CMD_DISPLAY_UPDATE_CTRL2);
    bus.data(0x03);  // ANALOG_OFF_PHASE | CLOCK_OFF
    bus.cmd(CMD_MASTER_ACTIVATION);
    bus.waitBusy(" display power-down");
    _isScreenOn = false;
  }
  // Stock parity: deep sleep mode 2 (0x03) discards controller RAM. Nothing may
  // treat RAM as a valid diff baseline after wake — initController() re-arms
  // _needsInitialFull, so the first paint is an absolute clean anyway.
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0x03);
}

// Per-board waveform/LUT injection: a board supplies its own SSD1677 config
// (booster, scan, grayscale LUTs) without editing this driver — define
// `const Ssd1677Config& yourConfig();` in namespace freeink and build with
// -DFREEINK_SSD1677_CONFIG=yourConfig. Resolution is orthogonal: every driver,
// including X3, takes its geometry from the active BoardProfile.
#ifdef FREEINK_SSD1677_CONFIG
const Ssd1677Config& FREEINK_SSD1677_CONFIG();
static const Ssd1677Config& ssd1677ActiveConfig() { return FREEINK_SSD1677_CONFIG(); }
#else
// Select the per-board config from the active profile — no extra build flag, it
// follows the -DFREEINK_DEVICE_<NAME> selection (ACTIVE.board). Boards not listed
// use the X4/GDEQ0426T82 defaults.
static const Ssd1677Config& ssd1677ActiveConfig() {
  switch (BoardConfig::ACTIVE.board) {
    case BoardConfig::Board::Sticky: return ssd1677StickyConfig();
    case BoardConfig::Board::DeLink: return ssd1677DeLinkConfig();
    // X4 Pro runs on the stock X4/GDEQ0426T82 config — same controller and panel
    // class, confirmed painting on hardware. No custom LUT or drive voltages needed.
    case BoardConfig::Board::XteinkX4Pro: return ssd1677DefaultConfig();
    // X4 layers the fast-DU shortcut on the default only when the build has
    // opted in (see ssd1677X4Config); stock 0xFC parity otherwise.
#ifdef FREEINK_X4_FAST_DU_SHORTCUT
    case BoardConfig::Board::XteinkX4: return ssd1677X4Config();
#endif
    default: return ssd1677DefaultConfig();
  }
}
#endif

PanelDriver& ssd1677Driver() {
  static Ssd1677Driver instance(ssd1677ActiveConfig());
  return instance;
}

}  // namespace freeink
