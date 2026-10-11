// The Silkscreen 3.68" variant's panel identification (SILKSCREEN_PANEL_368) over the production
// detector: the X3's stock three-byte VER read decides (0x66 is a UC8279d, 0xFF a UC8253, anything
// else leaves the profile's UC8253), the two-pass VER/FLG/MTP probe never runs (a UC8253 also drives
// FLG), a conclusive verdict is reused on a deep-sleep wake with the bus silent, and a forced build
// never touches the bus. SDI has no external pull-up on this board, so the read releases it with the
// internal one: a pad nothing drives reads HIGH with it and LOW without (the model below), which is
// what makes a UC8253 that leaves the read undriven a conclusive, kept 0xFF. Build three times (see
// README.md): once as is, once each with -DSILKSCREEN_DISPLAY_CONTROLLER=uc8279 and =uc8253.
#include <Arduino.h>
#include <BoardConfig.h>
#include <XteinkDetect.h>
#include <esp_system.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#if !SILKSCREEN_PANEL_368 || !FREEINK_DEVICE_SILKSCREEN
#error "build this test with -DFREEINK_DEVICE_SILKSCREEN=1 -DSILKSCREEN_PANEL_368=1"
#endif

namespace {
using Controller = BoardConfig::DisplayController;
using Source = freeink::DisplayControllerSource;
using Verdict = freeink::DisplayControllerVerdict;

esp_reset_reason_t resetReason = ESP_RST_POWERON;
unsigned long now;
unsigned long readyAt;  // BUSY_N reads LOW until then
std::array<int, 64> levels{}, modes{};
// Replies per command. A command with none, or a read past its bytes, leaves SDI undriven; see digitalRead().
std::map<int, std::vector<uint8_t>> replies;
std::vector<int> commands;
std::vector<unsigned long> resetAt;
std::vector<int> resetLevel;
int command, commandBits, readBits, samples, holds;
int pullUpRequests;    // INPUT_PULLUP requests on SDI, counted before the pad takes them
bool padIgnoresPull;   // control: the pad does not take the pull-up request (the line as a bare INPUT read it)
const auto& pins = BoardConfig::SILKSCREEN_368.display;

// A panel answering the stock VER read with this id in byte 2 (byte 0 and 1 as the X3's modules read:
// reserved 0x00, CHIP_VER). drivesFlg: it also answers FLG (0x71) and the MTP read, as a UC8253 does.
std::map<int, std::vector<uint8_t>> panel(const uint8_t id, const bool drivesFlg = false) {
  std::map<int, std::vector<uint8_t>> p{{0x70, {0x00, 0x00, id}}};
  if (drivesFlg) {
    p[0x71] = {0x13};
    p[0xA2] = std::vector<uint8_t>(49, 0);
  }
  return p;
}

// A boot of the Silkscreen profile (the 3.68" variant by the macro) with this reset reason.
void boot(const esp_reset_reason_t reason, std::map<int, std::vector<uint8_t>> onBus, const unsigned long busyReadyAt = 0) {
  BoardConfig::selectDevice(BoardConfig::Board::Silkscreen);
  resetReason = reason;
  replies = std::move(onBus);
  now = 0;
  readyAt = busyReadyAt;
  commands.clear();
  resetAt.clear();
  resetLevel.clear();
  command = commandBits = readBits = samples = holds = pullUpRequests = 0;
  levels.fill(LOW);
  levels.at(pins.rst) = HIGH;  // RST_N rests HIGH (R5), so only the probe's pulses count
  modes.fill(INPUT);
}

// What the stock protocol does to the pins, on every probed boot.
void checkStockWire() {
  assert(commands == std::vector<int>{0x70});  // VER only: no FLG, no MTP, no second pass
  assert(samples == 24);                       // three bytes
  assert(pullUpRequests == 1);                 // SDI released with the internal pull-up, for the read (no external one)
  assert(resetAt.size() == 3 && resetLevel == (std::vector<int>{HIGH, LOW, HIGH}));
  assert(resetAt[1] - resetAt[0] == 10 && resetAt[2] - resetAt[1] == 50);  // 10 ms high, 50 ms low
  assert(holds == 1);                          // the sleep hold on RST released before the pulse
  // Every pin is released afterwards, the pull-up on SDI included: the driver's begin() sets them up again.
  assert(modes[pins.sclk] == INPUT && modes[pins.mosi] == INPUT && modes[pins.dc] == INPUT);
  assert(modes[pins.cs] == INPUT_PULLUP && levels[pins.cs] == HIGH && modes[pins.rst] == INPUT);
  const auto& diag = freeink::getXteinkDisplayProbeDiag();
  assert(diag.valid && diag.verBytesRead == 3);
  assert(diag.ver[3] == 0 && diag.ver[4] == 0 && diag.flg == 0 && !diag.mtpValid);
}
}  // namespace

esp_reset_reason_t esp_reset_reason() { return resetReason; }
void pinMode(int pin, int mode) {
  const bool sdiPull = pin == pins.mosi && mode == INPUT_PULLUP;
  if (sdiPull) ++pullUpRequests;
  modes.at(pin) = sdiPull && padIgnoresPull ? INPUT : mode;
}
void gpio_hold_dis(gpio_num_t pin) {
  if (pin == pins.rst) ++holds;
}
void digitalWrite(int pin, int value) {
  if (pin == pins.rst) {
    resetAt.push_back(now);
    resetLevel.push_back(value);
  }
  if (pin == pins.cs && value == LOW) command = commandBits = readBits = 0;
  if (pin == pins.sclk && value == HIGH && levels[pin] == LOW && levels[pins.cs] == LOW && levels[pins.dc] == LOW &&
      modes[pins.mosi] == OUTPUT) {
    command = ((command << 1) | levels[pins.mosi]) & 0xFF;
    if (++commandBits == 8) commands.push_back(command);
  }
  levels.at(pin) = value;
}
int digitalRead(int pin) {
  if (pin == pins.busy) return now >= readyAt ? HIGH : LOW;
  // Half-duplex read on SDI (IO12): selected, data phase, sampled while SCLK is HIGH, and SDA released
  // as an input.
  assert(pin == pins.mosi && levels[pins.cs] == LOW && levels[pins.dc] == HIGH);
  assert(levels[pins.sclk] == HIGH && (modes[pin] == INPUT || modes[pin] == INPUT_PULLUP));
  const int bit = readBits++;
  ++samples;
  const auto it = replies.find(command);
  if (it != replies.end() && static_cast<size_t>(bit / 8) < it->second.size()) {
    return (it->second[bit / 8] >> (7 - bit % 8)) & 1;  // the panel drives its answer, whatever the pad's pull
  }
  // Nothing drives the line, and Silkscreen's SDI has no external pull-up: a released pad with no pull
  // keeps the level it was last driven to (0x70 ends in a 0), so it reads LOW; with the internal
  // pull-up it reads HIGH.
  return modes[pin] == INPUT_PULLUP ? HIGH : LOW;
}
void delay(unsigned long ms) { now += ms; }
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return now; }

int main() {
  using namespace freeink;
  const auto& active = BoardConfig::ACTIVE;
  const auto& diag = getXteinkDisplayProbeDiag();

  // The profile the build selects: the 3.68" panel, no touch, the X3 pins' probe policy.
  boot(ESP_RST_POWERON, {});
  assert(BoardConfig::isSilkscreen() && active.board == BoardConfig::Board::Silkscreen);
  assert(active.displayWidth == 792 && active.displayHeight == 528);
  assert(BoardConfig::displayProbePolicy().ultraChipController == Controller::UC8279);
  assert(BoardConfig::displayProbePolicy().cacheAcrossDeepSleep);
  assert(!BoardConfig::hasTouch() && BoardConfig::hasFrontButtons());
  assert(!strcmp(active.name, "silkscreen_368") && BoardConfig::MAX_FRAMEBUFFER_BYTES == 52272);
  // The panel's data: the X3's 10 MHz clock, BUSY_N pulled up while awake, no SSD1677 pull-down.
  assert(active.displaySpiHz == BoardConfig::XTEINK_DISPLAY_SPI_HZ);
  assert(active.display.busyPullUp && !active.display.busyPullDown && BoardConfig::displayBusyPullUp());
  // No touch at all: no controller and no pin (and no fallback or sleep command, see BoardConfig.h).
  assert(active.touch.controller == BoardConfig::TouchController::None);
  assert(active.touch.sda < 0 && active.touch.scl < 0 && active.touch.irq < 0 && active.touch.reset < 0);
  // Everything else is the board's: IO11 stays parked HIGH, the panel pins and the power key are SILKSCREEN's.
  bool io11Parked = false;
  for (const auto& parked : active.parkedPins) {
    io11Parked |= parked.pin == 11 && parked.boot == BoardConfig::PinPark::High &&
                  parked.sleep == BoardConfig::PinPark::High;
  }
  assert(io11Parked);
  assert(active.input.power == BoardConfig::SILKSCREEN.input.power && active.input.powerExternalPull);
  assert(active.display.rst == BoardConfig::SILKSCREEN.display.rst && active.sdmmc.busWidth == 4);

#ifdef SILKSCREEN_DISPLAY_CONTROLLER
  // Forced: the profile carries the controller and nothing touches the bus, cold or woken.
  const Controller forced = BoardConfig::SILKSCREEN_368.displayController;
  assert(forced == Controller::UC8279 || forced == Controller::UC8253);
  assert(BoardConfig::displayProbePolicy().forced);
  for (const auto reason : {ESP_RST_POWERON, ESP_RST_DEEPSLEEP, ESP_RST_SW}) {
    boot(reason, panel(forced == Controller::UC8279 ? 0xFF : 0x66, true));  // a panel that would say otherwise
    assert(!applyXteinkDisplayController());
    assert(commands.empty() && resetAt.empty() && samples == 0);
    assert(displayControllerSource() == Source::Forced && active.displayController == forced && !diag.valid);
  }
  std::puts("Silkscreen 3.68\" forced-controller checks passed");
#else
  assert(!BoardConfig::displayProbePolicy().forced);
  assert(active.displayController == Controller::UC8253);  // the X3's default until the probe says UC8279d

  // A UC8279d (VER byte 2 = 0x66) found on a cold boot is promoted; the X3 stock read and nothing else.
  // About 111 ms of reset timing and the BUSY_N check (10 + 50 + 50 + 1).
  boot(ESP_RST_POWERON, panel(0x66));
  assert(applyXteinkDisplayController());
  assert(active.displayController == Controller::UC8279 && diag.promoted && displayControllerSource() == Source::Probed);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::Uc81xxConfirmed) && diag.ver[2] == 0x66);
  assert(now == 111 && !diag.busyTimedOut);
  checkStockWire();

  // The deep-sleep wake takes it from RTC memory: no reset pulse, no bus traffic, even with the bus silent.
  boot(ESP_RST_DEEPSLEEP, {});
  assert(applyXteinkDisplayController());
  assert(commands.empty() && resetAt.empty() && samples == 0 && now == 0);
  assert(displayControllerSource() == Source::Cached && displayControllerCached());
  assert(active.displayController == Controller::UC8279 && diag.promoted);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::Uc81xxConfirmed) && diag.ver[2] == 0x66);

  // A cold boot after that wake probes again (the cache is for a deep-sleep wake only), and the verdict
  // follows the panel: a UC8253 swapped in with the power off is found as one.
  boot(ESP_RST_POWERON, panel(0xFF, /*drivesFlg=*/true));
  assert(!applyXteinkDisplayController());
  assert(active.displayController == Controller::UC8253 && !diag.promoted);
  assert(displayControllerSource() == Source::Probed);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::PrimaryAssumed));
  checkStockWire();

  // The UC8253 trap: it also drives FLG and the MTP read. The two-pass probe takes a driven, stable FLG
  // for an UltraChip answer on Silkscreen; the stock read never asks, so it stays a UC8253 (checkStockWire
  // above: only 0x70 on the wire). Its verdict is kept for the next wake as well.
  boot(ESP_RST_DEEPSLEEP, {});
  assert(!applyXteinkDisplayController());
  assert(commands.empty() && displayControllerSource() == Source::Cached);
  assert(active.displayController == Controller::UC8253 && !diag.promoted);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::PrimaryAssumed));

  // A UC8253 that leaves the VER read undriven (the X3's modules read 0xFF through the bus's pull-up):
  // Silkscreen's SDI has none, so the probe releases it with the internal pull-up. The three bytes read
  // 0xFF, the verdict is PrimaryAssumed (conclusive: the profile's UC8253), and it is kept, so the next
  // deep-sleep wake takes it from RTC memory with the bus silent.
  boot(ESP_RST_POWERON, {});
  assert(!applyXteinkDisplayController());
  assert(active.displayController == Controller::UC8253 && !diag.promoted && displayControllerSource() == Source::Probed);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::PrimaryAssumed));
  assert(diag.ver[0] == 0xFF && diag.ver[1] == 0xFF && diag.ver[2] == 0xFF && now == 111);
  checkStockWire();
  boot(ESP_RST_DEEPSLEEP, {});
  assert(!applyXteinkDisplayController());
  assert(commands.empty() && resetAt.empty() && samples == 0 && now == 0);
  assert(displayControllerSource() == Source::Cached && displayControllerCached());
  assert(active.displayController == Controller::UC8253 && !diag.promoted);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::PrimaryAssumed));

  // Negative control: the same panel on a pad that does not take the pull-up request (the line as a bare
  // INPUT read it). It reads LOW, 0x00 is no id, the verdict is Inconclusive and is not kept, so every
  // wake probes again. This is what the pull-up above prevents; the production code still asks for it
  // (checkStockWire), and a driven UC8279d does not need it either way.
  padIgnoresPull = true;
  boot(ESP_RST_POWERON, {});
  assert(!applyXteinkDisplayController());
  assert(active.displayController == Controller::UC8253 && !diag.promoted);
  assert(diag.verdict == static_cast<uint8_t>(Verdict::Inconclusive));
  assert(diag.ver[0] == 0x00 && diag.ver[1] == 0x00 && diag.ver[2] == 0x00);
  checkStockWire();
  boot(ESP_RST_DEEPSLEEP, {});
  assert(!applyXteinkDisplayController());
  assert(!commands.empty() && samples == 24 && now == 111);
  assert(displayControllerSource() == Source::Probed && !displayControllerCached());
  boot(ESP_RST_POWERON, panel(0x66));
  assert(applyXteinkDisplayController());
  assert(active.displayController == Controller::UC8279 && diag.ver[2] == 0x66);
  padIgnoresPull = false;

  // An unknown id is inconclusive: the profile's UC8253 runs, and nothing is kept, so the next wake probes.
  for (const uint8_t id : {uint8_t(0x00), uint8_t(0x01), uint8_t(0x65), uint8_t(0x68), uint8_t(0xFE)}) {
    boot(ESP_RST_POWERON, panel(id, true));
    assert(!applyXteinkDisplayController());
    assert(active.displayController == Controller::UC8253 && !diag.promoted);
    assert(diag.verdict == static_cast<uint8_t>(Verdict::Inconclusive));
    checkStockWire();
    boot(ESP_RST_DEEPSLEEP, panel(0x66));
    assert(applyXteinkDisplayController());  // probed again, and finds the UC8279d now on the bus
    assert(!commands.empty() && displayControllerSource() == Source::Probed);
    assert(active.displayController == Controller::UC8279);
  }

  // Any other reset probes: a panic, a software restart or a brown-out right after a cached wake.
  boot(ESP_RST_POWERON, panel(0x66));
  assert(applyXteinkDisplayController());
  for (const auto reason : {ESP_RST_PANIC, ESP_RST_SW, ESP_RST_BROWNOUT, ESP_RST_POWERON}) {
    boot(reason, panel(0xFF));
    assert(!applyXteinkDisplayController());
    assert(displayControllerSource() == Source::Probed && active.displayController == Controller::UC8253);
    checkStockWire();
  }

  // The panel's BUSY_N is held LOW: the bounded wait (300 ms, as stock) ends and the id is still read.
  boot(ESP_RST_POWERON, panel(0x66), /*busyReadyAt=*/10000);
  assert(applyXteinkDisplayController());
  assert(diag.busyTimedOut && now == 410);  // 110 ms of reset timing + the 300 ms bound
  checkStockWire();

  // A dropped verdict (the first panel reset found BUSY at the wrong idle level) is not reused.
  boot(ESP_RST_POWERON, panel(0x66));
  assert(applyXteinkDisplayController());
  forgetCachedDisplayController();
  boot(ESP_RST_DEEPSLEEP, panel(0x66));
  assert(applyXteinkDisplayController());
  assert(!commands.empty() && displayControllerSource() == Source::Probed);

  // The explicit X3 API reads the same pins and protocol; the raw VER bytes come back as three read bytes.
  boot(ESP_RST_POWERON, panel(0x66));
  uint8_t ver[5] = {9, 9, 9, 9, 9};
  uint8_t flg = 9;
  assert(detectXteinkDisplayController(ver, &flg) == Verdict::Uc81xxConfirmed);
  assert(ver[2] == 0x66 && ver[3] == 0 && ver[4] == 0 && flg == 0);
  checkStockWire();
  std::puts("Silkscreen 3.68\" controller policy checks passed");
#endif
}
