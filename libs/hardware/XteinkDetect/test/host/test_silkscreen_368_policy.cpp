// The Silkscreen 3.68" variant (SILKSCREEN_PANEL_368) identifies its panel by nothing: the controller is the X3's
// UC8279d, fixed in the profile (SILKSCREEN_368), the probe policy is forced with no verdict cache, and
// XteinkDetect compiles no probe for this build (neither the X3 stock VER read nor the two-pass VER/FLG/MTP
// read). A cold boot and a deep-sleep wake, whatever the reset reason and whatever answers on the bus, make no
// bus traffic for identification, touch no pin and spend no time. The stub display bus below counts every call
// that could be such traffic: each must stay at zero. Build twice (see README.md): as is, and with
// -DSILKSCREEN_DISPLAY_CONTROLLER=uc8279, which is accepted and changes nothing.
// Every check below is an assert(), several of them around the call under test: a build with -DNDEBUG
// must not turn this into a pass that ran nothing.
#undef NDEBUG
#include <Arduino.h>
#include <BoardConfig.h>
#include <XteinkDetect.h>
#include <esp_system.h>

#include <cassert>
#include <cstdio>
#include <cstring>

#if !SILKSCREEN_PANEL_368 || !FREEINK_DEVICE_SILKSCREEN
#error "build this test with -DFREEINK_DEVICE_SILKSCREEN=1 -DSILKSCREEN_PANEL_368=1"
#endif

namespace {
using Controller = BoardConfig::DisplayController;
using Source = freeink::DisplayControllerSource;
using Verdict = freeink::DisplayControllerVerdict;

esp_reset_reason_t resetReason = ESP_RST_POWERON;
// What a probe would have to do: configure a pin, drive or read one, release a sleep hold, wait.
struct Calls {
  int pinModes = 0, writes = 0, reads = 0, holds = 0, waits = 0;
  bool none() const { return pinModes == 0 && writes == 0 && reads == 0 && holds == 0 && waits == 0; }
} calls;

// A boot of the Silkscreen profile (the 3.68" variant by the macro) with this reset reason.
void boot(const esp_reset_reason_t reason) {
  BoardConfig::selectDevice(BoardConfig::Board::Silkscreen);
  resetReason = reason;
  calls = {};
}
}  // namespace

esp_reset_reason_t esp_reset_reason() { return resetReason; }
void pinMode(int, int) { ++calls.pinModes; }
void gpio_hold_dis(gpio_num_t) { ++calls.holds; }
void digitalWrite(int, int) { ++calls.writes; }
int digitalRead(int) {
  ++calls.reads;
  return HIGH;
}
void delay(unsigned long) { ++calls.waits; }
void delayMicroseconds(unsigned int) { ++calls.waits; }
unsigned long millis() { return 0; }

int main() {
  using namespace freeink;
  const auto& active = BoardConfig::ACTIVE;
  const auto& diag = getXteinkDisplayProbeDiag();

  // The profile the firmware boots with. On an S3 nothing calls selectDevice() before ACTIVE is first
  // read, so DEFAULT_DEVICE (the pristine ACTIVE) is what runs: read both before boot() selects anything.
  assert(!strcmp(BoardConfig::DEFAULT_DEVICE.name, "silkscreen_368"));
  assert(BoardConfig::DEFAULT_DEVICE.displayWidth == 792 && BoardConfig::DEFAULT_DEVICE.displayHeight == 528);
  assert(BoardConfig::DEFAULT_DEVICE.touch.controller == BoardConfig::TouchController::None);
  assert(BoardConfig::DEFAULT_DEVICE.displayController == Controller::UC8279);
  assert(!strcmp(active.name, "silkscreen_368") && active.displayWidth == 792 && active.displayHeight == 528);
  assert(active.touch.controller == BoardConfig::TouchController::None);
  assert(active.displayController == Controller::UC8279);  // fixed in data, with or without =uc8279

  // Nothing has resolved the controller yet: the source says so until applyXteinkDisplayController() runs.
  assert(displayControllerSource() == Source::NotResolved && !displayControllerCached());

  // The profile the build selects: the 3.68" panel, no touch, a forced probe policy with no cache.
  boot(ESP_RST_POWERON);
  assert(BoardConfig::isSilkscreen() && active.board == BoardConfig::Board::Silkscreen);
  assert(active.displayWidth == 792 && active.displayHeight == 528);
  assert(active.displayController == Controller::UC8279);
  assert(BoardConfig::displayProbePolicy().forced);
  assert(!BoardConfig::displayProbePolicy().cacheAcrossDeepSleep);
  assert(BoardConfig::displayProbePolicy().ultraChipController == Controller::UC8279);
  assert(BoardConfig::displayProbePolicy().x4ClassPanel);  // CrossPoint's xteinkClassPanel() reads it
  assert(!BoardConfig::hasTouch() && BoardConfig::hasFrontButtons());
  assert(!strcmp(active.name, "silkscreen_368") && BoardConfig::MAX_FRAMEBUFFER_BYTES == 52272);
  // The panel's data: the X3's 10 MHz clock, BUSY_N pulled up while awake, no SSD1677 pull-down.
  assert(active.displaySpiHz == BoardConfig::XTEINK_DISPLAY_SPI_HZ);
  assert(active.display.busyPullUp && !active.display.busyPullDown && BoardConfig::displayBusyPullUp());
  // No touch at all: no controller and no pin (and no fallback or sleep command, see BoardConfig.h).
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

  // A cold boot, a deep-sleep wake and every other reset: the controller is the profile's, from the build, with
  // no pin touched, no wait and no bus traffic (the stub counts them), and nothing is kept for a later wake.
  for (const auto reason : {ESP_RST_POWERON, ESP_RST_DEEPSLEEP, ESP_RST_SW, ESP_RST_PANIC, ESP_RST_BROWNOUT, ESP_RST_EXT}) {
    boot(reason);
    assert(!applyXteinkDisplayController());  // nothing is promoted
    assert(calls.none());
    assert(displayControllerSource() == Source::Forced && !displayControllerCached());
    assert(active.displayController == Controller::UC8279 && active.displayControllerVariant == 0);
    assert(!diag.valid && !diag.promoted && !diag.mtpValid && diag.verBytesRead == 0 && !diag.busyTimedOut);
    // The cache a wake would take is not there to drop.
    forgetCachedDisplayController();
    assert(calls.none() && displayControllerSource() == Source::Forced);
  }

  // The probe API itself reads nothing: it answers the profile's controller with no bytes, on the bus
  // as on a wake (no probe is compiled into this build).
  for (const auto reason : {ESP_RST_POWERON, ESP_RST_DEEPSLEEP}) {
    boot(reason);
    uint8_t ver[5] = {9, 9, 9, 9, 9};
    uint8_t flg = 9;
    assert(detectXteinkDisplayController(ver, &flg) == Verdict::PrimaryAssumed);
    for (const uint8_t b : ver) assert(b == 0);
    assert(flg == 0 && calls.none());
    assert(detectXteinkDisplayController() == Verdict::PrimaryAssumed && calls.none());
  }

  // The C3-only fingerprints are compiled out here (an S3 build): inert, with no bus traffic either.
  boot(ESP_RST_POWERON);
  uint8_t score1 = 9, score2 = 9;
  assert(detectXteinkVerdict(&score1, &score2) == XteinkVerdict::Inconclusive && score1 == 0 && score2 == 0);
  assert(!detectXteinkIsX3() && !selectXteinkDevice());
  assert(detectX3DisplayController() == X3DisplayVerdict::Uc8253Assumed);
  assert(calls.none());
  assert(active.displayController == Controller::UC8279 && !strcmp(active.name, "silkscreen_368"));

#ifdef SILKSCREEN_DISPLAY_CONTROLLER
  // =uc8279 is accepted and is a no-op: nothing above differs, which the run just made.
  std::puts("Silkscreen 3.68\" fixed-controller checks passed (SILKSCREEN_DISPLAY_CONTROLLER=uc8279 accepted as a no-op)");
#else
  std::puts("Silkscreen 3.68\" fixed-controller checks passed");
#endif
}
