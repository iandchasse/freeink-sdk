// PowerManager::sleepPanelWithoutDriver() on the 4.26" Silkscreen board and on its 3.68" variant
// (SILKSCREEN_PANEL_368, a UC8279d panel): the controller's own deep-sleep command, how long the reset
// settles before it, the BUSY pull, and what is left alone. run_sleep_panel.py cuts the production function and its two
// helpers out of PowerManager.cpp into sleep_panel_under_test.inc, so this executes the text the
// firmware compiles, over a recording GPIO model. Build it once per board (see README.md).
// Every check below is an assert(), several of them around the call under test: a build with -DNDEBUG
// must not turn this into a pass that ran nothing.
#undef NDEBUG
#include <Arduino.h>
#include <BoardConfig.h>
#include <driver/gpio.h>

#include <cassert>
#include <cstdio>
#include <vector>

#if !FREEINK_DEVICE_SILKSCREEN
#error "build this test with -DFREEINK_DEVICE_SILKSCREEN=1 (and -DSILKSCREEN_PANEL_368=1 -DFREEINK_CAP_TOUCH=0)"
#endif

namespace {
using Controller = BoardConfig::DisplayController;

struct Event {
  enum Kind { Mode, Write, Level } kind;
  int pin;
  int value;
  unsigned long ms;
};
std::vector<Event> events;
unsigned long now;
unsigned long busyAssertedMs;  // BUSY is asserted until this long after RST rises
bool busyActiveHigh;
unsigned long rstRoseAt;
int levels[64];
}  // namespace

void pinMode(int pin, int mode) { events.push_back({Event::Mode, pin, mode, now}); }
void digitalWrite(int pin, int value) {
  events.push_back({Event::Write, pin, value, now});
  if (pin == BoardConfig::ACTIVE.display.rst && value == HIGH && levels[pin] == LOW) rstRoseAt = now;
  levels[pin] = value;
}
int digitalRead(int pin) {
  assert(pin == BoardConfig::ACTIVE.display.busy);
  const bool asserted = now - rstRoseAt < busyAssertedMs;
  return asserted == busyActiveHigh ? HIGH : LOW;
}
void delay(unsigned long ms) { now += ms; }
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return now; }
void gpio_hold_dis(gpio_num_t) {}
void gpio_hold_en(gpio_num_t) {}
void gpio_set_level(gpio_num_t pin, int value) {
  events.push_back({Event::Level, pin, value, now});
  if (pin == BoardConfig::ACTIVE.display.rst && value == HIGH && levels[pin] == LOW) rstRoseAt = now;
  levels[pin] = value;
}
void gpio_set_direction(gpio_num_t, gpio_mode_t) {}
void gpio_pullup_dis(gpio_num_t) {}
void gpio_pulldown_dis(gpio_num_t) {}

namespace freeink {
struct PowerManager {
  static bool sleepPanelWithoutDriver();
};
#include "sleep_panel_under_test.inc"
}  // namespace freeink

namespace {
struct Result {
  bool ok = false;
  unsigned long rstLowMs = 0;  // RST falls to RST rises
  unsigned long settleMs = 0;  // RST rises to the first byte (CS falling), the BUSY wait included
  std::vector<int> commands;   // bytes clocked with DC low
  std::vector<int> data;       // bytes clocked with DC high
  int busyMode = -1;           // the last pinMode() on BUSY
  size_t touched = 0;          // pin events recorded at all
};

// One call of the production function with `controller` on the active Silkscreen profile, on a panel that
// holds BUSY asserted for `assertedMs` after the reset releases.
Result run(const Controller controller, const unsigned long assertedMs, const bool activeHigh) {
  BoardConfig::selectDevice(BoardConfig::Board::Silkscreen);
  BoardConfig::ACTIVE.displayController = controller;
  events.clear();
  now = 0;
  rstRoseAt = 0;
  busyAssertedMs = assertedMs;
  busyActiveHigh = activeHigh;
  for (int& level : levels) level = LOW;

  Result r;
  r.ok = freeink::PowerManager::sleepPanelWithoutDriver();
  r.touched = events.size();

  // Replay the pin events: the reset pulse, the SPI mode-0 bytes bit-banged with CS low, BUSY's mode.
  const auto& d = BoardConfig::ACTIVE.display;
  int cs = HIGH, dc = LOW, mosi = LOW, sclk = LOW, bits = 0, value = 0;
  unsigned long rstFell = 0, rstRose = 0, firstByteAt = 0;
  bool fell = false, rose = false, clocked = false;
  for (const Event& e : events) {
    if (e.kind == Event::Mode) {
      if (e.pin == d.busy) r.busyMode = e.value;
      continue;
    }
    if (e.pin == d.rst) {
      if (e.value == LOW && !fell) {
        fell = true;
        rstFell = e.ms;
      } else if (e.value == HIGH && fell && !rose) {
        rose = true;
        rstRose = e.ms;
      }
    }
    if (e.pin == d.cs) {
      if (e.value == LOW && cs == HIGH) {
        if (rose && !clocked) {
          clocked = true;
          firstByteAt = e.ms;
        }
        bits = value = 0;
      }
      cs = e.value;
    }
    if (e.pin == d.dc) dc = e.value;
    if (e.pin == d.mosi) mosi = e.value;
    if (e.pin == d.sclk) {
      if (e.value == HIGH && sclk == LOW && cs == LOW) {
        value = ((value << 1) | mosi) & 0xFF;
        if (++bits == 8) {
          (dc == LOW ? r.commands : r.data).push_back(value);
          bits = value = 0;
        }
      }
      sclk = e.value;
    }
  }
  r.rstLowMs = rstRose - rstFell;
  r.settleMs = firstByteAt - rstRose;
  return r;
}

// The one command byte and the one data byte clocked out.
void expectBytes(const Result& r, const int command, const int check) {
  assert(r.commands.size() == 1 && r.data.size() == 1);
  assert(r.commands[0] == command && r.data[0] == check);
}
}  // namespace

int main() {
#if SILKSCREEN_PANEL_368
  // The 3.68" variant: the X3's UC8279d, which its profile fixes. Its driver waits 60 ms after its reset
  // releases (10 ms in EpdBus::reset() and 50 more), so the DSLP is not sent to a controller that is still
  // initialising. BUSY_N idles HIGH and is pulled up meanwhile.
  static_assert(FREEINK_SILKSCREEN_368, "the variant's build");
  assert(BoardConfig::ACTIVE.displayController == Controller::UC8279);
  {
    const Controller controller = Controller::UC8279;
    Result r = run(controller, /*assertedMs=*/0, /*activeHigh=*/false);
    assert(r.ok);
    expectBytes(r, 0x07, 0xA5);  // DSLP, then its check code
    assert(r.busyMode == INPUT_PULLUP);
    assert(r.rstLowMs == 11);
    assert(r.settleMs == 61);  // the 60 ms settle (+1 for the tick), BUSY_N already idle

    // BUSY_N still asserted 90 ms after the reset releases: the bounded wait rides it out, then DSLP.
    r = run(controller, 90, false);
    assert(r.ok && r.settleMs >= 90 && r.settleMs <= 92);
    expectBytes(r, 0x07, 0xA5);

    // BUSY_N asserted for good (a panel that is not answering): the 100 ms bound ends the wait and the
    // command goes out anyway, 61 ms of settle plus the bound and no more.
    r = run(controller, 1000000, false);
    assert(r.ok && r.settleMs >= 61 + 100 && r.settleMs <= 61 + 102);
    expectBytes(r, 0x07, 0xA5);
  }
  // A UC8253 has no driver in this build either (the variant links the UC8279d alone): the function
  // returns false and leaves every pin alone.
  {
    const Result r = run(Controller::UC8253, 0, false);
    assert(!r.ok && r.touched == 0);
  }
  std::puts("3.68\" variant: driver-free panel sleep checks passed");
#else
  // The 4.26" board: an SSD1677 by default, 0x10 / 0x03 with the pull-down on BUSY, and the UC8279 (the X4
  // driver's part) with DSLP 0x07 / 0xA5 and the pull-up. The reset settles 10 ms (+1) before the command,
  // as it did before the variant existed. A UC8253 has no driver in this build: the function returns
  // false and leaves every pin alone.
  static_assert(!FREEINK_SILKSCREEN_368, "the 4.26\" board's build");
  assert(BoardConfig::ACTIVE.displayController == Controller::SSD1677);
  Result r = run(Controller::SSD1677, 0, /*activeHigh=*/true);
  assert(r.ok);
  expectBytes(r, 0x10, 0x03);
  assert(r.busyMode == INPUT_PULLDOWN);
  assert(r.rstLowMs == 11 && r.settleMs == 11);

  r = run(Controller::UC8279, 0, false);
  assert(r.ok);
  expectBytes(r, 0x07, 0xA5);
  assert(r.busyMode == INPUT_PULLUP);
  assert(r.rstLowMs == 11 && r.settleMs == 11);

  r = run(Controller::UC8253, 0, false);
  assert(!r.ok && r.touched == 0);
  std::puts("4.26\" board: driver-free panel sleep checks passed");
#endif
}
