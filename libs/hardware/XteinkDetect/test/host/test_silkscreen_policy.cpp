// Silkscreen's display-controller policy (DisplayProbeConfig) over the production
// detector: any UltraChip answer is a UC8279 whatever its LUT_VER, a verdict is
// reused on a deep-sleep wake only, an inconclusive one is not kept, and a
// forced build never touches the bus. Build twice (see README.md): once as is,
// once with -DSILKSCREEN_DISPLAY_CONTROLLER=uc8279.
#include <Arduino.h>
#include <BoardConfig.h>
#include <XteinkDetect.h>
#include <esp_system.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <map>
#include <vector>

namespace {
using Controller = BoardConfig::DisplayController;
using Source = freeink::DisplayControllerSource;

esp_reset_reason_t resetReason = ESP_RST_POWERON;
unsigned long now;
std::array<int, 64> levels{}, modes{};
// Replies per command; a command with none reads the pulled-up, floating line (an SSD1677).
std::map<int, std::vector<uint8_t>> replies;
std::vector<int> commands;
int command, commandBits, readBits, flgReads;
bool flakyFlg;  // FLG answers once, then floats: the two passes disagree
const auto& pins = BoardConfig::SILKSCREEN.display;

std::map<int, std::vector<uint8_t>> ultraChip(const uint8_t lutVer) {
  return {{0x70, {0x00, 0x00, lutVer, 0xFF, 0xFF}}, {0x71, {0x13}}, {0xA2, std::vector<uint8_t>(49, 0)}};
}

// A boot of the Silkscreen profile with this reset reason and this panel on the bus.
void boot(const esp_reset_reason_t reason, std::map<int, std::vector<uint8_t>> panel, const bool flaky = false) {
  BoardConfig::selectDevice(BoardConfig::Board::Silkscreen);
  resetReason = reason;
  replies = std::move(panel);
  flakyFlg = flaky;
  commands.clear();
  command = commandBits = readBits = flgReads = 0;
  levels.fill(LOW);
  modes.fill(INPUT);
}
}  // namespace

esp_reset_reason_t esp_reset_reason() { return resetReason; }
void pinMode(int pin, int mode) { modes.at(pin) = mode; }
void gpio_hold_dis(gpio_num_t) {}
void digitalWrite(int pin, int value) {
  if (pin == pins.cs && value == LOW) command = commandBits = readBits = 0;
  if (pin == pins.sclk && value == HIGH && levels[pin] == LOW && levels[pins.cs] == LOW && levels[pins.dc] == LOW &&
      modes[pins.mosi] == OUTPUT) {
    command = ((command << 1) | levels[pins.mosi]) & 0xFF;
    if (++commandBits == 8) {
      commands.push_back(command);
      if (command == 0x71) ++flgReads;
    }
  }
  levels.at(pin) = value;
}
int digitalRead(int pin) {
  if (pin == pins.busy) return HIGH;
  // Half-duplex read on SDI (IO12): selected, data phase, the MCU pull-up on.
  assert(pin == pins.mosi && levels[pins.cs] == LOW && levels[pins.dc] == HIGH && modes[pin] == INPUT_PULLUP);
  const int bit = readBits++;
  const auto it = replies.find(command);
  if (it == replies.end() || static_cast<size_t>(bit / 8) >= it->second.size()) return HIGH;
  if (flakyFlg && command == 0x71 && flgReads > 1) return HIGH;
  return (it->second[bit / 8] >> (7 - bit % 8)) & 1;
}
void delay(unsigned long ms) { now += ms; }
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return now; }

int main() {
  using namespace freeink;
  const auto& active = BoardConfig::ACTIVE;
  const auto& diag = getXteinkDisplayProbeDiag();

#ifdef SILKSCREEN_DISPLAY_CONTROLLER
  // Forced: the profile carries the controller and nothing touches the bus.
  boot(ESP_RST_POWERON, ultraChip(0x02));
  assert(active.displayController == Controller::UC8279);
  assert(!applyXteinkDisplayController());
  assert(commands.empty() && displayControllerSource() == Source::Forced);
  assert(active.displayController == Controller::UC8279 && !diag.valid);
  boot(ESP_RST_DEEPSLEEP, {});
  assert(!applyXteinkDisplayController() && commands.empty());
  std::puts("Silkscreen forced-controller checks passed");
#else
  // A cold boot probes; an UltraChip answer is a UC8279 with its raw LUT_VER.
  boot(ESP_RST_POWERON, ultraChip(0x02));
  assert(active.displayController == Controller::SSD1677);
  assert(applyXteinkDisplayController());
  assert(!commands.empty() && commands[0] == 0x71 && displayControllerSource() == Source::Probed);
  assert(active.displayController == Controller::UC8279 && active.displayControllerVariant == 0x02);

  // The deep-sleep wake reuses it without a transaction, even with the bus silent.
  boot(ESP_RST_DEEPSLEEP, {});
  assert(applyXteinkDisplayController());
  assert(commands.empty() && displayControllerSource() == Source::Cached);
  assert(active.displayController == Controller::UC8279 && active.displayControllerVariant == 0x02);
  assert(diag.valid && diag.promoted && diag.ver[2] == 0x02 && diag.flg == 0x13);
  assert(diag.verdict == static_cast<uint8_t>(DisplayControllerVerdict::Uc81xxConfirmed));

  // Any other reset probes again: a panel swapped with the power off is found.
  for (const auto reason : {ESP_RST_POWERON, ESP_RST_PANIC, ESP_RST_SW, ESP_RST_BROWNOUT}) {
    boot(reason, {});
    assert(!applyXteinkDisplayController());
    assert(!commands.empty() && displayControllerSource() == Source::Probed);
    assert(active.displayController == Controller::SSD1677);
    assert(diag.verdict == static_cast<uint8_t>(DisplayControllerVerdict::PrimaryAssumed));
  }
  boot(ESP_RST_DEEPSLEEP, ultraChip(0x02));
  assert(!applyXteinkDisplayController() && commands.empty() && displayControllerSource() == Source::Cached);
  assert(active.displayController == Controller::SSD1677 && !diag.promoted);

  // LUT_VER never selects UC8179 (not linked): 0x01 and unknown ids stay UC8279.
  for (const uint8_t lutVer : {uint8_t(0x01), uint8_t(0x55), uint8_t(0x68), uint8_t(0x67)}) {
    boot(ESP_RST_POWERON, ultraChip(lutVer));
    assert(applyXteinkDisplayController());
    assert(active.displayController == Controller::UC8279 && active.displayControllerVariant == lutVer);
  }

  // An inconclusive probe runs as SSD1677 and is not kept: the wake probes again.
  boot(ESP_RST_POWERON, ultraChip(0x02), /*flaky=*/true);
  assert(!applyXteinkDisplayController());
  assert(diag.verdict == static_cast<uint8_t>(DisplayControllerVerdict::Inconclusive));
  assert(active.displayController == Controller::SSD1677);
  boot(ESP_RST_DEEPSLEEP, ultraChip(0x02));
  assert(applyXteinkDisplayController());
  assert(!commands.empty() && displayControllerSource() == Source::Probed);
  assert(active.displayController == Controller::UC8279);
  std::puts("Silkscreen controller policy checks passed");
#endif
}
