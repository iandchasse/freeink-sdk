// A fixed GDEY075T7 controller must not produce probe traffic at boot or wake.
#undef NDEBUG
#include <Arduino.h>
#include <BoardConfig.h>
#include <XteinkDetect.h>
#include <esp_system.h>

#include <cassert>
#include <cstdio>
#include <cstring>

#if !FREEINK_SILKSCREEN_750
#error "Build with FREEINK_DEVICE_SILKSCREEN=1 and SILKSCREEN_PANEL_750=1"
#endif

namespace {
unsigned ioCalls = 0;
esp_reset_reason_t resetReason = ESP_RST_POWERON;
}
esp_reset_reason_t esp_reset_reason() { return resetReason; }
void pinMode(int, int) { ++ioCalls; }
void gpio_hold_dis(gpio_num_t) { ++ioCalls; }
void digitalWrite(int, int) { ++ioCalls; }
int digitalRead(int) { ++ioCalls; return HIGH; }
void delay(unsigned long) { ++ioCalls; }
void delayMicroseconds(unsigned int) { ++ioCalls; }
unsigned long millis() { return 0; }

int main() {
  using namespace freeink;
  assert(BoardConfig::ACTIVE.displayController == BoardConfig::DisplayController::UC8179);
  assert(!std::strcmp(BoardConfig::ACTIVE.name, "silkscreen_750"));
  assert(BoardConfig::MAX_FRAMEBUFFER_BYTES == 48000);
  assert(BoardConfig::ACTIVE.displaySpiHz == 10000000);
  assert(BoardConfig::ACTIVE.display.busyPullUp && !BoardConfig::ACTIVE.display.busyPullDown);
  assert(BoardConfig::hasFrontButtons() && !BoardConfig::hasTouch());
  for (auto reason : {ESP_RST_POWERON, ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SW}) {
    resetReason = reason;
    BoardConfig::selectDevice(BoardConfig::Board::Silkscreen);
    ioCalls = 0;
    uint8_t ver[5] = {9, 9, 9, 9, 9}, flg = 9;
    assert(detectXteinkDisplayController(ver, &flg) == DisplayControllerVerdict::PrimaryAssumed);
    assert(!applyXteinkDisplayController());
    assert(displayControllerSource() == DisplayControllerSource::Forced && !displayControllerCached());
    forgetCachedDisplayController();
    for (uint8_t value : ver) assert(value == 0);
    assert(flg == 0 && ioCalls == 0);
    assert(BoardConfig::ACTIVE.displayController == BoardConfig::DisplayController::UC8179);
    assert(!getXteinkDisplayProbeDiag().valid);
  }
  std::puts("Silkscreen 7.5-inch: fixed UC8179, preserved board policy, no boot/wake probe traffic passed");
}
