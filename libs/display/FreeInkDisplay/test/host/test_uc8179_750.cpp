#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

#define private public
#include "FreeInkDisplay.h"
#include "src/driver/Gdey075t7Driver.h"
#undef private

using namespace freeink;
using Bytes = std::vector<uint8_t>;

const EpdBus::Write& last(const EpdBus& bus, uint8_t command) {
  for (auto i = bus.writes.rbegin(); i != bus.writes.rend(); ++i)
    if (i->command == command) return *i;
  assert(false);
  return bus.writes.front();
}

bool contains(const EpdBus& bus, uint8_t command) {
  return std::any_of(bus.writes.begin(), bus.writes.end(), [command](const auto& w) { return w.command == command; });
}

void checkNoCustomLuts(const EpdBus& bus) {
  for (const auto& write : bus.writes) {
    assert(write.command < 0x20 || write.command > 0x25);
    assert(write.command != 0x03 && write.command != 0x65 && write.command != 0xE1);
    assert(write.command != 0x91 && write.command != 0x92);
  }
}

void checkFull(const EpdBus& bus, const Bytes& frame) {
  assert(last(bus, 0x00).bytes == Bytes{0x1F});
  assert(last(bus, 0xE0).bytes == Bytes{2} && last(bus, 0xE5).bytes == Bytes{0x5A});
  assert(last(bus, 0x10).bytes == frame && last(bus, 0x13).bytes == frame);
  assert(last(bus, 0x10).transactions == 1 && last(bus, 0x13).transactions == 1);
  assert(contains(bus, 0x12));
  checkNoCustomLuts(bus);
}

int main() {
  BoardConfig::ACTIVE.displayController = BoardConfig::DisplayController::UC8179;
  Gdey075t7Driver driver;
  EpdBus bus;
  assert(driver.geometry().bufferSize == 48000 && driver.geometry().widthBytes == 100);
  assert(driver.spiHz() == 10000000 && driver.busyPolarity() == BusyPolarity::UcIdleHigh);
  BoardConfig::ACTIVE.displaySpiHz = 0;
  assert(driver.spiHz() == 2000000);
  BoardConfig::ACTIVE.displaySpiHz = 10000000;
  assert(!driver.supportsAsyncDisplay());
  for (auto mode : {GrayscaleMode::Overlay, GrayscaleMode::Absolute, GrayscaleMode::Direct})
    assert(!driver.grayscaleCapabilities(mode).supported());
  driver.begin(bus);
  assert(last(bus, 0x00).bytes == Bytes{0x1F});
  assert(last(bus, 0x01).bytes == (Bytes{0x07, 0x07, 0x3F, 0x3F, 0x09}));
  assert(last(bus, 0x06).bytes == (Bytes{0x17, 0x17, 0x28, 0x17}));
  assert(last(bus, 0x61).bytes == (Bytes{0x03, 0x20, 0x01, 0xE0}));
  assert(last(bus, 0x15).bytes == Bytes{0});
  assert(last(bus, 0x50).bytes == (Bytes{0x29, 0x07}));  // PR80 enables controller NEW-to-OLD copy.
  assert(last(bus, 0xE3).bytes == Bytes{0x22});
  checkNoCustomLuts(bus);

  Bytes first(48000, 0xAA), next(48000, 0x55);
  first[0] = 0x12;
  first[47900] = 0x34;
  bus.clear();
  assert(!driver.displayStart(bus, first.data(), nullptr, RefreshMode::Fast, true));
  checkFull(bus, first);  // First FAST is a blocking full refresh with native, unflipped plane order.
  assert(contains(bus, 0x04) && contains(bus, 0x02) && !driver._screenOn);
  bus.clear();
  driver.displayFinish(bus, first.data());
  assert(bus.writes.empty());

  driver.display(bus, next.data(), nullptr, RefreshMode::Fast, true);
  assert(last(bus, 0xE0).bytes == Bytes{2} && last(bus, 0xE5).bytes == Bytes{0x6E});
  assert(!contains(bus, 0x10));  // Single-buffer refresh uses the controller's retained baseline.
  assert(last(bus, 0x13).bytes == next && last(bus, 0x13).transactions == 1);
  assert(contains(bus, 0x02));
  checkNoCustomLuts(bus);

  bus.clear();
  driver.display(bus, first.data(), next.data(), RefreshMode::Fast, false);
  assert(last(bus, 0x10).bytes == next && last(bus, 0x13).bytes == first);
  assert(last(bus, 0xE5).bytes == Bytes{0x6E} && !contains(bus, 0x02));
  for (auto mode : {RefreshMode::Full, RefreshMode::Half}) {
    bus.clear();
    driver.display(bus, next.data(), first.data(), mode, true);
    checkFull(bus, next);
  }

  driver.requestResync(1);
  bus.clear();
  driver.display(bus, first.data(), nullptr, RefreshMode::Fast, false);
  checkFull(bus, first);
  bus.clear();
  driver.deepSleep(bus);
  assert(contains(bus, 0x02) && last(bus, 0x07).bytes == Bytes{0xA5});
  bus.clear();
  driver.display(bus, next.data(), first.data(), RefreshMode::Fast, true);
  assert(last(bus, 0x61).bytes == (Bytes{0x03, 0x20, 0x01, 0xE0}));
  checkFull(bus, next);  // Direct wake/reinitialization also requires a full baseline.
  driver.begin(bus);
  driver.skipInitialResync();
  bus.clear();
  driver.display(bus, first.data(), nullptr, RefreshMode::Fast, true);
  checkFull(bus, first);
  bus.clear();
  driver.display(bus, nullptr, nullptr, RefreshMode::Fast, true);
  assert(bus.writes.empty());

  FreeInkDisplay facade(13, 12, 14, 21, 47, 48);
  facade.begin();
  assert(std::strcmp(facade.getControllerName(), "UC8179") == 0);
  assert(facade._driver == &gdey075t7Driver());
  assert(!facade.grayscaleCapabilities().supported() && !facade.supportsAsyncRefresh());
  facade.clearScreen();
  facade.displayBuffer(FreeInkDisplay::FAST_REFRESH);
  checkNoCustomLuts(facade._bus);
  facade._bus.clear();
  facade.triggerDisplayAsync(FreeInkDisplay::FAST_REFRESH, false);
  assert(!facade._refreshPending && last(facade._bus, 0xE5).bytes == Bytes{0x6E});
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  assert(facade._asyncShadow == nullptr);
#endif

  // Existing EPUB/image callers submit masks even when grayscale is unsupported.
  auto& selected = static_cast<Gdey075t7Driver&>(*facade._driver);
  assert(!selected._firstRefresh && selected._screenOn);
  facade._bus.clear();
  facade.clearScreen(0x00);
  facade.copyGrayscaleLsbBuffers(facade.frameBuffer);
  facade.copyGrayscaleMsbBuffers(facade.frameBuffer);
  facade.displayGrayBuffer(false);
  assert(facade._bus.writes.empty() && !selected._firstRefresh && selected._screenOn);
  facade.displayGrayBuffer(true);
  assert(facade._bus.writes.size() == 1 && contains(facade._bus, 0x02));
  assert(!selected._firstRefresh && !selected._screenOn);
  facade.clearScreen();
  facade._bus.clear();
  facade.displayBuffer(FreeInkDisplay::FAST_REFRESH, true);
  assert(last(facade._bus, 0xE5).bytes == Bytes{0x6E});
  std::puts("PR80 GDEY075T7: native scan, blocking factory full/partial, previous planes, reset/wake, power-off and mono facade passed");
}
