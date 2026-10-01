#include "EpdBus.h"

#include <BoardConfig.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#if FREEINK_DEVICE_PAPERMONO
#include <PaperMonoBoard.h>
#endif
#if FREEINK_DEVICE_WS397
#include <Axp2101.h>
#endif
#if FREEINK_CAP_DISPLAY_PROBE_POLICY
#include <esp_system.h>
// XteinkDetect's display-probe policy (XteinkDetect.h), declared here instead of
// included: PlatformIO's library finder does not read #if, so the include would
// make XteinkDetect a dependency of this library on every board and reorder
// their link.
namespace freeink {
bool displayControllerCached();
void forgetCachedDisplayController();
}  // namespace freeink
#endif

// Consumer escape hatch for boards whose EPD power/reset live behind glue the
// SDK doesn't know. Weak REFERENCES only — an SDK-internal implementation
// can't ride these (a weak ref doesn't force the archive member that defines
// it to link), which is why the Paper Mono path below is a direct call into
// its board-support header instead.
extern "C" void freeink_board_epd_power(bool enabled) __attribute__((weak));
extern "C" void freeink_board_epd_reset(bool high) __attribute__((weak));

namespace {
// EPD power/reset for boards without direct GPIOs: SDK board support first,
// then the consumer hooks.
void boardEpdPower(bool enabled) {
#if FREEINK_DEVICE_METALIO_EINK4
  if (BoardConfig::isMetalioEInk4()) {
    // Shared screen/SD rail must stay on until hardware power-off.
    if (enabled) freeink::metalio::ensureBooted();
    return;
  }
#endif
#if FREEINK_DEVICE_PAPERMONO
  freeink::papermono::setEpdPower(enabled);
#elif FREEINK_DEVICE_WS397
  freeink::axp2101::setEpdPower(enabled);  // ALDO3, not a GPIO
#else
  if (freeink_board_epd_power) freeink_board_epd_power(enabled);
#endif
}
bool boardEpdPowerAvailable() {
#if FREEINK_DEVICE_METALIO_EINK4
  if (BoardConfig::isMetalioEInk4()) return true;
#endif
#if FREEINK_DEVICE_PAPERMONO || FREEINK_DEVICE_WS397
  return true;
#else
  return freeink_board_epd_power != nullptr;
#endif
}
void boardEpdReset(bool high) {
#if FREEINK_DEVICE_PAPERMONO
  freeink::papermono::setEpdReset(high);
#else
  if (freeink_board_epd_reset) freeink_board_epd_reset(high);
#endif
}

#if FREEINK_CAP_DISPLAY_PROBE_POLICY
// The bound on the idle check below: the X3 stock protocol's bounded BUSY wait
// after its reset.
constexpr unsigned long CACHED_IDLE_WAIT_MS = 300;
// A UC BUSY_N wait this long is no refresh (a FULL takes seconds): the panel is
// not the UC part the driver was chosen for.
constexpr unsigned long UC_WAIT_CAP_MS = 60000;

// DisplayProbeConfig::cacheAcrossDeepSleep: a deep-sleep wake takes the
// controller from RTC memory with no bus traffic, so a panel swapped while the
// board slept (3V3 stays live through deep sleep) would meet the wrong driver,
// and an SSD1677 never ends a UC8279 BUSY_N wait. The first panel reset of such a
// boot checks the controller: a UC8279 idles BUSY_N HIGH and an SSD1677 idles
// BUSY LOW (a missing panel reads idle through the driver's pull). One read when
// it matches; a level that stays wrong for CACHED_IDLE_WAIT_MS drops the cached
// verdict and restarts, and the restart probes the panel before any command
// reaches it.
void checkCachedController(const int8_t busyPin, const freeink::BusyPolarity p) {
  static bool checked = false;
  if (checked || busyPin < 0 || !freeink::displayControllerCached()) return;
  checked = true;
  if (p != freeink::BusyPolarity::UcIdleHigh && p != freeink::BusyPolarity::ActiveHigh) return;
  const int idle = p == freeink::BusyPolarity::UcIdleHigh ? HIGH : LOW;
  const unsigned long start = millis();
  while (digitalRead(busyPin) != idle) {
    if (millis() - start >= CACHED_IDLE_WAIT_MS) {
      if (Serial) {
        Serial.printf("[%lu] [EPD] BUSY stays %s after the reset: not the %s probed before the sleep; restarting\n",
                      millis(), idle == HIGH ? "LOW" : "HIGH",
                      BoardConfig::controllerPartName(BoardConfig::ACTIVE.displayController));
      }
      freeink::forgetCachedDisplayController();
      esp_restart();
    }
    delay(1);
  }
}
#endif
}  // namespace

namespace freeink {

// ── ISR-driven waveform-completion notification ──────────────────────────────
// A single binary semaphore, shared between the BUSY-pin GPIO ISR and
// waitRefreshComplete(). The ISR is attached only for the duration of one
// refresh wait (and only after the waveform is confirmed running), so it fires
// on the real completion edge, not on the idle->busy transition or SPI noise.
// File-static so the plain-C ISR can reach it; only one panel is ever active at
// a time, so a single instance is safe. DRAM_ATTR keeps it out of flash for the
// IRAM_ATTR ISR. Ported from the CrossPoint community-sdk EInkDisplay.
static DRAM_ATTR SemaphoreHandle_t s_epdRefreshDone = nullptr;

static void IRAM_ATTR epdBusyIsr() {
  if (!s_epdRefreshDone) return;
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_epdRefreshDone, &woken);
  if (woken) portYIELD_FROM_ISR();
}

#if FREEINK_DIAGNOSTICS
// Written by the task driving the panel, read by any task (fields may be one
// wait apart).
static EpdBusDiagnostics s_busDiag;
static unsigned long s_refreshStartMs = 0;

static void recordWait(const unsigned long start, const bool timedOut, const char* tag) {
  ++s_busDiag.waits;
  if (timedOut) ++s_busDiag.timeouts;
  s_busDiag.lastWaitMs = millis() - start;
  s_busDiag.lastWaitTimedOut = timedOut;
  s_busDiag.lastWaitTag = tag;
}

// Close the refresh noteRefresh() opened, once its completion wait has ended.
static void closeRefresh() {
  if (!s_busDiag.refreshInFlight) return;
  s_busDiag.refreshInFlight = false;
  s_busDiag.lastRefreshMs = millis() - s_refreshStartMs;
  s_busDiag.lastRefreshTimedOut = s_busDiag.lastWaitTimedOut;
}

void EpdBus::noteRefresh(const uint8_t mode, const uint8_t sequence, const bool async, const bool turnOff,
                         const bool customLut) {
  s_refreshStartMs = millis();
  ++s_busDiag.refreshes;
  s_busDiag.lastRefreshMode = mode;
  s_busDiag.lastRefreshSequence = sequence;
  s_busDiag.lastRefreshAsync = async;
  s_busDiag.lastRefreshTurnOff = turnOff;
  s_busDiag.lastRefreshCustomLut = customLut;
  s_busDiag.refreshInFlight = true;
}

EpdBusDiagnostics EpdBus::diagnostics() { return s_busDiag; }
#endif

void EpdBus::begin(const EpdPins& pins, uint32_t spiHz, BusyPolarity busy, int8_t spiMiso, int8_t coCs) {
  _pins = pins;
  _spiHz = spiHz;
  _busy = busy;
  _coCs = coCs;
  _spi = SPISettings(spiHz, MSBFIRST, SPI_MODE0);

  // One-shot semaphore backing waitRefreshComplete()'s ISR wait (created once).
  if (!s_epdRefreshDone) s_epdRefreshDone = xSemaphoreCreateBinary();

  // Power the EPD rail first (boards that gate it, e.g. Sticky's EP_PWR_EN), so the
  // panel is alive before SPI bring-up and the reset pulse. No-op when unassigned.
  // gpio_hold_dis first: PowerManager::powerDownRailsForSleep() holds this pin LOW
  // for deep sleep, and the hold survives the wake reset — without releasing it,
  // the HIGH write silently bounces off the latch and the rail stays off.
  if (pins.powerEnable >= 0) {
    gpio_hold_dis(static_cast<gpio_num_t>(pins.powerEnable));
    pinMode(pins.powerEnable, OUTPUT);
    digitalWrite(pins.powerEnable, HIGH);
    delay(100);
  } else if (boardEpdPowerAvailable()) {
    boardEpdPower(true);
    delay(100);
  }

  // Deselect the panel before SPI takes the clock pin: preset CS HIGH, then enable
  // its driver, so CS is never driven LOW (GPIO_OUT resets to 0) or left floating
  // while SCLK is live. Also releases a sleep hold on CS.
  if (pins.cs >= 0) {
    gpio_hold_dis(static_cast<gpio_num_t>(pins.cs));
    gpio_set_level(static_cast<gpio_num_t>(pins.cs), 1);
    pinMode(pins.cs, OUTPUT);
    digitalWrite(pins.cs, HIGH);
  }

  SPI.begin(pins.sclk, spiMiso, pins.mosi, pins.cs);

  pinMode(pins.cs, OUTPUT);
  pinMode(pins.dc, OUTPUT);
  // Release any deep-sleep hold on RST. powerDownRailsForSleep() holds it HIGH
  // while the panel rail remains powered, or LOW alongside a gated-off rail;
  // either hold survives the wake reset and would make the reset pulse below
  // bounce off the latch. The HIGH level is preset before the driver is enabled
  // (GPIO_OUT resets to 0) and the hold is released last, so RST never glitches
  // LOW ahead of the reset pulse. Boards routing RST through an IO expander leave
  // the pin unassigned and reset via the freeink_board_epd_reset hook instead.
  if (pins.rst >= 0) {
    const auto rst = static_cast<gpio_num_t>(pins.rst);
    gpio_set_level(rst, 1);
    pinMode(pins.rst, OUTPUT);
    digitalWrite(pins.rst, HIGH);
    gpio_hold_dis(rst);
  }
  if (busy == BusyPolarity::ActiveLow) {
    pinMode(pins.busy, INPUT_PULLUP);
  } else if (busy == BusyPolarity::ActiveHigh && pins.busyPullDown) {
    pinMode(pins.busy, INPUT_PULLDOWN);  // no external pull: a missing panel reads not-busy
  } else if (busy == BusyPolarity::UcIdleHigh && BoardConfig::displayBusyPullUp()) {
    pinMode(pins.busy, INPUT_PULLUP);  // the same for BUSY_N, whose wait has no timeout
  } else {
    pinMode(pins.busy, INPUT);
  }
  if (_coCs >= 0) {
    pinMode(_coCs, OUTPUT);
    digitalWrite(_coCs, HIGH);
  }
  digitalWrite(pins.cs, HIGH);
  digitalWrite(pins.dc, HIGH);
}

void EpdBus::reset(uint16_t extraSettleMs) {
  if (BoardConfig::isOnePage()) {
    // OnePage shares GPIO27 between EPD reset and the SD/MIC power rail.
    // Pulsing it low after Storage.begin() cuts power to the mounted SD card,
    // causing subsequent SD reads to fail. Keep the rail powered; SSD1677
    // performs CMD_SOFT_RESET immediately after.
    if (_pins.rst >= 0) {
      digitalWrite(_pins.rst, HIGH);
    }
    return;
  }

  const auto setReset = [this](bool high) {
    if (_pins.rst >= 0) {
      digitalWrite(_pins.rst, high ? HIGH : LOW);
    } else {
      boardEpdReset(high);
    }
  };
  setReset(true);
  delay(10);
  setReset(false);
  delay(10);
  setReset(true);
  delay(10);
  if (extraSettleMs) {
    delay(extraSettleMs);
  }
#if FREEINK_CAP_DISPLAY_PROBE_POLICY
  checkCachedController(_pins.busy, _busy);
#endif
}

void EpdBus::cmd(uint8_t c) {
  SPI.beginTransaction(_spi);
  digitalWrite(_pins.dc, LOW);
  digitalWrite(_pins.cs, LOW);
  SPI.transfer(c);
  digitalWrite(_pins.cs, HIGH);
  SPI.endTransaction();
}

void EpdBus::data(uint8_t d) {
  SPI.beginTransaction(_spi);
  digitalWrite(_pins.dc, HIGH);
  digitalWrite(_pins.cs, LOW);
  SPI.transfer(d);
  digitalWrite(_pins.cs, HIGH);
  SPI.endTransaction();
}

void EpdBus::data(const uint8_t* d, uint16_t len) {
  SPI.beginTransaction(_spi);
  digitalWrite(_pins.dc, HIGH);
  digitalWrite(_pins.cs, LOW);
  SPI.writeBytes(d, len);
  digitalWrite(_pins.cs, HIGH);
  SPI.endTransaction();
}

void EpdBus::cmdData(uint8_t c, const uint8_t* d, uint16_t len) {
  SPI.beginTransaction(_spi);
  digitalWrite(_pins.cs, LOW);
  digitalWrite(_pins.dc, LOW);
  SPI.transfer(c);
  if (len > 0 && d != nullptr) {
    digitalWrite(_pins.dc, HIGH);
    SPI.writeBytes(d, len);
  }
  digitalWrite(_pins.cs, HIGH);
  SPI.endTransaction();
}

void EpdBus::cmdData2(uint8_t c, uint8_t d0, uint8_t d1) {
  const uint8_t d[2] = {d0, d1};
  cmdData(c, d, 2);
}

void EpdBus::beginTxn() {
  if (_coCs >= 0) {
    digitalWrite(_coCs, HIGH);
  }
  SPI.beginTransaction(_spi);
  digitalWrite(_pins.cs, LOW);
}

void EpdBus::endTxn() {
  digitalWrite(_pins.cs, HIGH);
  SPI.endTransaction();
}

void EpdBus::rawCmd(uint8_t c) {
  digitalWrite(_pins.dc, LOW);
  SPI.transfer(c);
  digitalWrite(_pins.dc, HIGH);
}

void EpdBus::rawData(uint8_t d) {
  digitalWrite(_pins.dc, HIGH);
  SPI.transfer(d);
}

void EpdBus::rawWriteBytes(const uint8_t* d, uint16_t len) {
  digitalWrite(_pins.dc, HIGH);
  SPI.writeBytes(d, len);
}

void EpdBus::waitBusy(const char* tag) { waitBusy(_busy, tag); }

void EpdBus::waitBusy(BusyPolarity p, const char* tag) {
  const unsigned long start = millis();
  // Both hooks engage lazily, only once the wait has proven long (see
  // setBusyWaitHooks). longWait gates the slice hook independently of the
  // begin hook's presence; hookFired guarantees the end hook is balanced.
  bool longWait = false;
  bool hookFired = false;
  bool x3SawLow = false;

  if (p == BusyPolarity::ActiveHigh) {
    while (digitalRead(_pins.busy) == HIGH) {
      busyIdle(longWait, HIGH, 1);
      if (!longWait && millis() - start > BUSY_WAIT_HOOK_THRESHOLD_MS) {
        longWait = true;
        if (_busyWaitBeginHook != nullptr) {
          hookFired = true;
          _busyWaitBeginHook();
        }
      }
      if (millis() - start > 30000) break;
    }
  } else if (p == BusyPolarity::ActiveLow) {
    bool busy = digitalRead(_pins.busy) == LOW;
    if (!busy) {
      while (millis() - start < 100) {
        if (digitalRead(_pins.busy) == LOW) {
          busy = true;
          break;
        }
        delay(1);
      }
    }
    if (busy) {
      do {
        busyIdle(longWait, LOW, 10);
        if (!longWait && millis() - start > BUSY_WAIT_HOOK_THRESHOLD_MS) {
          longWait = true;
          if (_busyWaitBeginHook != nullptr) {
            hookFired = true;
            _busyWaitBeginHook();
          }
        }
        if (millis() - start > 30000) break;
      } while (digitalRead(_pins.busy) == LOW);
    }
  } else if (p == BusyPolarity::UcIdleHigh) {
    // X4 Pro UC8179/UC8279 production behavior: allow one RTOS tick for the
    // command to take effect, then wait only for the documented idle level.
    // There is deliberately no fixed millisecond timeout on this controller
    // path; issuing the next command while BUSY_N is still LOW can make the UC
    // controller discard plane or LUT writes. The display-probe policy alone caps
    // it far beyond any refresh: a panel that is not the UC part (an SSD1677
    // idles LOW) would otherwise hold the CPU awake until the cell is flat.
    delay(1);
    while (digitalRead(_pins.busy) == LOW) {
      busyIdle(longWait, LOW, 1);
      if (!longWait && millis() - start > BUSY_WAIT_HOOK_THRESHOLD_MS) {
        longWait = true;
        if (_busyWaitBeginHook != nullptr) {
          hookFired = true;
          _busyWaitBeginHook();
        }
      }
#if FREEINK_CAP_DISPLAY_PROBE_POLICY
      if (millis() - start > UC_WAIT_CAP_MS) {
        if (Serial) {
          Serial.printf("[%lu] [EPD] BUSY_N still LOW after %lu ms%s: not a UC panel?\n", millis(), UC_WAIT_CAP_MS,
                        tag ? tag : "");
        }
        freeink::forgetCachedDisplayController();
        break;
      }
#endif
    }
  } else {  // X3TwoPhase: wait for the LOW edge, then wait back to HIGH
    while (digitalRead(_pins.busy) == HIGH) {
      delay(1);
      if (millis() - start > 1000) break;
    }
    if (digitalRead(_pins.busy) == LOW) {
      x3SawLow = true;
      while (digitalRead(_pins.busy) == LOW) {
        busyIdle(longWait, LOW, 1);
        if (!longWait && millis() - start > BUSY_WAIT_HOOK_THRESHOLD_MS) {
          longWait = true;
          if (_busyWaitBeginHook != nullptr) {
            hookFired = true;
            _busyWaitBeginHook();
          }
        }
        if (millis() - start > 30000) break;
      }
    }
  }

  if (hookFired && _busyWaitEndHook != nullptr) _busyWaitEndHook();
#if FREEINK_DIAGNOSTICS
  // Every loop above ends on its idle level unless it gave up (UcIdleHigh only
  // with the display-probe policy's cap).
  const int busyLevel = digitalRead(_pins.busy);
  recordWait(start,
             p == BusyPolarity::ActiveHigh
                 ? busyLevel == HIGH
                 : (p != BusyPolarity::UcIdleHigh || FREEINK_CAP_DISPLAY_PROBE_POLICY) && busyLevel == LOW,
             tag);
#endif
  if (p == BusyPolarity::X3TwoPhase && !x3SawLow) return;

  if (tag && Serial) {
    Serial.printf("[%lu]   Wait complete: %s (%lu ms)\n", millis(), tag, millis() - start);
  }
}

void EpdBus::waitRefreshComplete(const char* tag) {
  // The X4 Pro UC production wait is level-based, not edge-qualified. Keep the
  // same one-tick/idle-HIGH rule for refresh completion so a missed assertion
  // edge can never make the caller write RAM while the waveform is still busy.
  if (_busy == BusyPolarity::UcIdleHigh) {
    waitBusy(_busy, tag);
#if FREEINK_DIAGNOSTICS
    closeRefresh();
#endif
    return;
  }
  // A host that installed a busy-wait slice hook (e.g. CrossPoint light-sleeping
  // through the refresh) must keep the polling path: waitBusy() invokes the slice
  // hook on each idle step, while this ISR path sleeps the task on a semaphore and
  // never calls it. Bypassing the hook costs that host its power policy (~9% more
  // per refresh, measured ~29 mC vs ~26.5 mC on X3), and is a latent hazard: edge
  // interrupts do not fire during light sleep, so a completion edge taken while the
  // host is slept would be missed and the wait would stall to its 30 s timeout. The
  // slice hook already delivers GPIO-precise wake, so the ISR path buys these hosts
  // nothing — fall back to the hooked poll.
  if (_busyWaitSliceHook != nullptr) {
    // Refresh-completion context: BUSY assertion can trail MASTER_ACTIVATION
    // by a few microseconds, and the ActiveHigh polled path (unlike ActiveLow's
    // 100 ms grace loop, or the ISR path's 20 ms edge wait below) would fall
    // through immediately — returning mid-waveform, after which single-buffer
    // drivers rewrite controller RAM while the panel is still driving. Give
    // the poll the same bounded grace here, in the refresh-only context, so
    // waitBusy() itself (which also serves command waits where BUSY may never
    // assert) stays untouched for every other caller.
    if (_busy == BusyPolarity::ActiveHigh) {
      const unsigned long graceStart = millis();
      while (digitalRead(_pins.busy) != HIGH && millis() - graceStart < 20) {
        delayMicroseconds(200);
      }
    }
    waitBusy(tag);
#if FREEINK_DIAGNOSTICS
    closeRefresh();
#endif
    return;
  }
  // ISR-driven completion wait: sleep the task on a semaphore and wake on the
  // exact BUSY completion edge, instead of polling every 1 ms. Falls back to
  // polling if the semaphore could not be created.
  if (!s_epdRefreshDone) {
    waitBusy(tag);
#if FREEINK_DIAGNOSTICS
    closeRefresh();
#endif
    return;
  }
  // Levels by polarity. CHANGE is armed so both the delayed BUSY assertion and
  // its completion are event-driven; the task never polls during either phase.
  const bool activeHigh = (_busy == BusyPolarity::ActiveHigh);
  const int doneLevel = activeHigh ? LOW : HIGH;
  const int workingLevel = activeHigh ? HIGH : LOW;
  const unsigned long start = millis();

  xSemaphoreTake(s_epdRefreshDone, 0);  // drain any stale token
  attachInterrupt(digitalPinToInterrupt(_pins.busy), epdBusyIsr, CHANGE);

  bool sawWorking = digitalRead(_pins.busy) == workingLevel;
  if (!sawWorking) {
    // BUSY assertion can trail MASTER_ACTIVATION by a few microseconds. Sleep
    // until an edge instead of delay(1) polling. No edge in 20 ms means the
    // command was a no-op or its entire pulse completed before we armed.
    if (xSemaphoreTake(s_epdRefreshDone, pdMS_TO_TICKS(20)) != pdTRUE) {
      detachInterrupt(digitalPinToInterrupt(_pins.busy));
#if FREEINK_DIAGNOSTICS
      recordWait(start, false, tag);
      closeRefresh();
#endif
      return;
    }
    sawWorking = digitalRead(_pins.busy) == workingLevel;
    if (!sawWorking && digitalRead(_pins.busy) == doneLevel) {
      detachInterrupt(digitalPinToInterrupt(_pins.busy));
#if FREEINK_DIAGNOSTICS
      recordWait(start, false, tag);
      closeRefresh();
#endif
      return;
    }
  }

  // Long sleep — fire the power hooks (if any) around it, matching the poll path.
  const bool hook = (_busyWaitBeginHook != nullptr);
  if (hook) _busyWaitBeginHook();
  while (digitalRead(_pins.busy) != doneLevel) {
    if (xSemaphoreTake(s_epdRefreshDone, pdMS_TO_TICKS(30000)) != pdTRUE) break;
  }
  if (hook && _busyWaitEndHook != nullptr) _busyWaitEndHook();

  detachInterrupt(digitalPinToInterrupt(_pins.busy));
#if FREEINK_DIAGNOSTICS
  recordWait(start, digitalRead(_pins.busy) != doneLevel, tag);  // still working: the 30 s wait gave up
  closeRefresh();
#endif
  if (tag && Serial) {
    Serial.printf("[%lu]   Wait complete: %s (%lu ms)\n", millis(), tag, millis() - start);
  }
}

void EpdBus::sendPlaneFlipped(uint8_t ramCmd, const uint8_t* plane, uint16_t height, uint16_t widthBytes) {
  cmd(ramCmd);  // own CS pulse
  beginTxn();   // single CS-low burst for the whole plane
  for (int y = static_cast<int>(height) - 1; y >= 0; y--) {
    rawWriteBytes(plane + static_cast<uint32_t>(y) * widthBytes, widthBytes);
  }
  endTxn();
}

void EpdBus::sendPlaneFlippedInverted(uint8_t ramCmd, const uint8_t* plane, uint16_t height,
                                      uint16_t widthBytes) {
  // Streams ~plane[i] row-reversed without a host-side copy of the inverted
  // frame (the C3 boards have no RAM to spare for one). Same single CS-low
  // burst contract as sendPlaneFlipped().
  uint8_t chunk[128];
  cmd(ramCmd);
  beginTxn();
  for (int y = static_cast<int>(height) - 1; y >= 0; y--) {
    const uint8_t* row = plane + static_cast<uint32_t>(y) * widthBytes;
    uint16_t offset = 0;
    while (offset < widthBytes) {
      const uint16_t n = static_cast<uint16_t>(widthBytes - offset) < sizeof(chunk)
                             ? static_cast<uint16_t>(widthBytes - offset)
                             : static_cast<uint16_t>(sizeof(chunk));
      for (uint16_t i = 0; i < n; ++i) chunk[i] = static_cast<uint8_t>(~row[offset + i]);
      rawWriteBytes(chunk, n);
      offset = static_cast<uint16_t>(offset + n);
    }
  }
  endTxn();
}

void EpdBus::fillPlane(uint8_t ramCmd, uint8_t fillByte, uint16_t height, uint16_t widthBytes) {
  // Reusable constant-fill chunk; wide rows are written in multiple bursts so a
  // widthBytes larger than the chunk is streamed in full (no silent truncation).
  uint8_t chunk[128];
  memset(chunk, fillByte, sizeof(chunk));
  cmd(ramCmd);
  beginTxn();
  for (uint16_t y = 0; y < height; y++) {
    uint16_t remaining = widthBytes;
    while (remaining) {
      const uint16_t n = remaining < sizeof(chunk) ? remaining : static_cast<uint16_t>(sizeof(chunk));
      rawWriteBytes(chunk, n);
      remaining = static_cast<uint16_t>(remaining - n);
    }
  }
  endTxn();
}

}  // namespace freeink
