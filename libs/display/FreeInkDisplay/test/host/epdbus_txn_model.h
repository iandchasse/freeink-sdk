#pragma once
// The SPI-transaction contract of the real EpdBus, as a model for the host tests' recording buses.
//
// EpdBus::cmd(), data(), cmdData(), cmdData2() and the plane streamers each open a Transaction of their
// own, and beginTxn() returns one that stays open until end() or the end of its scope. A Transaction is
// SPI.beginTransaction(), and arduino-esp32 takes a NON-recursive mutex there (SPIClass::paramLock, then the
// HAL's spi->lock) that endTransaction() gives back. A bus call made while a beginTxn() Transaction is still
// alive therefore blocks its task for ever: no panic, no watchdog, a frozen screen. The model counts such
// calls (and names the first) instead of hanging, and keeps the order of everything that went over the bus,
// so a test can assert both.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

namespace freeink {

struct BusEvent {
  enum Kind : char {
    Cmd = 'c',      // EpdBus::cmd()
    Data = 'd',     // EpdBus::data()
    CmdData = 'm',  // EpdBus::cmdData() and cmdData2()
    Plane = 'p',    // sendPlaneFlipped(), sendPlaneFlippedInverted() and fillPlane()
    Open = '(',     // beginTxn(): a Transaction opens, CS goes low
    Write = 'w',    // Transaction::writeBytes()
    End = ')',      // the Transaction ends, CS goes high
  };
  Kind kind;
  int value;                   // the command byte, or -1
  size_t size;                 // payload bytes
  std::vector<uint8_t> bytes;  // Write and CmdData: the payload
};

class EpdBusTxnModel {
 public:
  int alive = 0;                      // beginTxn() Transactions open now
  int maxAlive = 0;                   // the most that were open at once
  unsigned opened = 0;                // beginTxn() Transactions opened
  unsigned nestedCalls = 0;           // bus calls made while one was open: each blocks for ever on hardware
  const char* firstNested = nullptr;  // the first of them
  int firstNestedCmd = -1;            // its command byte, when it had one
  std::vector<BusEvent> log;          // everything that went over the bus, in order

  bool csLow() const { return alive > 0; }  // beginTxn() asserts CS and its end() releases it

  // A bus call that opens a transaction of its own (or the beginTxn() that opens a held one).
  void call(BusEvent::Kind kind, const char* what, int cmd = -1, const uint8_t* p = nullptr, size_t n = 0) {
    if (alive > 0 && nestedCalls++ == 0) {
      firstNested = what;
      firstNestedCmd = cmd;
      std::fprintf(stderr,
                   "EpdBus::%s(0x%02X) while a beginTxn() Transaction is alive: the task would block for ever "
                   "in SPI.beginTransaction()\n",
                   what, cmd & 0xFF);
    }
    BusEvent e{kind, cmd, n, {}};
    if (p != nullptr && kind != BusEvent::Plane) e.bytes.assign(p, p + n);
    log.push_back(std::move(e));
  }

  void clear() {
    maxAlive = alive;
    nestedCalls = 0;
    firstNested = nullptr;
    firstNestedCmd = -1;
    log.clear();
  }
};

// EpdBus::Transaction: move-only, ends at end() or its destructor.
class EpdBusTxn {
 public:
  EpdBusTxn() = default;
  explicit EpdBusTxn(EpdBusTxnModel* model) : _m(model), _active(model != nullptr) {
    if (!_active) return;
    _m->call(BusEvent::Open, "beginTxn");
    ++_m->opened;
    if (++_m->alive > _m->maxAlive) _m->maxAlive = _m->alive;
  }
  ~EpdBusTxn() { end(); }
  EpdBusTxn(const EpdBusTxn&) = delete;
  EpdBusTxn& operator=(const EpdBusTxn&) = delete;
  EpdBusTxn(EpdBusTxn&& o) noexcept : _m(o._m), _active(o._active) {
    o._m = nullptr;
    o._active = false;
  }
  EpdBusTxn& operator=(EpdBusTxn&& o) noexcept {
    if (this != &o) {
      end();
      _m = o._m;
      _active = o._active;
      o._m = nullptr;
      o._active = false;
    }
    return *this;
  }

  void end() {
    if (!_active || _m == nullptr) return;
    _active = false;
    --_m->alive;
    _m->log.push_back({BusEvent::End, -1, 0, {}});
  }
  void writeBytes(const uint8_t* d, size_t n) {
    if (_active && _m != nullptr) _m->log.push_back({BusEvent::Write, -1, n, std::vector<uint8_t>(d, d + n)});
  }

 private:
  EpdBusTxnModel* _m = nullptr;
  bool _active = false;
};

}  // namespace freeink
