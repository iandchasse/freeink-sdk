#pragma once

// FreeInk SDK — the front-light load record and its rules
// (FrontlightConfig::loadProbeMs, FREEINK_CAP_FRONTLIGHT_PROBE).
//
// Plain data and decisions with no hardware access, so a host test can run them
// (test/host/test_load_policy.cpp). FrontlightManager owns the one record, updates
// it from the switch-on probe and the in-use monitor, and keeps it in RTC memory
// across deep sleep and software restarts; a cold boot starts it empty.

#include <cstdint>

namespace freeink {
namespace frontlight {

// What the last switch-on found.
enum class Load : uint8_t {
  Unknown,  // nothing probed since the record was last cleared
  Present,  // a string lit
  Absent,   // every string the switch-on could try tripped: no light connected
};

// One LED string behind the colour select (cool, or warm).
enum class StringState : uint8_t {
  Unknown,
  Present,  // lit through a probe
  Missing,  // tripped while the other string lit: a single-colour light
};

struct LoadRecord {
  Load load = Load::Unknown;
  StringState cool = StringState::Unknown;
  StringState warm = StringState::Unknown;

  StringState& string(const bool isWarm) { return isWarm ? warm : cool; }
  StringState string(const bool isWarm) const { return isWarm ? warm : cool; }
};

// The string a switch-on lights: the requested one, unless it is missing. Without
// a colour select there is one string, which the record calls cool.
constexpr bool targetWarm(const LoadRecord& r, const bool colourSelect, const bool requestedWarm) {
  if (!colourSelect) return false;
  return r.string(requestedWarm) == StringState::Missing ? !requestedWarm : requestedWarm;
}

// The probe on `warm` lit its string.
inline void noteLit(LoadRecord& r, const bool warm) {
  r.string(warm) = StringState::Present;
  r.load = Load::Present;
}

// The probe on `warm` tripped. Returns true when the other string is to be tried
// in the same switch-on: a colour select, this was not already that second try,
// and the other string is not known to be missing. Otherwise nothing could light,
// so the record says Absent and forgets both strings: the next switch-on the user
// asks for tries them again.
inline bool noteTripped(LoadRecord& r, const bool colourSelect, const bool warm, const bool secondTry) {
  if (colourSelect && !secondTry && r.string(!warm) != StringState::Missing) return true;
  r = LoadRecord{};
  r.load = Load::Absent;
  return false;
}

// The second try, on the other string, lit after `trippedWarm` tripped.
inline void noteOtherLit(LoadRecord& r, const bool trippedWarm) {
  r.string(trippedWarm) = StringState::Missing;
  noteLit(r, !trippedWarm);
}

// The 100 ms monitor tripped after the probe had passed on `warm`: that string is
// no longer confirmed, and the next switch-on probes again.
inline void noteInUseTrip(LoadRecord& r, const bool warm) {
  r.string(warm) = StringState::Unknown;
  r.load = Load::Unknown;
}

// Restore Light on Wake may switch the light on at boot unless the last switch-on
// found nothing (a cold boot has cleared the record, so it tries once).
constexpr bool restoreAllowed(const LoadRecord& r) { return r.load != Load::Absent; }

// How the chip came out of reset, as far as the record is concerned.
enum class ResetKind : uint8_t { Cold, DeepSleepWake, SoftwareRestart };
constexpr bool keepRecord(const ResetKind kind) { return kind != ResetKind::Cold; }

// RTC-memory form: a magic word, then the three fields, checked on the way back.
constexpr uint32_t RECORD_MAGIC = 0x464c4431;  // "FLD1"
struct PackedRecord {
  uint32_t magic;
  uint8_t load;
  uint8_t cool;
  uint8_t warm;
  uint8_t check;  // the three bytes above, XOR 0xA5
};

constexpr PackedRecord pack(const LoadRecord& r) {
  const auto load = static_cast<uint8_t>(r.load);
  const auto cool = static_cast<uint8_t>(r.cool);
  const auto warm = static_cast<uint8_t>(r.warm);
  return PackedRecord{RECORD_MAGIC, load, cool, warm, static_cast<uint8_t>(load ^ cool ^ warm ^ 0xA5)};
}

// False (and `out` untouched) for anything pack() did not write, e.g. the
// uninitialised RTC memory of a power-on.
inline bool unpack(const PackedRecord& p, LoadRecord& out) {
  if (p.magic != RECORD_MAGIC || p.check != static_cast<uint8_t>(p.load ^ p.cool ^ p.warm ^ 0xA5)) return false;
  if (p.load > static_cast<uint8_t>(Load::Absent) || p.cool > static_cast<uint8_t>(StringState::Missing) ||
      p.warm > static_cast<uint8_t>(StringState::Missing)) {
    return false;
  }
  out.load = static_cast<Load>(p.load);
  out.cool = static_cast<StringState>(p.cool);
  out.warm = static_cast<StringState>(p.warm);
  return true;
}

}  // namespace frontlight
}  // namespace freeink
