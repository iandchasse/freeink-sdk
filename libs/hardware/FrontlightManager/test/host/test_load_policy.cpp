// Host test for the front-light load record (FrontlightLoad.h): which string a
// switch-on lights, what a probe trip does to the record, when the other string is
// tried, what Restore Light on Wake may do, and the RTC-memory form.
#include <FrontlightLoad.h>

#include <cstdio>
#include <cstdlib>

namespace fl = freeink::frontlight;

namespace {

int failures = 0;

void check(const bool ok, const char* what) {
  if (ok) return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}

constexpr bool WARM = true;
constexpr bool COOL = false;

void defaults() {
  const fl::LoadRecord r;
  check(r.load == fl::Load::Unknown, "a new record knows nothing");
  check(fl::targetWarm(r, true, WARM) == WARM, "warm asked, warm tried");
  check(fl::targetWarm(r, true, COOL) == COOL, "cool asked, cool tried");
  check(fl::targetWarm(r, false, WARM) == COOL, "no colour select: the one string");
  check(fl::restoreAllowed(r), "restore allowed with nothing known");
}

void bothStringsLight() {
  fl::LoadRecord r;
  fl::noteLit(r, WARM);
  check(r.load == fl::Load::Present && r.warm == fl::StringState::Present, "warm lit");
  check(r.cool == fl::StringState::Unknown, "cool not probed yet");
  fl::noteLit(r, COOL);
  check(r.cool == fl::StringState::Present, "cool lit on a colour change");
  check(fl::targetWarm(r, true, WARM) == WARM, "both there: warm stays warm");
}

// A cool-only strip at the default warmth (warm asked).
void coolOnlyStrip() {
  fl::LoadRecord r;
  check(fl::noteTripped(r, true, WARM, false), "warm tripped: try cool in the same switch-on");
  check(r.load == fl::Load::Unknown, "nothing decided before the second try");
  fl::noteOtherLit(r, WARM);
  check(r.warm == fl::StringState::Missing && r.cool == fl::StringState::Present, "warm missing, cool present");
  check(r.load == fl::Load::Present, "load present");
  check(fl::targetWarm(r, true, WARM) == COOL, "warm asked again: cool lights, never the missing warm");
  check(fl::targetWarm(r, true, COOL) == COOL, "cool asked: cool");
  check(fl::restoreAllowed(r), "restore allowed with a light there");

  // The strip is unplugged: cool trips, and warm is known missing.
  check(!fl::noteTripped(r, true, COOL, false), "no second try onto a missing string");
  check(r.load == fl::Load::Absent, "no light");
  check(r.warm == fl::StringState::Missing && r.cool == fl::StringState::Unknown,
        "warm stays missing, cool is no longer confirmed");
  check(!fl::restoreAllowed(r), "no restore into nothing");
  check(fl::targetWarm(r, true, WARM) == COOL, "the user's retry tries cool, never the missing warm");

  // The strip is plugged back in: the retry lights cool, and warm stays missing.
  fl::noteLit(r, COOL);
  check(r.load == fl::Load::Present && r.warm == fl::StringState::Missing, "cool lit again, warm still missing");
}

void noLightAtAll() {
  fl::LoadRecord r;
  check(fl::noteTripped(r, true, WARM, false), "warm tripped: try cool");
  check(!fl::noteTripped(r, true, COOL, true), "the second try tripped too");
  check(r.load == fl::Load::Absent, "no light");
  check(r.warm == fl::StringState::Unknown && r.cool == fl::StringState::Unknown, "no string marked missing");
  check(!fl::restoreAllowed(r), "restore blocked");
  // The user's retry that finds a light clears it.
  fl::noteLit(r, WARM);
  check(r.load == fl::Load::Present && fl::restoreAllowed(r), "a light found on retry");
}

// Lit on cool; the user moves warmth to warm, which is not fitted.
void colourChangeOntoMissingString() {
  fl::LoadRecord r;
  fl::noteLit(r, COOL);
  check(fl::noteTripped(r, true, WARM, false), "warm tripped after the change: back to cool");
  fl::noteOtherLit(r, WARM);
  check(r.warm == fl::StringState::Missing && r.cool == fl::StringState::Present, "warm marked missing");
  check(fl::targetWarm(r, true, WARM) == COOL, "later warm requests light cool");
}

void singleStringBoard() {
  fl::LoadRecord r;
  check(!fl::noteTripped(r, false, COOL, false), "no colour select: nothing else to try");
  check(r.load == fl::Load::Absent, "no light");
}

void inUseTrip() {
  fl::LoadRecord r;
  fl::noteLit(r, WARM);
  fl::noteLit(r, COOL);
  fl::noteInUseTrip(r, WARM);
  check(r.warm == fl::StringState::Unknown && r.cool == fl::StringState::Present, "the lit string unconfirmed");
  check(r.load == fl::Load::Unknown, "not a no-light result");
  check(fl::restoreAllowed(r), "an in-use trip does not block restore (the consumer clears its saved on)");
}

// Colour blending: each string lit through a probe first, the dominant one first.
void blending() {
  fl::LoadRecord r;
  auto plan = fl::colourPlan(r, true, true, 70, false, false);
  check(!plan.blend && plan.warm == WARM, "nothing lit yet: the dominant warm string first");
  fl::noteLit(r, WARM);
  plan = fl::colourPlan(r, true, true, 70, false, true);
  check(!plan.blend && plan.warm == COOL, "warm lit: cool next, still one string");
  fl::noteLit(r, COOL);
  plan = fl::colourPlan(r, true, true, 70, true, true);
  check(plan.blend && plan.warm == WARM, "both lit: blend, from the dominant warm string");
  plan = fl::colourPlan(r, true, true, 30, true, true);
  check(plan.blend && plan.warm == COOL, "a cool mix starts from cool");
  check(!fl::colourPlan(r, true, true, 0, true, true).blend, "0 %: fixed cool");
  check(!fl::colourPlan(r, true, true, 100, true, true).blend, "100 %: fixed warm");
  check(fl::colourPlan(r, true, true, 100, true, true).warm == WARM, "100 % lights warm");
  check(!fl::colourPlan(r, true, false, 70, true, true).blend, "no blend carrier: fixed");
  check(fl::colourPlan(r, true, false, 70, true, true).warm == WARM, "fixed at >= 50 %: warm");
  check(!fl::colourPlan(r, false, true, 70, true, true).blend, "no colour select: never blends");
  plan = fl::colourPlan(r, true, true, 30, false, true);
  check(!plan.blend && plan.warm == COOL, "cool dominant and not lit since the reset: cool first");

  // A cool-only light: no blend, warmth lights cool.
  fl::LoadRecord coolOnly;
  check(fl::noteTripped(coolOnly, true, WARM, false), "setup");
  fl::noteOtherLit(coolOnly, WARM);
  check(!fl::blendPossible(coolOnly, true, true), "a string missing: no blend");
  plan = fl::colourPlan(coolOnly, true, true, 70, true, false);
  check(!plan.blend && plan.warm == COOL, "warm missing: a warm mix lights cool");
}

void rtcForm() {
  fl::LoadRecord r;
  check(fl::noteTripped(r, true, WARM, false), "setup");
  fl::noteOtherLit(r, WARM);
  const fl::PackedRecord packed = fl::pack(r);
  fl::LoadRecord back;
  check(fl::unpack(packed, back), "a packed record reads back");
  check(back.load == r.load && back.cool == r.cool && back.warm == r.warm, "same fields");
  check(fl::targetWarm(back, true, WARM) == COOL, "a deep-sleep wake keeps warm missing");

  fl::PackedRecord garbage = packed;
  garbage.magic ^= 1u;
  fl::LoadRecord untouched;
  check(!fl::unpack(garbage, untouched) && untouched.load == fl::Load::Unknown, "wrong magic rejected");
  garbage = packed;
  garbage.check ^= 0x10u;
  check(!fl::unpack(garbage, untouched), "wrong check byte rejected");
  garbage = packed;
  garbage.load = 7;
  garbage.check = static_cast<uint8_t>(garbage.load ^ garbage.cool ^ garbage.warm ^ 0xA5);
  check(!fl::unpack(garbage, untouched), "out-of-range field rejected");

  check(!fl::keepRecord(fl::ResetKind::Cold), "a cold boot starts empty");
  check(fl::keepRecord(fl::ResetKind::DeepSleepWake), "a deep-sleep wake keeps the record");
  check(fl::keepRecord(fl::ResetKind::SoftwareRestart), "a software restart keeps the record");
}

}  // namespace

int main() {
  defaults();
  bothStringsLight();
  coolOnlyStrip();
  noLightAtAll();
  colourChangeOntoMissingString();
  singleStringBoard();
  inUseTrip();
  blending();
  rtcForm();
  if (failures != 0) {
    std::printf("%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("front-light load record: all checks passed\n");
  return EXIT_SUCCESS;
}
