// BOTTOM, SIDE, RELEASED_MV and the 2-sample debounce below are copies of
// SILKSCREEN.adcLadders in libs/hardware/BoardConfig/include/BoardConfig.h:
// change both together, or these tests stop checking the windows the board runs.

#include <cstdio>

#include "../../include/AdcLadderDecode.h"

namespace {

using freeink::input_detail::ADC_LADDER_RELEASED;
using freeink::input_detail::ADC_LADDER_UNCLEAN;
using freeink::input_detail::AdcLadderDebounce;
using freeink::input_detail::decodeAdcLadder;

int checksRun = 0;
int checksFailed = 0;

#define CHECK(condition)                                               \
  do {                                                                 \
    ++checksRun;                                                       \
    if (!(condition)) {                                                \
      ++checksFailed;                                                  \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition); \
    }                                                                  \
  } while (0)

struct Key {
  uint16_t minMv;
  uint16_t maxMv;
  uint8_t button;
};

constexpr uint8_t BACK = 0, CONFIRM = 1, LEFT = 2, RIGHT = 3, UP = 4, DOWN = 5;
constexpr int RAW_MID = 2000;  // any unsaturated raw reading
constexpr uint16_t RELEASED_MV = 3000;

// Mirrors BoardConfig.h SILKSCREEN.adcLadders (silkscreen-pcb HARDWARE.md 9.1).
constexpr Key BOTTOM[] = {{0, 200, RIGHT}, {1090, 1290, LEFT}, {2075, 2325, CONFIRM}, {2680, 2935, BACK}};
constexpr Key SIDE[] = {{0, 200, DOWN}, {1690, 1920, UP}, {2400, 2665, DOWN}, {2745, 2995, UP}};

int bottom(int mv, int raw = RAW_MID) { return decodeAdcLadder(BOTTOM, 4, mv, raw, RELEASED_MV); }
int side(int mv, int raw = RAW_MID) { return decodeAdcLadder(SIDE, 4, mv, raw, RELEASED_MV); }

void testNominalLevels() {
  CHECK(bottom(33) == RIGHT);
  CHECK(bottom(1185) == LEFT);
  CHECK(bottom(2200) == CONFIRM);
  CHECK(bottom(2800) == BACK);
  CHECK(bottom(3300) == ADC_LADDER_RELEASED);

  CHECK(side(33) == DOWN);    // DOWN(1)
  CHECK(side(1800) == UP);    // UP(1)
  CHECK(side(2533) == DOWN);  // DOWN(2)
  CHECK(side(2877) == UP);    // UP(2)
  CHECK(side(3300) == ADC_LADDER_RELEASED);
}

void testSaturatedIdleIsReleased() {
  // Idle 3.3 V top-codes; a calibration that maps full scale below the released
  // threshold must still read as released.
  CHECK(bottom(2950, 4095) == ADC_LADDER_RELEASED);
  CHECK(side(2950, 4090) == ADC_LADDER_RELEASED);
  CHECK(side(2950, 4089) == UP);
}

void testChords() {
  // Ladder 1: LEFT+BACK aliases to LEFT; the others fall in reject gaps.
  CHECK(bottom(1113) == LEFT);
  CHECK(bottom(1004) == ADC_LADDER_UNCLEAN);  // LEFT+OK
  CHECK(bottom(953) == ADC_LADDER_UNCLEAN);   // LEFT+OK+BACK
  CHECK(bottom(1966) == ADC_LADDER_UNCLEAN);  // OK+BACK
  // Ladder 2: all three chords are rejected at nominal.
  CHECK(side(1667) == ADC_LADDER_UNCLEAN);  // UP(1)+UP(2), same logical key either way
  CHECK(side(1545) == ADC_LADDER_UNCLEAN);  // UP(1)+DOWN(2)
  CHECK(side(2276) == ADC_LADDER_UNCLEAN);  // DOWN(2)+UP(2)
  // Any chord with a 100 ohm key reads as that key.
  CHECK(bottom(32) == RIGHT);
  CHECK(side(32) == DOWN);
}

void testWindowEdges() {
  CHECK(bottom(-20) == RIGHT);  // zero-offset reading below the bottom window
  CHECK(bottom(0) == RIGHT);
  CHECK(bottom(200) == RIGHT);
  CHECK(bottom(201) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(1089) == ADC_LADDER_UNCLEAN);  // keeps LEFT+OK (<= ~1089 mV) rejected
  CHECK(bottom(1090) == LEFT);
  CHECK(bottom(1290) == LEFT);
  CHECK(bottom(1291) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(2074) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(2075) == CONFIRM);
  CHECK(bottom(2325) == CONFIRM);
  CHECK(bottom(2679) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(2680) == BACK);
  CHECK(bottom(2935) == BACK);
  CHECK(bottom(2936) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(2999) == ADC_LADDER_UNCLEAN);
  CHECK(bottom(3000) == ADC_LADDER_RELEASED);

  CHECK(side(1689) == ADC_LADDER_UNCLEAN);
  CHECK(side(1690) == UP);
  CHECK(side(1920) == UP);
  CHECK(side(2399) == ADC_LADDER_UNCLEAN);
  CHECK(side(2400) == DOWN);
  CHECK(side(2665) == DOWN);
  CHECK(side(2744) == ADC_LADDER_UNCLEAN);
  CHECK(side(2745) == UP);
  CHECK(side(2995) == UP);
  CHECK(side(2996) == ADC_LADDER_UNCLEAN);
}

void testWorstCaseCorners() {
  // +/-1% resistors, +/-1.5% rail, +/-50 mV ADC (REQUIREMENTS btn-windows-*).
  CHECK(bottom(1102) == LEFT);
  CHECK(bottom(1268) == LEFT);
  CHECK(bottom(2103) == CONFIRM);
  CHECK(bottom(2298) == CONFIRM);
  CHECK(bottom(2699) == BACK);
  CHECK(bottom(2901) == BACK);
  CHECK(side(1706) == UP);
  CHECK(side(1894) == UP);
  CHECK(side(2433) == DOWN);
  CHECK(side(2632) == DOWN);
  CHECK(side(2776) == UP);
  CHECK(side(2978) == UP);
}

void testReleasedThresholdDisabled() {
  // releasedMinMv 0: only a saturated raw reading is released.
  CHECK(decodeAdcLadder(BOTTOM, 4, 3300, RAW_MID, 0) == ADC_LADDER_UNCLEAN);
  CHECK(decodeAdcLadder(BOTTOM, 4, 3300, 4095, 0) == ADC_LADDER_RELEASED);
}

void testDebounce() {
  AdcLadderDebounce d;
  CHECK(d.update(ADC_LADDER_RELEASED, 2) == ADC_LADDER_RELEASED);
  CHECK(d.update(LEFT, 2) == ADC_LADDER_RELEASED);  // one sample is not enough
  CHECK(d.update(LEFT, 2) == LEFT);
  CHECK(d.update(LEFT, 2) == LEFT);

  // An unclean sample keeps the committed key and breaks the run.
  CHECK(d.update(ADC_LADDER_UNCLEAN, 2) == LEFT);
  CHECK(d.update(ADC_LADDER_RELEASED, 2) == LEFT);
  CHECK(d.update(ADC_LADDER_UNCLEAN, 2) == LEFT);
  CHECK(d.update(ADC_LADDER_RELEASED, 2) == LEFT);
  CHECK(d.update(ADC_LADDER_RELEASED, 2) == ADC_LADDER_RELEASED);

  // Alternating decodes never commit.
  CHECK(d.update(BACK, 2) == ADC_LADDER_RELEASED);
  CHECK(d.update(CONFIRM, 2) == ADC_LADDER_RELEASED);
  CHECK(d.update(BACK, 2) == ADC_LADDER_RELEASED);
  CHECK(d.update(BACK, 2) == BACK);

  // A run interrupted by an unclean sample restarts.
  AdcLadderDebounce e;
  CHECK(e.update(RIGHT, 2) == ADC_LADDER_RELEASED);
  CHECK(e.update(ADC_LADDER_UNCLEAN, 2) == ADC_LADDER_RELEASED);
  CHECK(e.update(RIGHT, 2) == ADC_LADDER_RELEASED);
  CHECK(e.update(RIGHT, 2) == RIGHT);
}

}  // namespace

int main() {
  testNominalLevels();
  testSaturatedIdleIsReleased();
  testChords();
  testWindowEdges();
  testWorstCaseCorners();
  testReleasedThresholdDisabled();
  testDebounce();
  std::printf("%d checks, %d failed\n", checksRun, checksFailed);
  return checksFailed == 0 ? 0 : 1;
}
