#pragma once
#include <BoardConfig.h>

#include <cstdint>

// FreeInk SDK — battery monitor.
//
// Two backends behind one API (chosen at compile time, so construction and the
// public methods below are identical for both):
//   * ADC (default) — reads a divided LiPo voltage off an ADC pin and maps it to
//     a percentage; an optional charge-status pin (MCP73832 /STAT, active-LOW)
//     drives isCharging().
//   * I2C fuel gauge (FREEINK_BATTERY_I2C_GAUGE) — reads SoC/voltage/charge from a
//     BQ27220 gauge (+ optional BQ25896 charger) over I2C; the ADC pin/divider are
//     ignored. Used by X3 and LilyGo T5 S3. Config comes from
//     BoardConfig::ACTIVE.batteryGauge, and gauge-vs-ADC is chosen at *runtime*
//     (gaugeAddr != 0), so X3 (gauge) and X4 (ADC) work from one C3 binary.
class BatteryMonitor {
public:
    static constexpr int8_t PIN_NONE = -1;

    struct Status {
        bool supported = false;
        bool percentageKnown = false;
        bool millivoltsKnown = false;
        bool chargingKnown = false;
        bool externalPowerKnown = false;
        uint16_t percentage = 0;
        uint16_t millivolts = 0;
        bool charging = false;
        bool externalPower = false;
        // Raw M5PM1 telemetry for diagnostics; -1 when not read (non-PM1 boards
        // or failed I/O). VIN is the DC input rail, 5VINOUT the bidirectional
        // USB-C rail, powerSource the PM1's PWR_SRC bitmap (reg 0x04 [2:0]:
        // bit0=5VIN, bit1=5VINOUT, bit2=BAT).
        int32_t pm1VinMv = -1;
        int32_t pm1VinOutMv = -1;
        int16_t pm1PowerSource = -1;
    };

    // Uses BoardConfig::ACTIVE's battery pins/backend.
    BatteryMonitor();

    // Optional divider multiplier defaults to 2.0; optional charge-status pin
    // defaults to unused.
    explicit BatteryMonitor(int8_t adcPin, float dividerMultiplier = 2.0f, int8_t chargeStatusPin = PIN_NONE);

    // Read voltage and return percentage (0-100)
    uint16_t readPercentage() const;

    // Like readPercentage(), but reports whether the read succeeded. An I2C gauge
    // can fail transiently; on failure this returns false and leaves `out`
    // unchanged, so a caller can keep its last good value. The ADC path always
    // succeeds.
    bool readPercentageChecked(uint16_t& out) const;

    // Read every battery field the active board can report. `supported` is false
    // when the board profile has no battery telemetry path. Per-field `Known`
    // flags distinguish a valid false/zero value from unsupported or failed I/O.
    Status readStatus() const;

    // Read the battery voltage in millivolts (accounts for divider)
    uint16_t readMillivolts() const;

    // Read the battery voltage in volts (accounts for divider)
    double readVolts() const;

    // True when the battery is actively charging. Sources by backend:
    //   * ADC boards: the MCP73832-style charge-status pin (LOW = charging);
    //     always false when no charge-status pin is configured.
    //   * Gauge boards: a charger IC's status (BQ25896 CHRG_STAT) when present,
    //     else the gauge's own Current() sign (BQ27220, positive = charging), so
    //     a board with a gauge but no charger IC (e.g. X3) still reports it.
    bool isCharging() const;

    // True when external power (a USB cable or charger) is physically present.
    //
    // This is NOT isCharging(): a full battery stops charging while still
    // plugged in, so isCharging() goes false with the cable still attached.
    // Only a source that reports the input rail itself can answer this: the
    // BQ25896's REG0B (VBUS_STAT[7:5] plus PG_STAT), the M5PM1's PWR_SRC, or an
    // analog charge-status ladder (charger idle, charging or complete = present;
    // on battery = not present, though a cable too weak to carry the load may
    // still be attached; unknown or the no-battery blink = no opinion).
    //
    // `known` (optional, out) is set false when the board has no way to tell,
    // or the read failed. Callers MUST branch on it: a bare false means "no
    // opinion" just as often as it means "unplugged", and treating the two the
    // same is how you get a device that thinks the cable is gone because it
    // never had a way to see it.
    bool isExternalPowerPresent(bool* known = nullptr) const;

    // Like readMillivolts(), averaged over `samples` ADC reads on the ADC backend
    // (a high-impedance divider reads noisy single samples). Other backends read
    // once.
    uint16_t readMillivoltsAveraged(uint8_t samples) const;

    // Low-battery cutoff (BoardConfig::PowerConfig::lowBatteryCutoffMv, ADC
    // backend only). Takes one averaged reading per call, at most one per second
    // (a sooner call repeats the last verdict), and returns true once
    // CUTOFF_SAMPLES consecutive readings are at or below the cutoff. It never
    // fires while the charge-status ladder reports external power (USB idle,
    // charging, complete or the no-battery blink), nor on a reading below
    // CELL_PRESENT_MIN_MV (no cell: P+ reads about 0 V on USB). The divider taps
    // P+ behind the protection FETs and reads low under load, so with `radioOn`
    // a reading is judged against the cutoff minus
    // PowerConfig::cutoffRadioAllowanceMv. Call from one task. `known` is false
    // when no reading was judged against the cutoff (no cutoff, external power,
    // no cell); `millivolts` receives the last reading.
    static constexpr uint8_t CUTOFF_SAMPLES = 3;
    static constexpr uint16_t CELL_PRESENT_MIN_MV = 2500;
    bool isBelowCutoff(bool* known = nullptr, uint16_t* millivolts = nullptr, bool radioOn = false) const;

    // Analog charge-status ladder (BoardConfig::ChargeStatusLadder). poll takes
    // one sample when one is due (not before settleAfterBootMs of uptime, then at
    // most every minIntervalMs) and updates the decoded state; call it from the
    // main loop only. chargeStatus() returns the cached state without touching
    // the ADC, so any task may call it. Unknown on boards without a ladder.
    // With a lowBatteryCutoffMv, a decoded Charging or Complete while P+ stays at
    // or below the cutoff without rising for 5 min (sampled once a minute) is
    // reported as OnBattery: a real charge lifts P+.
    static void pollChargeStatus();
    static BoardConfig::ChargeState chargeStatus();
    // True once the ladder has decoded a state this boot and then lost it
    // (ChargeStatusLadder::unknownHoldSamples readings outside every window).
    static bool chargeStatusLost();

#if FREEINK_DIAGNOSTICS
    // Charge-status ladder internals, for a bring-up screen that must not take
    // its own samples (the ladder is sampled at most once per minIntervalMs).
    // Reads cached state only; any task may call it.
    struct ChargeLadderDiagnostics {
      using State = BoardConfig::ChargeState;
      bool sampled = false;              // at least one sample taken this boot
      uint32_t lastSampleMs = 0;         // millis() of the last sample
      State decoded = State::Unknown;    // decoder output
      State published = State::Unknown;  // chargeStatus()
      State candidate = State::Unknown;  // awaiting debounce
      uint8_t candidateRun = 0;          // agreeing readings for `candidate`
      uint8_t completeRun = 0;           // consecutive Complete-window readings
      uint8_t unknownRun = 0;            // consecutive readings outside every window
      bool everDecoded = false;          // a state was decoded this boot
      bool lost = false;                 // chargeStatusLost()
      bool disproved = false;            // the P+ cross-check publishes Charging/Complete as OnBattery
      uint8_t trendSamples = 0;          // readings in the open P+ cross-check window
      uint16_t trendStartMv = 0;         // P+ at the start of that window
      uint32_t lastMv = 0;               // last oversampled pin reading
      uint32_t samples = 0;              // samples taken this boot
    };
    static ChargeLadderDiagnostics chargeLadderDiagnostics();
#endif

    // Percentage from a millivolt value, off a standard 1S Li-ion discharge
    // curve. The result is always a multiple of 10: voltage cannot resolve a
    // Li-ion pack any finer than that, and pretending otherwise just produces a
    // number that wanders while the battery sits still. 0 mV (a failed read)
    // maps to 0%, not 100%.
    static uint16_t percentageFromMillivolts(uint16_t millivolts);

    // Same lookup, with a deadband around the notch boundaries so a reading that
    // hovers on a boundary keeps reporting `previousPercent` instead of
    // oscillating. Pass the last value this returned; pass > 100 for no history.
    static uint16_t percentageFromMillivolts(uint16_t millivolts, uint16_t previousPercent);

private:
    bool hasAdcBackend() const;
    bool hasGaugeBackend() const;
    bool hasM5Pm1Backend() const;
    bool readM5Pm1Status(Status& status) const;

    int8_t _adcPin;
    float _dividerMultiplier;
    int8_t _chargeStatusPin;
};
