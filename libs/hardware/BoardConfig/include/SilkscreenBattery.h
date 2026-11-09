#pragma once

#include <cstdint>

// Display estimation only; these do not configure the charger or cutoff.
namespace BoardConfig::silkscreen_battery {
constexpr uint16_t CAPACITY_MAH = 500;
constexpr uint16_t CHARGE_CURRENT_MA = 250;
constexpr uint16_t DEFAULT_STEP_MV = 150;
// Ramp S toward S * factor over the first 20 minutes; 1000 = 1.0.
constexpr uint16_t STEP_CREEP_PERMILLE = 1000;
static_assert(CAPACITY_MAH > 0 && CHARGE_CURRENT_MA > 0, "valid charge rate");
}  // namespace BoardConfig::silkscreen_battery
