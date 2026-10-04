#pragma once
// SharedAdc.h takes the SDK's ADC lock; the host test is single-threaded.
using TickType_t = unsigned;
constexpr int pdTRUE = 1;
constexpr TickType_t portMAX_DELAY = ~0u;
