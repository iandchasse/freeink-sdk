#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using StaticSemaphore_t = int;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
constexpr int pdTRUE = 1;
#define pdMS_TO_TICKS(ms) (ms)
