#pragma once
using gpio_num_t = int;
inline void gpio_hold_dis(int) {}
inline void gpio_hold_en(int) {}

constexpr int GPIO_MODE_OUTPUT = 1;
inline void gpio_set_level(int, int) {}
inline void gpio_set_direction(int, int) {}

constexpr int GPIO_MODE_INPUT = 2;
inline void gpio_pullup_dis(int) {}
inline void gpio_pulldown_dis(int) {}
