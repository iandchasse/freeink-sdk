#pragma once
using gpio_num_t = int;
enum gpio_mode_t { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 };
void gpio_hold_dis(gpio_num_t);
inline void gpio_hold_en(gpio_num_t) {}
inline void gpio_set_level(gpio_num_t, int) {}
inline void gpio_set_direction(gpio_num_t, gpio_mode_t) {}
inline void gpio_pullup_dis(gpio_num_t) {}
inline void gpio_pulldown_dis(gpio_num_t) {}
