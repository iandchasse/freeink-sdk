#pragma once
using gpio_num_t=int;
inline void gpio_hold_dis(int) {}
inline void gpio_hold_en(int) {}
// BoardConfig.h's pin-park helpers (unused by Picco's profile).
enum gpio_mode_t { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 };
inline void gpio_set_level(int, int) {}
inline void gpio_set_direction(int, gpio_mode_t) {}
inline void gpio_pullup_dis(int) {}
inline void gpio_pulldown_dis(int) {}
struct gpio_config_t {
  unsigned long long pin_bit_mask;
  int mode, pull_up_en, pull_down_en, intr_type;
};
constexpr int GPIO_MODE_INPUT_OUTPUT_OD = 7, GPIO_PULLUP_DISABLE = 0, GPIO_PULLDOWN_DISABLE = 0, GPIO_INTR_DISABLE = 0;
inline int gpio_config(const gpio_config_t*) { return 0; }
inline int gpio_get_level(int) { return 1; }
