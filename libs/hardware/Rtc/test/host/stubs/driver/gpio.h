#pragma once
// SharedI2c.h's stuck-bus recovery (the shared line's Rtc.cpp includes it): pin calls that do nothing on the host.
using gpio_num_t = int;
inline void gpio_hold_dis(int) {}
struct gpio_config_t {
  unsigned long long pin_bit_mask;
  int mode, pull_up_en, pull_down_en, intr_type;
};
constexpr int GPIO_MODE_INPUT = 1, GPIO_MODE_INPUT_OUTPUT_OD = 7, GPIO_PULLUP_DISABLE = 0, GPIO_PULLDOWN_DISABLE = 0,
              GPIO_INTR_DISABLE = 0;
inline void gpio_set_level(int, int) {}
inline int gpio_get_level(int) { return 1; }
inline int gpio_config(const gpio_config_t*) { return 0; }
