#pragma once

using gpio_num_t = int;
inline bool gpioHeld[49] = {};
inline void gpio_hold_dis(int pin) { gpioHeld[pin] = false; }
inline void gpio_hold_en(int pin) { gpioHeld[pin] = true; }
// BoardConfig.h's pin-park helpers (the Silkscreen profile's). This header shadows
// metalio_stubs/driver/gpio.h on the eego test's include path, so it carries the
// same declarations.
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
