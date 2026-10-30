#pragma once
#include <Arduino.h>
using esp_timer_handle_t = void*;
struct esp_timer_create_args_t { void (*callback)(void*) = nullptr; void* arg = nullptr; int dispatch_method = 0; const char* name = nullptr; bool skip_unhandled_events = false; };
constexpr int ESP_OK = 0, ESP_TIMER_TASK = 0;
inline int64_t esp_timer_get_time() { return int64_t(fakeNow) * 1000; }
inline int esp_timer_create(const esp_timer_create_args_t*, esp_timer_handle_t* timer) { static int token; *timer = &token; return ESP_OK; }
inline int esp_timer_stop(esp_timer_handle_t) { return ESP_OK; }
inline int esp_timer_start_periodic(esp_timer_handle_t, uint64_t) { return ESP_OK; }
