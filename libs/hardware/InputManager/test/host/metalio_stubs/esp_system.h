#pragma once
enum esp_reset_reason_t { ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_SW, ESP_RST_PANIC, ESP_RST_DEEPSLEEP };
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
