#pragma once
enum esp_reset_reason_t { ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT,
  ESP_RST_SW, ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT,
  ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT };
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
#include <stdint.h>
inline uint32_t esp_random() { static uint32_t x = 12345; x = x * 1103515245u + 12345u; return x >> 8; }
