#pragma once
// Host build: the task watchdog does nothing.
#include <stdint.h>
typedef int esp_err_t;
inline esp_err_t esp_task_wdt_init(uint32_t, bool) { return 0; }
inline esp_err_t esp_task_wdt_add(void*) { return 0; }
inline esp_err_t esp_task_wdt_reset() { return 0; }
