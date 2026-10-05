#pragma once
#include "freertos/FreeRTOS.h"
struct StaticTask_t {};
using StackType_t = uint8_t;
using TaskHandle_t = StaticTask_t*;
inline TaskHandle_t xTaskCreateStatic(void (*)(void*), const char*, uint32_t,
                                     void*, int, StackType_t*, StaticTask_t* task) {
  fake::watchdogEnabled = true;
  return task;
}
inline void vTaskDelay(uint32_t ms) { delay(ms); }
