#pragma once
#include "freertos/FreeRTOS.h"
struct StaticSemaphore_t { bool held = false; };
using SemaphoreHandle_t = StaticSemaphore_t*;
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* s) { return s; }
inline int xSemaphoreTake(SemaphoreHandle_t s, uint32_t) { s->held = true; return pdTRUE; }
inline void xSemaphoreGive(SemaphoreHandle_t s) { s->held = false; }
