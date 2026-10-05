#pragma once
#include "Arduino.h"
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) fake::enterCritical()
#define portEXIT_CRITICAL(mux) fake::exitCritical()
#define portENTER_CRITICAL_ISR(mux) fake::enterCritical()
#define portEXIT_CRITICAL_ISR(mux) fake::exitCritical()
#define pdTRUE 1
#define portMAX_DELAY 0xFFFFFFFFU
#define pdMS_TO_TICKS(ms) (ms)
