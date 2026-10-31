#pragma once
#include <Arduino.h>

// Contract
// Role: temporary UART software SW1/SW2 for robot_bench_v2_manual only.
// Called from: BenchV2App console/begin/update/stop. One SW2 command produces
// a finite press and release through ButtonInterrupts' unchanged debounce.
// Failure: battery/fault/calibration/mode change/STOP cancel pending starts.
// SW1 uses the permanent emergency path; there is no software release/reset.
// Match builds do not define BENCH_SOFTWARE_SWITCHES or link this module.
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
namespace BenchSoftwareSwitches {
void begin();
bool pressStart(uint32_t nowMs);
void update(uint32_t nowMs);
void cancelStart();
void emergencyStop();
}
#endif
