#pragma once
#include <Arduino.h>

// Contract
// Role: bench-only I2C scan. Probes the main bus and, if a TCA9548A answers
//   at 0x70..0x77, each of its eight channels, to show whether the mux is
//   there and which devices sit behind which channel.
// Called from: BenchV2App console ("I2C SCAN") and loop (step, report).
// Timing: at most PROBES_PER_STEP address-only probes per loop, each bounded
//   by the Wire timeout. The mux is set again at the start of every step
//   because ColorSensorManager reselects channels between loops.
// Failure: three bus errors in a row (not a missing device) abort the scan
//   and report I2C ERR. Never touches motors, SystemManager or sensor state.
// Output (one line per loop, only when UART has room):
//   I2C MAIN <hex addresses...>   devices with every mux channel off
//   I2C MUX <hex>|NONE            first mux found
//   I2C CH <n> <hex addresses...> devices only seen on that channel (8 lines)
//   I2C DONE <ms> | I2C ERR BUS <hex address>
namespace BenchI2cScan {
void start(uint32_t nowMs);
bool active();
void step(uint32_t nowMs);
void report();
}
