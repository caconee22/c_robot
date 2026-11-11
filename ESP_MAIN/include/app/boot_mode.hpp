#pragma once

#include <Arduino.h>

// Contract
// Role (11/11, 학현): one firmware holds both the match FSM and the bench
//   (tests, color calibration). Which one runs is chosen once at power-on:
//   SW1 and SW2 both held for BENCH_HOLD_MS from boot -> bench, otherwise
//   match. Released earlier (or not pressed) -> match, so a normal power-on
//   is never delayed.
// Called from: setup() only, before anything else touches the buttons. Puts
//   the motor pins in their early safe state first. Waits at most
//   BENCH_HOLD_MS, and only while both buttons are down.
// Safety: nothing can drive here; the buttons still held when the chosen
//   mode starts are ignored until released (button_interrupts.hpp).
namespace config {
namespace boot_mode {
constexpr uint32_t BENCH_HOLD_MS = 1000;
constexpr uint32_t POLL_MS = 10;
}  // namespace boot_mode
}  // namespace config

namespace BootMode {
// true = both buttons held for BENCH_HOLD_MS right from power-on.
bool benchRequested();
}  // namespace BootMode
