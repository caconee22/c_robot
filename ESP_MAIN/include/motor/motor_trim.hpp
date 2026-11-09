#pragma once

#include <Arduino.h>

// Contract
// Role: left/right motor output trim so the robot drives straight (11/09: it
//   curves a little on a straight command). Each side's output is multiplied
//   by its trim / 1000 inside MotorController::apply, after the output limit
//   and before the motor inversion, so every mode (match FSM, bench) uses it.
//   Reduce-only: 800..1000 permille. Slow the stronger side; the limit never
//   grows and the safety paths (brake, watchdog, SW1) do not use it.
// Storage: NVS namespace "motor", keys "trimL"/"trimR". Read once by
//   MotorController::begin; the bench sets it ("TRIM <L> <R>", RAM only) and
//   saves it ("TRIM SAVE"). The match firmware only reads it.
// Failure: NVS missing/unreadable or a value out of range -> 1000 (no trim).
// Note: one factor per side; the curve may differ at 40 % and 71 % output and
//   below the stall duty, and a mechanical cause (belt, bearing) comes first.
namespace MotorTrim {
constexpr uint16_t MIN_PERMILLE = 800;
constexpr uint16_t MAX_PERMILLE = 1000;

void load();                                // NVS -> RAM; 1000 when missing/bad
uint16_t left();
uint16_t right();
bool set(uint16_t leftPermille, uint16_t rightPermille);  // RAM; false when out of range
bool save();                                // RAM -> NVS; false when the write failed
int16_t scale(int16_t value, uint16_t trimPermille);
}  // namespace MotorTrim
