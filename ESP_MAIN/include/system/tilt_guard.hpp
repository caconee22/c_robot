#pragma once

#include <Arduino.h>

#include "sensors/imu_monitor.hpp"

// Contract
// Role: rollover check for the match firmware (2026-11-07 학현 decision). While
//   stopped it learns "level" from the IMU gravity vector; while running it
//   reports a trip once the robot stayed tilted past LIMIT_DEG from that level
//   for HOLD_MS. The caller turns a trip into the normal permanent emergency
//   stop (SystemManager::triggerEmergencyStop), so recovery is a reboot.
// Called from: loop() after ImuMonitor::update, before the second
//   SystemManager::update. Pure math on the latest sample; no I2C, no waiting.
// Failure: no fresh IMU data -> no judgment and the hold timer restarts, so a
//   missing or broken IMU never stops the robot by itself (it only loses this
//   check). Mounting direction does not matter: level is learned, not assumed.
// Values: start values, not yet checked on the robot (lift attacks must stay
//   under HOLD_MS).

namespace config {
namespace tilt {
constexpr float LIMIT_DEG = 80.0f;
constexpr uint32_t HOLD_MS = 3000;
constexpr uint8_t LEVEL_SAMPLES = 10;   // fresh samples while stopped before level is trusted
}  // namespace tilt
}  // namespace config

namespace TiltGuard {
void begin();
// Returns true while the tilt has been past the limit for HOLD_MS (running only).
bool update(uint32_t nowMs, bool running, const ImuStatus& status, const ImuSample& sample);
float lastAngleDeg();   // angle from level of the last judged sample, for tests and logs
}  // namespace TiltGuard
