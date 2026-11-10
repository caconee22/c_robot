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
// 11/10 field: upside down it tripped only after a long time, and not at all
//   while the wheels were driving. Causes: HOLD_MS was 3 s, and every single
//   raw sample under the limit (motor vibration, rocking, a wheel bump) reset
//   the hold. Now the gravity vector is low-pass filtered (FILTER_WEIGHT) and
//   the hold is only cleared after RELEASE_MS under the limit.
// Failure: no fresh IMU data -> no judgment (never trips without data) but the
//   hold is kept, so a short IMU gap while flipped does not restart it. A
//   missing or broken IMU never stops the robot by itself. Mounting direction
//   does not matter: level is learned, not assumed.
// Values: start values (lift attacks must stay under HOLD_MS past LIMIT_DEG).

namespace config {
namespace tilt {
constexpr float LIMIT_DEG = 80.0f;
constexpr uint32_t HOLD_MS = 1000;      // 11/10: was 3000
constexpr uint32_t RELEASE_MS = 200;    // under the limit this long clears the hold
constexpr float FILTER_WEIGHT = 0.15f;  // per 100 Hz sample, ~60 ms smoothing
constexpr uint8_t LEVEL_SAMPLES = 10;   // fresh samples while stopped before level is trusted
}  // namespace tilt
}  // namespace config

namespace TiltGuard {
void begin();
// Returns true while the tilt has been past the limit for HOLD_MS (running only).
bool update(uint32_t nowMs, bool running, const ImuStatus& status, const ImuSample& sample);
float lastAngleDeg();   // angle from level of the last judged sample, for tests and logs
}  // namespace TiltGuard
