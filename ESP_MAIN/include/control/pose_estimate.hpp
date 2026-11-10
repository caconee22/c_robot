#pragma once

#include <Arduino.h>

#include "sensors/imu_monitor.hpp"

// Contract
// Role: rough robot pose for the run log only (11/10). Not used by the FSM or
//   any motor control (AGENTS: no closed-loop control from the IMU).
//   - heading: gyro rate about the "up" axis integrated from the SW2 start
//     (0 deg there, left turn positive). "Up" is the resting gravity vector
//     learned while stopped, so the IMU mounting does not matter; the gyro
//     bias is learned while stopped too. Drifts a few deg per minute.
//   - x, y: dead reckoning from the commanded wheel outputs (after limit and
//     trim) times MM_PER_S_PER_PERMILLE, along the heading. Very rough: slip,
//     being pushed, battery voltage and the groove step are not seen.
//     x forward at the start, y to the left.
// Called from: loop() once per pass (update), RunLog (pose). Pure math on
//   values the caller passes; no hardware access, never blocks.
// Failure: no fresh IMU data -> heading holds (imuOk false), x/y keep using it.

namespace config {
namespace pose {
// Speed at output 1000, not measured yet (start value): 1000 -> 1000 mm/s.
constexpr float MM_PER_S_PER_PERMILLE = 1.0f;
constexpr float BIAS_WEIGHT = 0.02f;      // gyro bias / gravity average while stopped
constexpr uint32_t MAX_STEP_MS = 100;     // longer gaps are not integrated
}  // namespace pose
}  // namespace config

struct Pose {
  float headingDeg = 0.0f;
  float xMm = 0.0f;
  float yMm = 0.0f;
  bool imuOk = false;
};

namespace PoseEstimate {
void reset();
// running: motors allowed (a run). The first running pass zeroes the pose.
void update(uint32_t nowMs, bool running, const ImuStatus& imu, const ImuSample& sample,
            int16_t leftPermille, int16_t rightPermille);
Pose pose();
}  // namespace PoseEstimate
