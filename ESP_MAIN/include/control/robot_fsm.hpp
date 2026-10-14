#pragma once

#include "robot_types.hpp"

// Contract
// Role: decide what to do (DriveIntent) from floor colors, vision and safety.
//   Priority: safety stop > hazard-color escape > attack sequence > vision
//   tracking > search. No GPIO, I2C or UART access.
// Called from: loop() once per non-calibration pass.
// Tuning values: config::control (control_config.hpp).
class RobotFsm {
 public:
  static void begin();
  static DriveIntent update(uint32_t nowMs, const SensorSnapshot& sensors,
                            const VisionSnapshot& vision,
                            const SafetyStatus& safety);
  static RobotState state();
};
