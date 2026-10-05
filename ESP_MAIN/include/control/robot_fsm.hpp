#pragma once

#include "robot_types.hpp"

class RobotFsm {
 public:
  static void begin();
  static DriveIntent update(uint32_t nowMs, const SensorSnapshot& sensors,
                            const VisionSnapshot& vision,
                            const SafetyStatus& safety);
  static RobotState state();
};
