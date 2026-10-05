#pragma once

#include "robot_types.hpp"

class DriveLogic {
 public:
  static MotorCommand makeCommand(uint32_t nowMs, const DriveIntent& intent);
};
