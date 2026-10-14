#pragma once

#include "robot_types.hpp"

// Contract
// Role: convert a DriveIntent into a left/right MotorCommand with a
//   config::control::COMMAND_VALID_MS deadline. Pure calculation, no I/O.
// Called from: loop() right after RobotFsm::update().
class DriveLogic {
 public:
  static MotorCommand makeCommand(uint32_t nowMs, const DriveIntent& intent);
};
