#pragma once

#include "robot_types.hpp"

// Contract
// Role: turn new raw samples into reliable floor colors for the four corners
//   (FL, FR, RL, RR). Only a new sequence counts; a color is published after
//   config::sensors::COLOR_CONFIRM_COUNT equal results.
// Called from: update() once per loop pass; snapshot() before RobotFsm.
// Failure: a broken channel stays Unknown; it does not stop boot or driving.
class SensorManager {
 public:
  static bool begin();
  static void update();
  static SensorSnapshot snapshot();
};
