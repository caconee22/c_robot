#pragma once

#include "robot_types.hpp"

class SensorManager {
 public:
  static bool begin();
  static void update();
  static SensorSnapshot snapshot();
};
