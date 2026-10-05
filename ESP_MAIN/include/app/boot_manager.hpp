#pragma once

#include "robot_types.hpp"

class BootManager {
 public:
  static BootResult begin();
  static void staySafe();
};
