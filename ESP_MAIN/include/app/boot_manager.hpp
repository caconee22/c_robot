#pragma once

#include "robot_types.hpp"

// Contract
// Role: boot order from early pin safety to every module begin().
// Called from: setup() only. Not for run-time re-init or SW1 recovery.
// Failure: motor/watchdog init failure blocks driving; color sensor and
//   Raspberry Pi failures do not fail boot.
// Safety code: yes. Keep the init order.
class BootManager {
 public:
  static BootResult begin();
  static void staySafe();
};
