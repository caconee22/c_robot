#pragma once

#include <esp_system.h>

// Contract
// Role: report the ESP reset reason and whether a new SW2 gesture is needed.
// Called from: BootManager, SystemManager. Never triggers a reset itself.
class ResetManager {
 public:
  static void begin();
  static esp_reset_reason_t reason();
  static bool requiresRearm();
};
