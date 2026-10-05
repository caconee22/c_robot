#pragma once

#include <esp_system.h>

class ResetManager {
 public:
  static void begin();
  static esp_reset_reason_t reason();
  static bool requiresRearm();
};
