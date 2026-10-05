#include "hardware/reset_manager.hpp"

namespace {
esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
}

void ResetManager::begin() { resetReason = esp_reset_reason(); }

esp_reset_reason_t ResetManager::reason() { return resetReason; }

bool ResetManager::requiresRearm() {
  switch (resetReason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
    case ESP_RST_SW:
      return true;
    default:
      return false;
  }
}
