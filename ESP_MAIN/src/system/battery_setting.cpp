#include "system/battery_setting.hpp"

#include <Preferences.h>

namespace {
constexpr const char* NAMESPACE = "battery";
constexpr const char* KEY = "check";
}

bool BatterySetting::checkEnabled() {
  Preferences preferences;
  if (!preferences.begin(NAMESPACE, true)) return true;  // never written yet: check on
  const bool enabled = preferences.getUChar(KEY, 1) != 0;
  preferences.end();
  return enabled;
}

bool BatterySetting::setCheckEnabled(bool enabled) {
  Preferences preferences;
  if (!preferences.begin(NAMESPACE, false)) return false;
  const bool ok = preferences.putUChar(KEY, enabled ? 1 : 0) == 1;
  preferences.end();
  return ok;
}
