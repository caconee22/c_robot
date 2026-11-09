#include "motor/motor_trim.hpp"

#include <Preferences.h>

namespace {
constexpr const char* NAMESPACE = "motor";
constexpr const char* KEY_LEFT = "trimL";
constexpr const char* KEY_RIGHT = "trimR";

// Read by MotorController::apply, written by the bench console; both run on
// the main loop task.
volatile uint16_t trimLeft = MotorTrim::MAX_PERMILLE;
volatile uint16_t trimRight = MotorTrim::MAX_PERMILLE;

bool valid(uint16_t value) {
  return value >= MotorTrim::MIN_PERMILLE && value <= MotorTrim::MAX_PERMILLE;
}
}  // namespace

void MotorTrim::load() {
  trimLeft = trimRight = MAX_PERMILLE;
  Preferences preferences;
  if (!preferences.begin(NAMESPACE, true)) return;   // never saved: no trim
  const uint16_t left = preferences.getUShort(KEY_LEFT, MAX_PERMILLE);
  const uint16_t right = preferences.getUShort(KEY_RIGHT, MAX_PERMILLE);
  preferences.end();
  if (valid(left) && valid(right)) {
    trimLeft = left;
    trimRight = right;
  }
}

uint16_t MotorTrim::left() { return trimLeft; }
uint16_t MotorTrim::right() { return trimRight; }

bool MotorTrim::set(uint16_t leftPermille, uint16_t rightPermille) {
  if (!valid(leftPermille) || !valid(rightPermille)) return false;
  trimLeft = leftPermille;
  trimRight = rightPermille;
  return true;
}

bool MotorTrim::save() {
  Preferences preferences;
  if (!preferences.begin(NAMESPACE, false)) return false;
  const bool ok = preferences.putUShort(KEY_LEFT, trimLeft) == 2 &&
                  preferences.putUShort(KEY_RIGHT, trimRight) == 2;
  preferences.end();
  return ok;
}

int16_t MotorTrim::scale(int16_t value, uint16_t trimPermille) {
  return static_cast<int16_t>(static_cast<int32_t>(value) * trimPermille / 1000);
}
