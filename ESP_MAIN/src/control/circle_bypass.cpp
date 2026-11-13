#include "control/circle_bypass.hpp"

namespace {
using namespace config::circle;

bool running = false;
bool right = true;       // turn direction away from the circle
uint32_t startedMs = 0;
bool backing = true;          // step 1 (reverse) still running
uint32_t frontClearSinceMs = 0;
bool frontClear = false;
uint32_t turnStartMs = 0;     // set when the reverse ends

DriveIntent drive(DriveIntentType type, uint16_t duty, int16_t steering = 0) {
  DriveIntent result;
  result.type = type;
  result.requestedDuty = duty;
  result.steering = steering;
  return result;
}
}  // namespace

void CircleBypass::start(uint32_t nowMs, bool turnRight) {
  running = true;
  right = turnRight;
  startedMs = nowMs;
  backing = true;
  frontClear = false;
}

void CircleBypass::cancel() { running = false; }

bool CircleBypass::active() { return running; }

DriveIntent CircleBypass::update(uint32_t nowMs, const FloorHazardInfo& floor,
                                 BypassResult& result) {
  result = BypassResult::Running;
  if (!running) {
    result = BypassResult::Done;
    return drive(DriveIntentType::Brake, 0);
  }
  if (floor.front() && floor.rear()) {   // inside the groove: normal escape handles it
    running = false;
    result = BypassResult::NeedEscape;
    return drive(DriveIntentType::Brake, 0);
  }
  if (backing) {
    if (floor.rear()) {
      running = false;
      result = BypassResult::NeedEscape;
      return drive(DriveIntentType::Brake, 0);
    }
    const uint32_t backFor = nowMs - startedMs;
    if (floor.front()) {
      frontClear = false;
    } else if (!frontClear) {
      frontClear = true;
      frontClearSinceMs = nowMs;
    }
    const bool cleared = frontClear && nowMs - frontClearSinceMs >= BACK_CLEAR_MS;
    if (backFor < BACK_MAX_MS && (backFor < BACK_MS || !cleared)) {
      return drive(DriveIntentType::Reverse, BACK_DUTY);   // straight back
    }
    backing = false;
    turnStartMs = nowMs;
  }
  const uint32_t elapsed = nowMs - turnStartMs;
  if (elapsed < TURN_MS) {
    return drive(right ? DriveIntentType::SearchRight : DriveIntentType::SearchLeft, TURN_DUTY);
  }
  if (elapsed < TURN_MS + ARC_MS) {
    if (floor.front()) {   // touched the circle again: next step around it
      start(nowMs, right);
      return drive(DriveIntentType::Brake, 0);
    }
    // Forward, steering back toward the circle (it is on the left when turning right).
    return drive(DriveIntentType::Approach, ARC_DUTY, right ? -ARC_STEERING : ARC_STEERING);
  }
  running = false;
  result = BypassResult::Done;
  return drive(DriveIntentType::Brake, 0);
}
