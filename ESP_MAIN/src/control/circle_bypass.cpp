#include "control/circle_bypass.hpp"

namespace {
using namespace config::circle;

bool running = false;
bool right = true;       // turn direction away from the circle
uint32_t startedMs = 0;

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
  const uint32_t elapsed = nowMs - startedMs;
  if (elapsed < BACK_MS) {
    if (floor.rear()) {
      running = false;
      result = BypassResult::NeedEscape;
      return drive(DriveIntentType::Brake, 0);
    }
    return drive(DriveIntentType::Reverse, BACK_DUTY);   // straight back
  }
  if (elapsed < BACK_MS + TURN_MS) {
    return drive(right ? DriveIntentType::SearchRight : DriveIntentType::SearchLeft, TURN_DUTY);
  }
  if (elapsed < BACK_MS + TURN_MS + ARC_MS) {
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
