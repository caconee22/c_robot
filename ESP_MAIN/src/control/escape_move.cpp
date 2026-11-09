#include "control/escape_move.hpp"

namespace {
using namespace config::escape;

bool running = false;
bool backing = true;          // true: reverse, false: forward
int16_t steering = 0;         // nose swing while reversing
uint32_t directionSinceMs = 0;
uint32_t lastHazardMs = 0;
bool inGroove = false;
uint32_t grooveSinceMs = 0;
bool grooveForward = false;

DriveIntent drive(DriveIntentType type, uint16_t duty, int16_t steer = 0) {
  DriveIntent result;
  result.type = type;
  result.requestedDuty = duty;
  result.steering = steer;
  return result;
}

int16_t steeringFor(HazardSide side) {
  if (side == HazardSide::FrontLeft) return TURN_STEERING;     // nose swings right
  if (side == HazardSide::FrontRight) return -TURN_STEERING;   // nose swings left
  return 0;
}

void setDirection(bool reverse, uint32_t nowMs) {
  backing = reverse;
  directionSinceMs = nowMs;
}
}  // namespace

void EscapeMove::start(uint32_t nowMs, const FloorHazardInfo& floor) {
  running = true;
  inGroove = false;
  lastHazardMs = nowMs;
  steering = steeringFor(floor.first);
  setDirection(floor.first != HazardSide::Rear, nowMs);
}

void EscapeMove::cancel() { running = false; }

bool EscapeMove::active() { return running; }

DriveIntent EscapeMove::update(uint32_t nowMs, const FloorHazardInfo& floor) {
  if (!running) return drive(DriveIntentType::Brake, 0);
  if (floor.mask != 0) lastHazardMs = nowMs;

  if (floor.front() && floor.rear()) {
    // Inside a groove: brake briefly, then out opposite to the way we were moving.
    if (!inGroove) {
      inGroove = true;
      grooveSinceMs = nowMs;
      grooveForward = backing;
    }
    const uint32_t held = nowMs - grooveSinceMs;
    if (held < BOTH_BRAKE_MS) return drive(DriveIntentType::Brake, 0);
    const bool flipped = (((held - BOTH_BRAKE_MS) / FLIP_MS) & 1U) != 0;
    return grooveForward != flipped ? drive(DriveIntentType::Approach, FORWARD_DUTY)
                                    : drive(DriveIntentType::Reverse, REVERSE_DUTY);
  }
  if (inGroove) {
    // One end is out: keep going away from the end still on the color.
    inGroove = false;
    steering = 0;
    setDirection(floor.front() || (floor.mask == 0 && !grooveForward), nowMs);
  }

  if (floor.mask == 0) {
    if (nowMs - lastHazardMs >= CLEAR_MS) {
      running = false;
      return drive(DriveIntentType::Brake, 0);
    }
  } else if (backing && floor.rear() && !floor.front()) {
    setDirection(false, nowMs);   // backed onto a hazard: go forward instead
    return drive(DriveIntentType::Brake, 0);
  } else if (!backing && floor.front() && !floor.rear()) {
    setDirection(true, nowMs);
    return drive(DriveIntentType::Brake, 0);
  } else if (nowMs - directionSinceMs >= FLIP_MS) {
    setDirection(!backing, nowMs);  // not getting off the color this way
  }
  return backing ? drive(DriveIntentType::Reverse, REVERSE_DUTY, steering)
                 : drive(DriveIntentType::Approach, FORWARD_DUTY);
}
