#include "control/escape_move.hpp"

namespace {
using namespace config::escape;

bool running = false;
bool straight = false;   // 11/11 straight back-out (no turn)
bool chargingHint = false;
bool flipped = false;          // FLIP_MS passed without getting off: other way
uint32_t directionSinceMs = 0;
uint32_t lastHazardMs = 0;

DriveIntent drive(DriveIntentType type, uint16_t duty, int16_t steering = 0) {
  DriveIntent result;
  result.type = type;
  result.requestedDuty = duty;
  result.steering = steering;
  return result;
}

struct Exit {
  bool reverse;
  int16_t steering;
};

// Way out from the side that went in first (see the header table).
Exit exitFor(HazardSide first) {
  switch (first) {
    case HazardSide::FrontBoth: return {true, 0};
    case HazardSide::FrontLeft: return {true, static_cast<int16_t>(-TURN_STEERING)};  // field sign
    case HazardSide::FrontRight: return {true, TURN_STEERING};
    case HazardSide::RearBoth: return {false, 0};
    case HazardSide::RearLeft: return {false, REAR_STEERING};                       // nose right
    case HazardSide::RearRight: return {false, static_cast<int16_t>(-REAR_STEERING)};
    case HazardSide::Left: return {false, SIDE_STEERING};                           // hard right
    case HazardSide::Right: return {false, static_cast<int16_t>(-SIDE_STEERING)};
    case HazardSide::Unclear:
    case HazardSide::None:
    default: return {chargingHint, 0};
  }
}
}  // namespace

void EscapeMove::start(uint32_t nowMs, const FloorHazardInfo&, bool wasCharging, bool straightBack) {
  running = true;
  chargingHint = wasCharging;
  straight = straightBack;
  flipped = false;
  lastHazardMs = nowMs;
  directionSinceMs = nowMs;
}

void EscapeMove::cancel() { running = false; }

bool EscapeMove::active() { return running; }

DriveIntent EscapeMove::update(uint32_t nowMs, const FloorHazardInfo& floor) {
  if (!running) return drive(DriveIntentType::Brake, 0);
  if (floor.mask == 0) {
    if (nowMs - lastHazardMs >= CLEAR_MS) {
      running = false;
      return drive(DriveIntentType::Brake, 0);
    }
  } else {
    lastHazardMs = nowMs;
    if (nowMs - directionSinceMs >= FLIP_MS) {   // not getting off the color this way
      flipped = !flipped;
      directionSinceMs = nowMs;
    }
  }
  Exit out = exitFor(floor.first);
  if (flipped) out = {!out.reverse, 0};
  if (straight && out.reverse) out.steering = 0;   // 11/11: back out facing the opponent
  const bool pushedIn = floor.front() && floor.rear();
  uint16_t duty = pushedIn ? PUSHED_DUTY : (out.reverse ? REVERSE_DUTY : FORWARD_DUTY);
  // Keep the faster wheel under the motor limit so the turn is not clipped
  // (620 + 250 would ask 870 and lose a third of the turn when pushed in).
  const int16_t steer = out.steering < 0 ? -out.steering : out.steering;
  if (duty + steer > MOTOR_LIMIT) duty = static_cast<uint16_t>(MOTOR_LIMIT - steer);
  return drive(out.reverse ? DriveIntentType::Reverse : DriveIntentType::Approach, duty, out.steering);
}
