#include "control/wander.hpp"

namespace {
using namespace config::wander;

uint32_t rng = 0x9E3779B9U;
bool forward = false;
uint32_t segmentEndMs = 0;
DriveIntent segment;

uint32_t next() {  // xorshift32: enough to avoid a fixed pattern
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}

int32_t pick(int32_t low, int32_t high) {
  return low + static_cast<int32_t>(next() % static_cast<uint32_t>(high - low + 1));
}

void startSegment(uint32_t nowMs) {
  forward = !forward;
  segment = DriveIntent{};
  if (forward) {
    segment.type = DriveIntentType::Approach;
    segment.requestedDuty = static_cast<uint16_t>(pick(FORWARD_DUTY_MIN, FORWARD_DUTY_MAX));
    segment.steering = static_cast<int16_t>(pick(-FORWARD_STEERING_MAX, FORWARD_STEERING_MAX));
    segmentEndMs = nowMs + static_cast<uint32_t>(pick(FORWARD_MS_MIN, FORWARD_MS_MAX));
  } else {
    segment.type = (next() & 1U) ? DriveIntentType::SearchLeft : DriveIntentType::SearchRight;
    segment.requestedDuty = static_cast<uint16_t>(pick(TURN_DUTY_MIN, TURN_DUTY_MAX));
    segmentEndMs = nowMs + static_cast<uint32_t>(pick(TURN_MS_MIN, TURN_MS_MAX));
  }
}
}  // namespace

void Wander::reset(uint32_t nowMs) {
  rng ^= nowMs * 2654435761U;  // mix in the time so each entry starts differently
  if (rng == 0) rng = 0x9E3779B9U;
  forward = true;  // first segment is a turn, so it does not drive straight back the way it came
  startSegment(nowMs);
}

DriveIntent Wander::intent(uint32_t nowMs) {
  if (static_cast<int32_t>(nowMs - segmentEndMs) >= 0) startSegment(nowMs);
  return segment;
}
