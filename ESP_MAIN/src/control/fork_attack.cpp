#include "control/fork_attack.hpp"

#include "control/control_config.hpp"

namespace {
using namespace config::fork;

bool running = false;
bool everEnded = false;
uint32_t startedMs = 0;
uint32_t endedMs = 0;

ForkPhase phaseAt(uint32_t elapsed) {
  if (elapsed < BRAKE_MS) return ForkPhase::Brake;
  if (elapsed < BRAKE_MS + INSERT_MS) return ForkPhase::Insert;
  if (elapsed < BRAKE_MS + INSERT_MS + BURST_MS) return ForkPhase::Burst;
  return ForkPhase::Idle;
}

void finish(uint32_t nowMs) {
  running = false;
  everEnded = true;
  endedMs = nowMs;
}
}  // namespace

void ForkAttack::start(uint32_t nowMs) {
  running = true;
  startedMs = nowMs;
}

void ForkAttack::cancel() {
  if (running) {
    running = false;
    everEnded = true;
    endedMs = startedMs;   // cancelled early: cooldown counts from the start
  }
}

ForkPhase ForkAttack::phase(uint32_t nowMs) {
  if (!running) return ForkPhase::Idle;
  const ForkPhase p = phaseAt(nowMs - startedMs);
  if (p == ForkPhase::Idle) finish(nowMs);
  return p;
}

bool ForkAttack::ready(uint32_t nowMs) {
  if (running) return false;
  return !everEnded || nowMs - endedMs >= COOLDOWN_MS;
}

DriveIntent ForkAttack::intent(uint32_t nowMs, int32_t towerError) {
  DriveIntent out;   // Brake by default
  const ForkPhase p = phase(nowMs);
  if (p == ForkPhase::Idle || p == ForkPhase::Brake) return out;
  const int32_t steer = towerError * STEERING_MAX / static_cast<int32_t>(config::control::TARGET_CENTER_X);
  out.type = DriveIntentType::Approach;
  out.requestedDuty = p == ForkPhase::Insert ? INSERT_DUTY : BURST_DUTY;
  out.steering = static_cast<int16_t>(constrain(steer, -static_cast<int32_t>(STEERING_MAX),
                                                static_cast<int32_t>(STEERING_MAX)));
  return out;
}
