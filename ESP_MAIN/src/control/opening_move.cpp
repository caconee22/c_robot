#include "control/opening_move.hpp"

namespace {
using namespace config::opening;

struct Step {
  DriveIntentType type;
  uint16_t duty;
  uint32_t ms;
};

// Turn-left steps use SearchLeft: in-place spin, left wheel back, right forward.
constexpr Step STEPS[] = {
    {DriveIntentType::SearchLeft, TURN_DUTY, TURN_LEFT_45_MS},
    {DriveIntentType::Brake, 0, PAUSE_MS},
    {DriveIntentType::Approach, FORWARD_DUTY, FORWARD_MS},
    {DriveIntentType::Brake, 0, PAUSE_MS},
    {DriveIntentType::SearchLeft, TURN_DUTY, TURN_LEFT_90_MS},
    {DriveIntentType::Brake, 0, PAUSE_MS},
};
constexpr uint8_t STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

bool running = false;
uint32_t startedMs = 0;
uint8_t batteryCells = 0;
uint16_t scale = 1000;   // permille, fixed for one run

uint16_t scaleFor(uint8_t cells) {
  return cells == 3 ? THREE_CELL_SCALE_PERMILLE : FOUR_CELL_SCALE_PERMILLE;
}

// Index of the step at this time, STEP_COUNT when finished.
uint8_t stepAt(uint32_t nowMs) {
  uint32_t elapsed = nowMs - startedMs;
  for (uint8_t i = 0; i < STEP_COUNT; ++i) {
    if (elapsed < STEPS[i].ms) return i;
    elapsed -= STEPS[i].ms;
  }
  return STEP_COUNT;
}
}  // namespace

void OpeningMove::setBatteryCells(uint8_t cells) { batteryCells = cells; }

uint16_t OpeningMove::scalePermille() { return scale; }

void OpeningMove::start(uint32_t nowMs) {
  running = true;
  startedMs = nowMs;
  scale = scaleFor(batteryCells);
}

void OpeningMove::cancel() { running = false; }

bool OpeningMove::active(uint32_t nowMs) {
  if (running && stepAt(nowMs) >= STEP_COUNT) running = false;
  return running;
}

DriveIntent OpeningMove::intent(uint32_t nowMs) {
  DriveIntent result;   // Brake by default
  const uint8_t step = stepAt(nowMs);
  if (!running || step >= STEP_COUNT) return result;
  result.type = STEPS[step].type;
  result.requestedDuty = static_cast<uint16_t>(
      min(static_cast<uint32_t>(STEPS[step].duty) * scale / 1000, static_cast<uint32_t>(DUTY_MAX)));
  result.steering = 0;
  return result;
}
