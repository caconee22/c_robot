#include "bench/bench_controller.hpp"
#include "bench/bench_config.hpp"
#include "sensors/sensor_config.hpp"

namespace {
using namespace config::bench;
BenchMode selected = BenchMode::Stop;
bool wasAllowed = false, done = false, manualValid = false, escaping = false;
uint32_t runStarted = 0, manualReceived = 0, escapeStarted = 0;
int16_t manualLeft = 0, manualRight = 0;
uint8_t escapeMask = 0;

void resetMotion() {
  wasAllowed = done = manualValid = escaping = false;
  manualLeft = manualRight = 0;
}

MotorCommand command(uint32_t now, int16_t left = 0, int16_t right = 0) {
  MotorCommand result;
  result.leftPermille = left;
  result.rightPermille = right;
  result.deadlineSet = true;
  result.validUntilMs = now + COMMAND_VALID_MS;
  return result;
}

bool floorsReady(uint32_t now, const SensorSnapshot& sensors) {
  if (sensors.colorSensorReadyMask != 0x0F) return false;
  for (const auto& floor : sensors.floorColor) {
    if (!floor.reliable || floor.color == FloorColor::Unknown ||
        now - floor.timestampMs > config::sensors::SAMPLE_STALE_MS) return false;
  }
  return true;
}

MotorCommand track(uint32_t now, const VisionSnapshot& vision) {
  if (!vision.linkValid || !vision.frameFresh || !vision.targetValid ||
      !vision.cameraOk || !vision.pipelineOk || vision.targetX >= 1920 ||
      vision.targetHeight == 0 || vision.targetHeight > 1080) return command(now);
  const int error = static_cast<int>(vision.targetX) - CENTER_X;
  const int magnitude = error < 0 ? -error : error;
  if (selected == BenchMode::Align || magnitude > TURN_ONLY_ERROR) {
    if (magnitude <= CENTER_DEADBAND) return command(now);
    return error > 0 ? command(now, TURN_DUTY, -TURN_DUTY)
                     : command(now, -TURN_DUTY, TURN_DUTY);
  }
  // Box height is only an experimental proximity proxy, not a distance.
  if (vision.boxClipped || vision.targetHeight >= STOP_BOX_HEIGHT) return command(now);
  const int16_t steering = magnitude <= CENTER_DEADBAND ? 0 :
      static_cast<int16_t>(error * STEERING_MAX / CENTER_X);
  return command(now, FOLLOW_DUTY + steering, FOLLOW_DUTY - steering);
}
}

void BenchController::begin() { selected = BenchMode::Stop; resetMotion(); }
void BenchController::select(BenchMode mode) {
  selected = static_cast<uint8_t>(mode) <= static_cast<uint8_t>(BenchMode::Avoid)
                 ? mode : BenchMode::Stop;
  resetMotion();
}
BenchMode BenchController::mode() { return selected; }
bool BenchController::finished() { return done; }
const char* BenchController::modeName(BenchMode mode) {
  switch (mode) {
    case BenchMode::Motor: return "MOTOR";
    case BenchMode::Manual: return "MANUAL";
    case BenchMode::Data: return "DATA";
    case BenchMode::Align: return "ALIGN";
    case BenchMode::Follow: return "FOLLOW";
    case BenchMode::Avoid: return "AVOID";
    default: return "STOP";
  }
}

bool BenchController::setManual(int left, int right, uint32_t now, bool allowed) {
  if (selected != BenchMode::Manual || !allowed ||
      left < -OUTPUT_LIMIT || left > OUTPUT_LIMIT ||
      right < -OUTPUT_LIMIT || right > OUTPUT_LIMIT) {
    manualValid = false;
    return false;
  }
  manualLeft = left; manualRight = right;
  manualReceived = now; manualValid = true;
  return true;
}

uint8_t BenchController::hazardMask(const SensorSnapshot& sensors) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < COLOR_SENSOR_COUNT; ++i) {
    const auto& floor = sensors.floorColor[i];
    if (floor.reliable && (floor.color == FloorColor::Red ||
        floor.color == FloorColor::Yellow || floor.color == FloorColor::Blue)) mask |= 1U << i;
  }
  return mask;
}

MotorCommand BenchController::update(uint32_t now, bool allowed,
    const SensorSnapshot& sensors, const VisionSnapshot& vision) {
  if (!allowed) { resetMotion(); return command(now); }
  if (!wasAllowed) { runStarted = now; wasAllowed = true; }
  if (selected == BenchMode::Stop || selected == BenchMode::Data) return command(now);
  if (selected == BenchMode::Manual) {
    if (!manualValid || now - manualReceived >= MANUAL_HOLD_MS) {
      manualValid = false; return command(now);
    }
    return command(now, manualLeft, manualRight);
  }
  if (selected == BenchMode::Motor) {
    const uint32_t elapsed = now - runStarted;
    const uint32_t step = elapsed / (MOTOR_STEP_MS + MOTOR_GAP_MS);
    if (step >= 4) { done = true; return command(now); }
    if (elapsed % (MOTOR_STEP_MS + MOTOR_GAP_MS) >= MOTOR_STEP_MS) return command(now);
    const int16_t duty = (step & 1U) ? -MOTOR_TEST_DUTY : MOTOR_TEST_DUTY;
    return step < 2 ? command(now, duty, 0) : command(now, 0, duty);
  }
  if (selected == BenchMode::Avoid) {
    if (!floorsReady(now, sensors)) { escaping = false; return command(now); }
    const uint8_t hazards = hazardMask(sensors);
    const bool front = (hazards & 0x03) != 0, rear = (hazards & 0x0C) != 0;
    if (front && rear) { escaping = false; return command(now); }
    if (!escaping && hazards) {
      escaping = true; escapeStarted = now; escapeMask = hazards;
    }
    if (escaping) {
      const uint32_t elapsed = now - escapeStarted;
      const bool reverse = (escapeMask & 0x03) != 0;
      if (elapsed < ESCAPE_MOVE_MS) {
        if ((reverse && rear) || (!reverse && front)) { escaping = false; return command(now); }
        const int16_t duty = reverse ? -TURN_DUTY : TURN_DUTY;
        return command(now, duty, duty);
      }
      if (elapsed < ESCAPE_MOVE_MS + ESCAPE_TURN_MS) {
        if (hazards) escapeMask = hazards;
        const bool turnRight = (escapeMask & 0x05) != 0;
        return turnRight ? command(now, TURN_DUTY, -TURN_DUTY)
                         : command(now, -TURN_DUTY, TURN_DUTY);
      }
      escaping = false;
      // Never follow a target while a hazard still remains underneath.
      if (hazards) return command(now);
    }
  }
  return track(now, vision);
}
