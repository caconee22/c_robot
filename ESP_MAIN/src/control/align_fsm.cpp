#include "control/align_fsm.hpp"

#include "control/control_config.hpp"
#include "control/escape_move.hpp"
#include "control/floor_hazard.hpp"
#include "control/opening_move.hpp"
#include "system/event_logger.hpp"

namespace {

RobotState currentState = RobotState::Boot;
bool targetKnown = false;
uint32_t lastTargetMs = 0;
uint16_t lastTargetX = config::control::TARGET_CENTER_X;
bool runStarted = false;   // false until the first motor-allowed pass of a run
bool charging = false;
uint8_t centeredFrames = 0;
bool frameKnown = false;
uint8_t lastSequence = 0;
uint32_t lastFrameMs = 0;
FloorColor escapeColorNow = FloorColor::Unknown;

void transitionTo(RobotState next) {
  if (currentState == next) return;
  currentState = next;
  EventLogger::stateChanged(static_cast<uint16_t>(currentState));
}

DriveIntent intent(DriveIntentType type, uint16_t duty = 0, int16_t steering = 0) {
  DriveIntent result;
  result.type = type;
  result.requestedDuty = duty;
  result.steering = steering;
  return result;
}

// Same ramp as the bench ALIGN and RobotFsm: slow just outside the center
// window, fastest at ALIGN_FULL_ERROR px and beyond.
uint16_t alignDuty(int32_t error) {
  using namespace config::control;
  const int32_t magnitude = error < 0 ? -error : error;
  if (magnitude >= ALIGN_FULL_ERROR) return ALIGN_MAX_DUTY;
  const int32_t span = ALIGN_FULL_ERROR - TARGET_CENTER_DEADBAND;
  const int32_t over = constrain(magnitude - static_cast<int32_t>(TARGET_CENTER_DEADBAND), 0, span);
  return static_cast<uint16_t>(ALIGN_MIN_DUTY + over * (ALIGN_MAX_DUTY - ALIGN_MIN_DUTY) / span);
}

int16_t chargeSteering(int32_t error) {
  using namespace config::control;
  const int32_t scaled = error * CHARGE_STEERING_MAX / static_cast<int32_t>(TARGET_CENTER_X);
  return static_cast<int16_t>(constrain(scaled, -static_cast<int32_t>(CHARGE_STEERING_MAX),
                                        static_cast<int32_t>(CHARGE_STEERING_MAX)));
}

bool newFrame(const VisionSnapshot& vision) {
  if (frameKnown && vision.sequence == lastSequence && vision.frameUpdatedMs == lastFrameMs) {
    return false;
  }
  frameKnown = true;
  lastSequence = vision.sequence;
  lastFrameMs = vision.frameUpdatedMs;
  return true;
}

void stopCharge() {
  charging = false;
  centeredFrames = 0;
}

}  // namespace

void AlignFsm::begin() {
  currentState = RobotState::Boot;
  targetKnown = false;
  lastTargetMs = 0;
  lastTargetX = config::control::TARGET_CENTER_X;
  runStarted = false;
  frameKnown = false;
  stopCharge();
  OpeningMove::cancel();
  EscapeMove::cancel();
  FloorHazard::reset();
  transitionTo(RobotState::WaitStart);
}

DriveIntent AlignFsm::update(uint32_t nowMs, const SensorSnapshot& sensors,
                             const VisionSnapshot& vision, const SafetyStatus& safety) {
  using namespace config::control;
  const FloorHazardInfo floor = FloorHazard::update(nowMs, sensors);
  if (!safety.motorAllowed) {
    targetKnown = false;
    runStarted = false;
    stopCharge();
    OpeningMove::cancel();
    EscapeMove::cancel();
    transitionTo(safety.state == SafetyState::EmergencyStop ? RobotState::EmergencyStop
                                                            : RobotState::WaitStart);
    return intent(DriveIntentType::Brake);
  }

  // Every new run (SW2) starts with the fixed opening move.
  if (!runStarted) {
    runStarted = true;
    if (config::opening::ENABLED) {
      OpeningMove::start(nowMs);
      EventLogger::logText(static_cast<uint16_t>(EventId::Custom), LogSeverity::Info,
                           LogSource::System, "opening_scale", OpeningMove::scalePermille(), 0, 0);
    }
  }

  // Floor escape wins over everything but safety, the opening move included.
  if (!EscapeMove::active() && floor.mask != 0) {
    OpeningMove::cancel();
    stopCharge();
    escapeColorNow = floor.firstColor;
    EscapeMove::start(nowMs, floor);
  }
  if (EscapeMove::active()) {
    transitionTo(RobotState::ZoneEscape);
    const DriveIntent out = EscapeMove::update(nowMs, floor);
    if (EscapeMove::active()) return out;
    // Escape done this pass: fall through to tracking, lock-on kept.
  }

  if (OpeningMove::active(nowMs)) {
    transitionTo(RobotState::Opening);
    return OpeningMove::intent(nowMs);
  }
  if (config::opening::ENABLED && !config::opening::TRACK_AFTER) {
    transitionTo(RobotState::Stop);   // tracker off: hold still after the move
    return intent(DriveIntentType::Brake);
  }

  const bool targetVisible = vision.linkValid && vision.frameFresh && vision.cameraOk &&
                             vision.pipelineOk && vision.targetValid;
  if (targetVisible) {
    const bool fresh = newFrame(vision);
    targetKnown = true;
    lastTargetMs = nowMs;
    lastTargetX = vision.targetX;
    const int32_t error = static_cast<int32_t>(vision.targetX) - TARGET_CENTER_X;
    const int32_t magnitude = error < 0 ? -error : error;

    if (charging) {
      if (magnitude > CHARGE_ABORT_ERROR) {
        stopCharge();   // drifted too far: turn in place again
      } else {
        transitionTo(RobotState::Approach);
        return intent(DriveIntentType::Approach, CHARGE_DUTY, chargeSteering(error));
      }
    }
    transitionTo(RobotState::TrackAlign);
    if (error < -static_cast<int32_t>(TARGET_CENTER_DEADBAND)) {
      centeredFrames = 0;
      return intent(DriveIntentType::AlignLeft, alignDuty(error));
    }
    if (error > static_cast<int32_t>(TARGET_CENTER_DEADBAND)) {
      centeredFrames = 0;
      return intent(DriveIntentType::AlignRight, alignDuty(error));
    }
    if (magnitude <= ATTACK_CENTER_DEADBAND) {
      if (fresh && centeredFrames < UINT8_MAX) ++centeredFrames;
    } else {
      centeredFrames = 0;
    }
    if (CHARGE_ENABLED && centeredFrames >= CHARGE_CONFIRM_FRAMES) {
      charging = true;
      transitionTo(RobotState::Approach);
      return intent(DriveIntentType::Approach, CHARGE_DUTY, chargeSteering(error));
    }
    return intent(DriveIntentType::Brake);   // centered: hold still and confirm
  }

  if (charging && nowMs - lastTargetMs < CHARGE_LOST_HOLD_MS) {
    return intent(DriveIntentType::Approach, CHARGE_DUTY);   // short drop: keep going straight
  }
  stopCharge();
  if (targetKnown && nowMs - lastTargetMs < ALIGN_LOST_HOLD_MS) {
    transitionTo(RobotState::TrackAlign);    // still locked on
    return intent(DriveIntentType::Brake);   // short detector drop: wait, do not spin off
  }
  transitionTo(RobotState::Search);
  const bool left = !targetKnown || lastTargetX < TARGET_CENTER_X;
  return intent(left ? DriveIntentType::SearchLeft : DriveIntentType::SearchRight, SEARCH_DUTY);
}

RobotState AlignFsm::state() { return currentState; }

FloorColor AlignFsm::escapeColor() { return escapeColorNow; }
