#include "control/robot_fsm.hpp"

#include "control/control_config.hpp"
#include "system/event_logger.hpp"

namespace {

RobotState currentState = RobotState::Boot;
uint32_t stateEnteredMs = 0;
uint32_t lastTargetMs = 0;
uint16_t lastTargetX = config::control::TARGET_CENTER_X;
uint8_t activeHazardSensors = 0;
uint8_t targetConfirmFrames = 0;
uint8_t lastTargetSequence = 0;
bool targetSequenceKnown = false;
bool lastTargetKnown = false;
uint32_t lastTargetFrameMs = 0;
// Groove exit memory: the last hazard seen on only one end, and whether the
// robot was backing off when the escape started.
uint8_t lastOneEndHazards = 0;
bool escapeStartedReversing = false;
bool insideGroove = false;
bool grooveExitForward = false;
uint32_t insideGrooveSinceMs = 0;

void transitionTo(RobotState next, uint32_t nowMs) {
  if (currentState == next) return;
  currentState = next;
  stateEnteredMs = nowMs;
  EventLogger::stateChanged(static_cast<uint16_t>(currentState));
}

DriveIntent intent(DriveIntentType type, uint16_t duty = 0,
                   int16_t steering = 0) {
  DriveIntent result;
  result.type = type;
  result.requestedDuty = duty;
  result.steering = steering;
  return result;
}

bool isHazardColor(const FloorColorResult& result) {
  if (!result.reliable || result.color == FloorColor::Unknown ||
      result.color == FloorColor::Black ||
      static_cast<uint8_t>(result.color) >= FLOOR_COLOR_COUNT) {
    return false;
  }
  const uint8_t bit = 1U << static_cast<uint8_t>(result.color);
  return (config::control::HAZARD_COLOR_MASK & bit) != 0;
}

uint8_t hazardSensorMask(const SensorSnapshot& sensors) {
  uint8_t mask = 0;
  for (uint8_t index = 0; index < COLOR_SENSOR_COUNT; ++index) {
    if (isHazardColor(sensors.floorColor[index])) mask |= 1U << index;
  }
  return mask;
}

DriveIntent searchIntent(uint32_t nowMs) {
  const bool turnLeft =
      ((nowMs / config::control::SEARCH_DIRECTION_PERIOD_MS) & 1U) == 0;
  return intent(turnLeft ? DriveIntentType::SearchLeft
                         : DriveIntentType::SearchRight,
                config::control::SEARCH_DUTY);
}

DriveIntent zoneEscapeIntent(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - stateEnteredMs;
  const bool frontHazard = (activeHazardSensors & 0x03U) != 0;
  const bool rearHazard = (activeHazardSensors & 0x0CU) != 0;

  if (elapsed < config::control::ZONE_ESCAPE_REVERSE_MS) {
    if (frontHazard || !rearHazard) {
      return intent(DriveIntentType::Reverse,
                    config::control::ZONE_ESCAPE_DUTY);
    }
    return intent(DriveIntentType::Approach,
                  config::control::ZONE_ESCAPE_DUTY);
  }

  const bool leftHazard = (activeHazardSensors & 0x05U) != 0;
  const bool rightHazard = (activeHazardSensors & 0x0AU) != 0;
  if (leftHazard && !rightHazard) {
    return intent(DriveIntentType::EscapeRight,
                  config::control::ZONE_ESCAPE_DUTY);
  }
  if (rightHazard && !leftHazard) {
    return intent(DriveIntentType::EscapeLeft,
                  config::control::ZONE_ESCAPE_DUTY);
  }
  return searchIntent(nowMs);
}

bool frontHazardIn(uint8_t mask) { return (mask & 0x03U) != 0; }
bool rearHazardIn(uint8_t mask) { return (mask & 0x0CU) != 0; }

// The end that touched the hazard last is the end that went in last, so the
// way out is the opposite direction.
bool chooseGrooveExitForward() {
  if (rearHazardIn(lastOneEndHazards)) return true;
  if (frontHazardIn(lastOneEndHazards)) return false;
  return escapeStartedReversing;
}

DriveIntent grooveExitIntent(uint32_t nowMs) {
  if (!insideGroove) {
    insideGroove = true;
    insideGrooveSinceMs = nowMs;
    grooveExitForward = chooseGrooveExitForward();
  }
  const uint32_t held = nowMs - insideGrooveSinceMs;
  if (held < config::control::BOTH_HAZARD_BRAKE_MS) {
    return intent(DriveIntentType::Brake);
  }
  const bool flipped =
      (((held - config::control::BOTH_HAZARD_BRAKE_MS) /
        config::control::GROOVE_EXIT_FLIP_MS) & 1U) != 0;
  return intent(grooveExitForward != flipped ? DriveIntentType::Approach
                                             : DriveIntentType::Reverse,
                config::control::ZONE_ESCAPE_DUTY);
}

int16_t targetSteering(uint16_t targetX, int16_t maximum) {
  const int32_t error =
      static_cast<int32_t>(targetX) - config::control::TARGET_CENTER_X;
  const int32_t scaled =
      error * maximum / static_cast<int32_t>(config::control::TARGET_CENTER_X);
  return static_cast<int16_t>(constrain(scaled, -maximum, maximum));
}

DriveIntent forwardAtTarget(DriveIntentType type, uint16_t duty,
                            uint16_t targetX, int16_t maximumSteering) {
  return intent(type, duty, targetSteering(targetX, maximumSteering));
}

void resetTargetConfirmation() { targetConfirmFrames = 0; }

void observeTargetFrame(const VisionSnapshot& vision) {
  if (!targetSequenceKnown || vision.sequence != lastTargetSequence ||
      vision.frameUpdatedMs != lastTargetFrameMs) {
    lastTargetSequence = vision.sequence;
    targetSequenceKnown = true;
    lastTargetFrameMs = vision.frameUpdatedMs;
    if (targetConfirmFrames < UINT8_MAX) ++targetConfirmFrames;
  }
}

}  // namespace

void RobotFsm::begin() {
  currentState = RobotState::Boot;
  stateEnteredMs = millis();
  lastTargetMs = 0;
  lastTargetX = config::control::TARGET_CENTER_X;
  activeHazardSensors = 0;
  targetConfirmFrames = 0;
  targetSequenceKnown = false;
  lastTargetKnown = false;
  lastOneEndHazards = 0;
  escapeStartedReversing = false;
  insideGroove = false;
  transitionTo(RobotState::WaitStart, stateEnteredMs);
}

DriveIntent RobotFsm::update(uint32_t nowMs, const SensorSnapshot& sensors,
                             const VisionSnapshot& vision,
                             const SafetyStatus& safety) {
  if (!safety.motorAllowed) {
    resetTargetConfirmation();
    lastTargetKnown = false;
    targetSequenceKnown = false;
    lastOneEndHazards = 0;
    insideGroove = false;
    transitionTo(safety.state == SafetyState::EmergencyStop
                     ? RobotState::EmergencyStop
                     : RobotState::WaitStart,
                 nowMs);
    return intent(DriveIntentType::Brake);
  }

  // Colored-floor escape always interrupts an attack, including blind push.
  const uint8_t hazards = hazardSensorMask(sensors);
  if (frontHazardIn(hazards) != rearHazardIn(hazards)) {
    lastOneEndHazards = hazards;
  }
  if (hazards != 0 && currentState != RobotState::ZoneEscape) {
    activeHazardSensors = hazards;
    escapeStartedReversing = currentState == RobotState::Backoff;
    insideGroove = false;
    resetTargetConfirmation();
    transitionTo(RobotState::ZoneEscape, nowMs);
  }

  if (currentState == RobotState::ZoneEscape) {
    const bool front = (hazards & 0x03U) != 0;
    const bool rear = (hazards & 0x0CU) != 0;
    const bool reversing = (activeHazardSensors & 0x03U) != 0;
    const uint32_t elapsed = nowMs - stateEnteredMs;
    if (front && rear) {
      // Whole robot over a hazard color: braking forever would leave it in
      // the groove. Brake briefly, then drive out (see grooveExitIntent).
      activeHazardSensors = hazards;
      stateEnteredMs = nowMs;
      return grooveExitIntent(nowMs);
    }
    if (insideGroove) {
      // One end is out. Restart the normal escape away from the end that is
      // still inside; if neither end reads a hazard, keep moving the same way.
      insideGroove = false;
      activeHazardSensors =
          hazards != 0 ? hazards : (grooveExitForward ? 0x0CU : 0x03U);
      stateEnteredMs = nowMs;
      return zoneEscapeIntent(nowMs);
    }
    // Never complete an old timed movement into a newly detected groove.
    if (elapsed < config::control::ZONE_ESCAPE_REVERSE_MS &&
        ((reversing && rear) || (!reversing && front))) {
      activeHazardSensors = hazards;
      stateEnteredMs = nowMs;
      return intent(DriveIntentType::Brake);
    }
    if (elapsed >= config::control::ZONE_ESCAPE_REVERSE_MS && hazards != 0) {
      activeHazardSensors = hazards;
    }
    const uint32_t escapeDuration = config::control::ZONE_ESCAPE_REVERSE_MS +
                                    config::control::ZONE_ESCAPE_TURN_MS;
    if (nowMs - stateEnteredMs < escapeDuration) {
      return zoneEscapeIntent(nowMs);
    }
    if (hazards != 0) {
      activeHazardSensors = hazards;
      stateEnteredMs = nowMs;
      return zoneEscapeIntent(nowMs);
    }
    activeHazardSensors = 0;
    lastOneEndHazards = 0;
    transitionTo(RobotState::Search, nowMs);
    return searchIntent(nowMs);
  }

  const bool targetVisible = vision.linkValid && vision.frameFresh &&
                             vision.targetValid && vision.cameraOk &&
                             vision.pipelineOk;
  if (targetVisible) {
    observeTargetFrame(vision);
    lastTargetMs = vision.frameUpdatedMs;
    lastTargetX = vision.targetX;
    lastTargetKnown = true;
  }

  // Finish the short mechanical attack sequence through brief camera loss.
  // Hazard and emergency checks remain above it.
  if (currentState == RobotState::DiveReady) {
    if (nowMs - stateEnteredMs < config::control::ATTACK_CHARGE_MS) {
      return forwardAtTarget(DriveIntentType::AttackCharge,
                             config::control::ATTACK_CHARGE_DUTY,
                             targetVisible ? vision.targetX : lastTargetX,
                             config::control::PUSH_STEERING_MAX);
    }
    transitionTo(RobotState::DiveBrake, nowMs);
    return intent(DriveIntentType::DiveBrake);
  }

  if (currentState == RobotState::DiveBrake) {
    if (nowMs - stateEnteredMs < config::control::DIVE_BRAKE_MS) {
      return intent(DriveIntentType::DiveBrake);
    }
    transitionTo(RobotState::SpokeEngaged, nowMs);
    return forwardAtTarget(DriveIntentType::SpokeInsert,
                           config::control::SPOKE_INSERT_DUTY,
                           targetVisible ? vision.targetX : lastTargetX,
                           config::control::PUSH_STEERING_MAX);
  }

  if (currentState == RobotState::SpokeEngaged) {
    if (nowMs - stateEnteredMs < config::control::SPOKE_INSERT_MS) {
      return forwardAtTarget(DriveIntentType::SpokeInsert,
                             config::control::SPOKE_INSERT_DUTY,
                             targetVisible ? vision.targetX : lastTargetX,
                             config::control::PUSH_STEERING_MAX);
    }
    transitionTo(RobotState::Push, nowMs);
  }

  if (currentState == RobotState::Push) {
    if (targetVisible) {
      if (vision.targetHeight < config::control::ATTACK_BOX_HEIGHT / 2 ||
          abs(static_cast<int32_t>(vision.targetX) -
              config::control::TARGET_CENTER_X) > 600) {
        resetTargetConfirmation();
        transitionTo(RobotState::TargetConfirm, nowMs);
        return intent(DriveIntentType::Brake);
      }
      return forwardAtTarget(DriveIntentType::Push,
                             config::control::PUSH_DUTY, vision.targetX,
                             config::control::PUSH_STEERING_MAX);
    }
    if (lastTargetKnown &&
        nowMs - lastTargetMs < config::control::BLIND_PUSH_HOLD_MS) {
      return intent(DriveIntentType::Push, config::control::PUSH_DUTY);
    }
    transitionTo(RobotState::Backoff, nowMs);
    return intent(DriveIntentType::Reverse, config::control::BACKOFF_DUTY);
  }

  if (currentState == RobotState::Backoff) {
    if (nowMs - stateEnteredMs < config::control::BACKOFF_MS) {
      return intent(DriveIntentType::Reverse, config::control::BACKOFF_DUTY);
    }
    resetTargetConfirmation();
    transitionTo(RobotState::Search, nowMs);
    return searchIntent(nowMs);
  }

  if (!targetVisible) {
    resetTargetConfirmation();
    if (lastTargetKnown &&
        nowMs - lastTargetMs < config::control::LOST_TARGET_HOLD_MS) {
      transitionTo(RobotState::LostTarget, nowMs);
      return intent(lastTargetX < config::control::TARGET_CENTER_X
                        ? DriveIntentType::SearchLeft
                        : DriveIntentType::SearchRight,
                    config::control::SEARCH_DUTY);
    }
    transitionTo(RobotState::Search, nowMs);
    return searchIntent(nowMs);
  }

  const int32_t error = static_cast<int32_t>(vision.targetX) -
                        config::control::TARGET_CENTER_X;
  if (targetConfirmFrames < config::control::TARGET_CONFIRM_FRAMES) {
    transitionTo(RobotState::TargetConfirm, nowMs);
    if (error < -static_cast<int32_t>(
                    config::control::TARGET_CENTER_DEADBAND)) {
      return intent(DriveIntentType::AlignLeft,
                    config::control::TARGET_CONFIRM_DUTY);
    }
    if (error > static_cast<int32_t>(
                    config::control::TARGET_CENTER_DEADBAND)) {
      return intent(DriveIntentType::AlignRight,
                    config::control::TARGET_CONFIRM_DUTY);
    }
    return forwardAtTarget(DriveIntentType::Approach,
                           config::control::TARGET_CONFIRM_DUTY,
                           vision.targetX,
                           config::control::APPROACH_STEERING_MAX);
  }

  if (error < -static_cast<int32_t>(
                  config::control::TARGET_CENTER_DEADBAND)) {
    transitionTo(RobotState::TrackAlign, nowMs);
    return intent(DriveIntentType::AlignLeft, config::control::ALIGN_DUTY);
  }
  if (error > static_cast<int32_t>(
                  config::control::TARGET_CENTER_DEADBAND)) {
    transitionTo(RobotState::TrackAlign, nowMs);
    return intent(DriveIntentType::AlignRight, config::control::ALIGN_DUTY);
  }

  const bool attackCentered =
      abs(error) <= config::control::ATTACK_CENTER_DEADBAND;
  if (attackCentered &&
      vision.targetHeight >= config::control::ATTACK_BOX_HEIGHT) {
    transitionTo(RobotState::DiveReady, nowMs);
    return forwardAtTarget(DriveIntentType::AttackCharge,
                           config::control::ATTACK_CHARGE_DUTY,
                           vision.targetX,
                           config::control::PUSH_STEERING_MAX);
  }

  transitionTo(RobotState::Approach, nowMs);
  return forwardAtTarget(DriveIntentType::Approach,
                         config::control::APPROACH_DUTY, vision.targetX,
                         config::control::APPROACH_STEERING_MAX);
}

RobotState RobotFsm::state() { return currentState; }
