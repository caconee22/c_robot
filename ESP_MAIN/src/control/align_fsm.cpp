#include "control/align_fsm.hpp"

#include <math.h>

#include "control/circle_bypass.hpp"
#include "control/control_config.hpp"
#include "control/escape_move.hpp"
#include "control/floor_hazard.hpp"
#include "control/fork_attack.hpp"
#include "control/opening_move.hpp"
#include "communication/run_log.hpp"
#include "motor/motor_trim.hpp"
#include "system/event_logger.hpp"

namespace {

RobotState currentState = RobotState::Boot;
bool targetKnown = false;
uint32_t lastTargetMs = 0;
uint16_t lastTargetX = config::control::TARGET_CENTER_X;
uint16_t lastTargetHeight = 0;
bool bypassTurnedRight = true;   // last circle bypass turn, for the search side after it
uint32_t bypassStartMs = 0;
bool runStarted = false;   // false until the first motor-allowed pass of a run
bool charging = false;
bool pushThroughActive = false;   // 11/11 push-through window running
// 11/11 straight back-out repeat limit (same front side within the window).
uint8_t straightCount = 0;
uint32_t straightWindowMs = 0;
HazardSide straightSide = HazardSide::None;
uint32_t pushThroughStartMs = 0;
uint8_t centeredFrames = 0;
bool frameKnown = false;
uint8_t lastSequence = 0;
uint32_t lastFrameMs = 0;
FloorColor escapeColorNow = FloorColor::Unknown;
// Align damping: tower x speed in the image (px/ms) from consecutive frames,
// and whether the robot already settled inside the window.
bool settled = false;
bool velocityKnown = false;
float velocityX = 0.0f;
int32_t previousX = 0;
uint32_t previousFrameMs = 0;

void transitionTo(RobotState next) {
  if (currentState == next) return;
  RunLog::push(RunLogKind::State, static_cast<uint16_t>(next), static_cast<uint16_t>(currentState),
               lastTargetX, lastTargetHeight);
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

void forgetMotion() {
  settled = false;
  velocityKnown = false;
  velocityX = 0.0f;
}

void trackVelocity(const VisionSnapshot& vision) {
  const int32_t x = vision.targetX;
  if (velocityKnown) {
    const uint32_t dt = vision.frameUpdatedMs - previousFrameMs;
    if (dt > 0 && dt < 200) velocityX = static_cast<float>(x - previousX) / dt;
  }
  previousX = x;
  previousFrameMs = vision.frameUpdatedMs;
  velocityKnown = true;
}

}  // namespace

void AlignFsm::begin() {
  currentState = RobotState::Boot;
  targetKnown = false;
  pushThroughActive = false;
  straightCount = 0;
  straightSide = HazardSide::None;
  lastTargetMs = 0;
  lastTargetX = config::control::TARGET_CENTER_X;
  runStarted = false;
  frameKnown = false;
  stopCharge();
  forgetMotion();
  OpeningMove::cancel();
  EscapeMove::cancel();
  CircleBypass::cancel();
  FloorHazard::reset();
  transitionTo(RobotState::WaitStart);
}

DriveIntent AlignFsm::update(uint32_t nowMs, const SensorSnapshot& sensors,
                             const VisionSnapshot& vision, const SafetyStatus& safety) {
  using namespace config::control;
  const FloorHazardInfo floor = FloorHazard::update(nowMs, sensors);
  if (!safety.motorAllowed) {
    if (runStarted) {
      RunLog::push(RunLogKind::Stop, static_cast<uint16_t>(safety.state));
      FloorHazard::reset();   // the next run starts without the old first-hit side
    }
    targetKnown = false;
    runStarted = false;
    stopCharge();
    forgetMotion();
    ForkAttack::cancel();
    OpeningMove::cancel();
    EscapeMove::cancel();
    CircleBypass::cancel();
    transitionTo(safety.state == SafetyState::EmergencyStop ? RobotState::EmergencyStop
                                                            : RobotState::WaitStart);
    return intent(DriveIntentType::Brake);
  }

  // Tower memory follows every frame, also while escaping or going around the
  // circle, so the lock-on survives those moves (11/10 field: it was lost).
  const bool targetVisible = vision.linkValid && vision.frameFresh && vision.cameraOk &&
                             vision.pipelineOk && vision.targetValid;
  if (targetVisible) {
    targetKnown = true;
    lastTargetMs = nowMs;
    lastTargetX = vision.targetX;
    lastTargetHeight = vision.targetHeight;
  }

  // Every new run (SW2) starts with the fixed opening move.
  if (!runStarted) {
    runStarted = true;
    if (config::opening::ENABLED) OpeningMove::start(nowMs);
    RunLog::push(RunLogKind::Start, config::opening::ENABLED ? OpeningMove::scalePermille() : 0,
                 MotorTrim::left(), MotorTrim::right(), OpeningMove::batteryCells());
    if (config::opening::ENABLED) {
      EventLogger::logText(static_cast<uint16_t>(EventId::Custom), LogSeverity::Info,
                           LogSource::System, "opening_scale", OpeningMove::scalePermille(), 0, 0);
    }
  }

  // Push-through (11/11): close opponent, front-only hazard while charging or
  // forking -> keep pushing PUSH_THROUGH_MS before the escape below starts.
  bool holdEscape = false;
  if (floor.mask == 0 || EscapeMove::active() || CircleBypass::active()) {
    pushThroughActive = false;
  } else if (config::control::PUSH_THROUGH_MS > 0) {
    const bool frontOnly = floor.front() && !floor.rear();
    const bool attacking = charging || ForkAttack::phase(nowMs) != ForkPhase::Idle;
    const bool close = targetKnown && nowMs - lastTargetMs < config::control::LOCKON_RECENT_MS &&
                       lastTargetHeight >= config::control::PUSH_THROUGH_MIN_HEIGHT;
    if (frontOnly && (attacking || pushThroughActive) && close) {
      if (!pushThroughActive) {
        pushThroughActive = true;
        pushThroughStartMs = nowMs;
      }
      holdEscape = nowMs - pushThroughStartMs < config::control::PUSH_THROUGH_MS;
    }
  }

  // Floor escape wins over everything but safety, the opening move included.
  // Red while locked on a far tower = the center circle: go around it.
  if (!holdEscape && !EscapeMove::active() && !CircleBypass::active() && floor.mask != 0) {
    pushThroughActive = false;
    const bool wasCharging = charging;
    const bool wasForking = ForkAttack::phase(nowMs) != ForkPhase::Idle;
    OpeningMove::cancel();
    ForkAttack::cancel();
    stopCharge();
    forgetMotion();
    escapeColorNow = floor.firstColor;
    const bool frontHit = floor.first == HazardSide::FrontLeft ||
                          floor.first == HazardSide::FrontRight ||
                          floor.first == HazardSide::FrontBoth;
    const bool centerCircle = frontHit && floor.firstColor == FloorColor::Red && targetKnown &&
                              nowMs - lastTargetMs < config::circle::LOCK_RECENT_MS &&
                              lastTargetHeight < config::circle::TOWER_MAX_HEIGHT;
    if (centerCircle) {
      // Circle on the left (front left first) -> turn right; both front -> toward the tower.
      const bool turnRight = floor.first == HazardSide::FrontLeft ||
                             (floor.first == HazardSide::FrontBoth && lastTargetX >= TARGET_CENTER_X);
      CircleBypass::start(nowMs, turnRight);
      bypassTurnedRight = turnRight;
      bypassStartMs = nowMs;
      EventLogger::logText(static_cast<uint16_t>(EventId::Custom), LogSeverity::Info,
                           LogSource::System, "circle_bypass", turnRight ? 1 : 0, lastTargetHeight, 0);
      RunLog::push(RunLogKind::Bypass, turnRight ? 1 : 0, lastTargetHeight, lastTargetX);
    } else {
      // 11/11: pushing a close opponent onto the color -> back straight out,
      // facing it. A red hit needs a closer opponent (center circle). The
      // same front side too often in a row -> turn as before (no loop).
      using namespace config::control;
      const uint16_t minHeight = floor.firstColor == FloorColor::Red ? STRAIGHT_MIN_HEIGHT_RED
                                                                     : STRAIGHT_MIN_HEIGHT;
      bool straight = STRAIGHT_BACK_ENABLED && frontHit && !floor.rear() &&
                      (wasCharging || wasForking) && targetKnown &&
                      nowMs - lastTargetMs < LOCKON_RECENT_MS && lastTargetHeight >= minHeight;
      if (straight) {
        if (floor.first != straightSide || nowMs - straightWindowMs > STRAIGHT_REPEAT_WINDOW_MS) {
          straightSide = floor.first;
          straightWindowMs = nowMs;
          straightCount = 0;
        }
        if (straightCount >= STRAIGHT_REPEAT_MAX) {
          straight = false;      // looping on one spot: turn out this time
          straightCount = 0;
          straightSide = HazardSide::None;
        } else {
          ++straightCount;
        }
      }
      EscapeMove::start(nowMs, floor, wasCharging, straight);
      RunLog::push(RunLogKind::Escape, static_cast<uint16_t>(floor.first), floor.mask,
                   static_cast<uint16_t>(floor.firstColor),
                   static_cast<uint16_t>((wasCharging || wasForking ? 1 : 0) | (straight ? 2 : 0)));
    }
  }
  if (CircleBypass::active()) {
    transitionTo(RobotState::CircleBypass);
    BypassResult result = BypassResult::Running;
    const DriveIntent out = CircleBypass::update(nowMs, floor, result);
    if (result == BypassResult::Running) return out;
    if (result == BypassResult::NeedEscape) EscapeMove::start(nowMs, floor);
    // The bypass turned the robot away from the circle, so a tower not seen
    // since then is now on the other side: search that way at once (11/10
    // field: it spun away from the tower using the x from before the turn).
    if (!targetVisible && static_cast<int32_t>(lastTargetMs - bypassStartMs) <= 0) {
      targetKnown = true;
      lastTargetX = bypassTurnedRight ? 0 : static_cast<uint16_t>(2 * TARGET_CENTER_X - 1);
      lastTargetMs = nowMs - ALIGN_LOST_HOLD_MS;
    }
    // Done: fall through to tracking, lock-on kept.
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

  // Fork attack in progress (brake -> insert -> burst): timed, steering at the
  // tower while it is seen. Escape and safety above cancel it.
  const ForkPhase fork = ForkAttack::phase(nowMs);
  if (fork != ForkPhase::Idle) {
    transitionTo(fork == ForkPhase::Brake    ? RobotState::DiveBrake
                 : fork == ForkPhase::Insert ? RobotState::SpokeEngaged
                                             : RobotState::Push);
    const int32_t towerError =
        targetVisible ? static_cast<int32_t>(vision.targetX) - TARGET_CENTER_X : 0;
    return ForkAttack::intent(nowMs, towerError);
  }

  if (targetVisible) {
    const bool fresh = newFrame(vision);
    const int32_t error = static_cast<int32_t>(vision.targetX) - TARGET_CENTER_X;
    const int32_t magnitude = error < 0 ? -error : error;

    if (charging) {
      if (magnitude > CHARGE_ABORT_ERROR) {
        stopCharge();   // drifted too far: turn in place again
      } else {
        // Close and centered while charging: fork attack (brake, slide under, burst).
        if (config::fork::ENABLED && ForkAttack::ready(nowMs) &&
            vision.targetHeight >= config::fork::TRIGGER_HEIGHT &&
            magnitude <= config::fork::TRIGGER_CENTER) {
          ForkAttack::start(nowMs);
          transitionTo(RobotState::DiveBrake);
          return ForkAttack::intent(nowMs, error);
        }
        transitionTo(RobotState::Approach);
        return intent(DriveIntentType::Approach, CHARGE_DUTY, chargeSteering(error));
      }
    }
    transitionTo(RobotState::TrackAlign);
    if (fresh) trackVelocity(vision);
    // Settled robots only turn again past the wider resume band (no rocking at the edge).
    const int32_t window = settled ? ALIGN_RESUME_ERROR : TARGET_CENTER_DEADBAND;
    if (magnitude > window) {
      settled = false;
      centeredFrames = 0;
      // Where the tower will be after the delay and coasting: stop now if it
      // reaches the window (or crosses the center) by then.
      const float predicted = error + velocityX * static_cast<float>(ALIGN_LEAD_MS);
      const bool crosses = (predicted < 0) != (error < 0);
      if (crosses || fabsf(predicted) <= TARGET_CENTER_DEADBAND) return intent(DriveIntentType::Brake);
      return intent(error < 0 ? DriveIntentType::AlignLeft : DriveIntentType::AlignRight, alignDuty(error));
    }
    settled = true;
    // Inside the align window (+-TARGET_CENTER_DEADBAND) every new frame counts.
    // 11/10 field log: the old +-150 charge window left a dead band 150..200 px
    // where it neither turned nor charged, and the robot braked there for good.
    // The charge steering closes the rest on the way in.
    if (fresh && centeredFrames < UINT8_MAX) ++centeredFrames;
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
  velocityKnown = false;   // a gap: the next frame starts a new speed estimate
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
