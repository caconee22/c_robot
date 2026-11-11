#include <Arduino.h>
#include <math.h>

#include "app/boot_manager.hpp"
#include "app/boot_mode.hpp"
#if defined(COMBINED_BENCH) && COMBINED_BENCH
#include "bench_v2/bench_v2_app.hpp"
#endif
#include "communication/vision_link.hpp"
#include "communication/run_log.hpp"
#include "control/align_fsm.hpp"
#include "control/control_config.hpp"
#include "control/drive_logic.hpp"
#include "control/fork_attack.hpp"
#include "control/opening_move.hpp"
#include "control/pose_estimate.hpp"
#include "control/robot_fsm.hpp"
#include "motor/motor_controller.hpp"
#include "sensors/imu_monitor.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/battery_monitor.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"
#include "system/tilt_guard.hpp"

namespace {

[[noreturn]] void stayInBootFailure() {
  while (true) {
    BootManager::staySafe();
    StatusLed::update(millis());
    delay(10);
  }
}

void showRobotState(RobotState state) {
  static RobotState previous = RobotState::Boot;
  if (state == previous) return;
  previous = state;
  StatusLed::clear(LedOwner::Fsm);
  switch (state) {
    case RobotState::Search:
    case RobotState::LostTarget:
    case RobotState::Wander:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Searching);
      break;
    case RobotState::TargetConfirm:
    case RobotState::TrackAlign:
    case RobotState::Approach:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Tracking);
      break;
    case RobotState::DiveReady:
    case RobotState::DiveBrake:
    case RobotState::SpokeEngaged:
    case RobotState::Push:
    case RobotState::Backoff:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Attacking);
      break;
    case RobotState::ZoneEscape:
    case RobotState::CircleBypass:
      StatusLed::setBoth(LedOwner::Fsm, (config::control::ALIGN_ONLY_FSM ? AlignFsm::escapeColor()
                                                                          : RobotFsm::escapeColor()) == FloorColor::Blue
                                            ? LedState::EscapeBlue
                                            : LedState::EscapeRed);
      break;
    default:
      break;
  }
}

// 11/11 two-button soft reset: the drive side goes back to its power-on
// state (FSM, helpers, tilt and pose learning, run log). Sensors, IMU,
// camera link, calibration and motor trim are kept as they are.
void softReset() {
  if (config::control::ALIGN_ONLY_FSM) AlignFsm::begin(); else RobotFsm::begin();
  ForkAttack::cancel();
  TiltGuard::begin();
  PoseEstimate::reset();
  RunLog::reset();
}

}  // namespace

// 11/11: COMBINED_BENCH builds hold the bench too (app/boot_mode.hpp): both
// buttons held 1 s at power-on -> bench for the whole power cycle.
bool benchMode = false;

void setup() {
#if defined(COMBINED_BENCH) && COMBINED_BENCH
  if (BootMode::benchRequested()) {
    benchMode = true;
    BenchV2App::begin();
    return;
  }
#endif
  const BootResult boot = BootManager::begin();
  if (!boot.safeToRun) stayInBootFailure();

  if (config::control::ALIGN_ONLY_FSM) AlignFsm::begin(); else RobotFsm::begin();
  SystemManager::setSoftResetHandler(softReset);
  BatteryMonitor::begin(millis());
  ImuMonitor::begin(millis());
  TiltGuard::begin();
  EventLogger::logText(static_cast<uint16_t>(EventId::Custom),
                       LogSeverity::Info, LogSource::System,
                       "fsm_ready");
}

void loop() {
#if defined(COMBINED_BENCH) && COMBINED_BENCH
  if (benchMode) {
    BenchV2App::update();
    return;
  }
#endif
  uint32_t nowMs = millis();

  // Safety comes first; each sensor step only performs short I2C transfers.
  SystemManager::update(nowMs);
  VisionLink::poll();
  SensorManager::update();
  BatteryMonitor::update(millis());
  // Opening move duty follows the pack type found at power-on (3S vs 4S), Ok only.
  OpeningMove::setBatteryCells(BatteryMonitor::status().result == BatteryCheckResult::Ok
                                   ? BatteryMonitor::status().cells : 0);
  ImuMonitor::update(millis());
  // Upside down (past the tilt limit) for too long: permanent emergency stop.
  if (TiltGuard::update(millis(), SystemManager::running(), ImuMonitor::status(),
                        ImuMonitor::latest())) {
    SystemManager::triggerEmergencyStop();
  }
  nowMs = millis();
  SystemManager::update(nowMs);
  const SystemStatus system = SystemManager::status();

  const SensorSnapshot sensors = SensorManager::snapshot();
  const VisionSnapshot vision = VisionLink::latest();
  // 11/10: ALIGN_ONLY_FSM = the rebuilt FSM (opening, escape, charge, align; control/align_fsm.hpp).
  const DriveIntent driveIntent =
      config::control::ALIGN_ONLY_FSM ? AlignFsm::update(nowMs, sensors, vision, system.safety)
                                      : RobotFsm::update(nowMs, sensors, vision, system.safety);
  const MotorCommand command = DriveLogic::makeCommand(nowMs, driveIntent);

  if (system.motorAllowed) {
    if (!MotorController::apply(command, system.safety) &&
        MotorController::status().lastResult == MotorControlResult::DriverFailure) {
      SystemManager::setMotorHealthy(false, FaultCode::Driver);
    }
  } else {
    MotorController::brake();
  }

  // Run log to the Pi over UART1 (11/10): 5 Hz alive frame while driving, low priority.
  const bool aliveSent = RunLog::alive(nowMs, system.motorAllowed && config::control::ALIGN_ONLY_FSM,
                static_cast<uint8_t>(AlignFsm::state()),
                vision.linkValid && vision.frameFresh && vision.cameraOk && vision.pipelineOk &&
                        vision.targetValid
                    ? vision.targetX
                    : 0,
                static_cast<uint8_t>(driveIntent.type), driveIntent.requestedDuty);
  // Rough pose for the log only (11/10): gyro heading + commanded distance.
  PoseEstimate::update(nowMs, system.motorAllowed, ImuMonitor::status(), ImuMonitor::latest(),
                       MotorController::status().appliedLeftPermille,
                       MotorController::status().appliedRightPermille);
  if (aliveSent) {
    const Pose pose = PoseEstimate::pose();
    RunLog::push(RunLogKind::Pose, static_cast<uint16_t>(static_cast<int16_t>(lroundf(pose.headingDeg * 10))),
                 static_cast<uint16_t>(static_cast<int16_t>(constrain(lroundf(pose.xMm), -32000L, 32000L))),
                 static_cast<uint16_t>(static_cast<int16_t>(constrain(lroundf(pose.yMm), -32000L, 32000L))),
                 pose.imuOk ? 1 : 0);
  }
  showRobotState(config::control::ALIGN_ONLY_FSM ? AlignFsm::state() : RobotFsm::state());
  StatusLed::update(nowMs);
  EventLogger::process();
  RunLog::process();
  delay(1);  // Yield to the watchdog and ESP32 background tasks.
}
