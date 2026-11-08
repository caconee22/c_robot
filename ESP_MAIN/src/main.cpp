#include <Arduino.h>

#include "app/boot_manager.hpp"
#include "communication/raspberry_link.hpp"
#include "control/drive_logic.hpp"
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
      StatusLed::setBoth(LedOwner::Fsm, RobotFsm::escapeColor() == FloorColor::Blue
                                            ? LedState::EscapeBlue
                                            : LedState::EscapeRed);
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  const BootResult boot = BootManager::begin();
  if (!boot.safeToRun) stayInBootFailure();

  RobotFsm::begin();
  BatteryMonitor::begin(millis());
  ImuMonitor::begin(millis());
  TiltGuard::begin();
  EventLogger::logText(static_cast<uint16_t>(EventId::Custom),
                       LogSeverity::Info, LogSource::System,
                       "fsm_ready");
}

void loop() {
  uint32_t nowMs = millis();

  // Safety comes first; each sensor step only performs short I2C transfers.
  SystemManager::update(nowMs);
  RaspberryLink::poll();
  SensorManager::update();
  BatteryMonitor::update(millis());
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
  const VisionSnapshot vision = RaspberryLink::latest();
  const DriveIntent driveIntent =
      RobotFsm::update(nowMs, sensors, vision, system.safety);
  const MotorCommand command = DriveLogic::makeCommand(nowMs, driveIntent);

  if (system.motorAllowed) {
    if (!MotorController::apply(command, system.safety) &&
        MotorController::status().lastResult == MotorControlResult::DriverFailure) {
      SystemManager::setMotorHealthy(false, FaultCode::Driver);
    }
  } else {
    MotorController::brake();
  }

  showRobotState(RobotFsm::state());
  StatusLed::update(nowMs);
  EventLogger::process();
  delay(1);  // Yield to the watchdog and ESP32 background tasks.
}
