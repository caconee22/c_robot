#include <Arduino.h>

#include "app/boot_manager.hpp"
#include "communication/raspberry_link.hpp"
#include "control/drive_logic.hpp"
#include "control/robot_fsm.hpp"
#include "motor/motor_controller.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"

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
      StatusLed::setBoth(LedOwner::Fsm, LedState::Searching);
      break;
    case RobotState::TargetConfirm:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Tracking);
      break;
    case RobotState::TrackAlign:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Aligning);
      break;
    case RobotState::Approach:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Approaching);
      break;
    case RobotState::DiveReady:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Charging);
      break;
    case RobotState::DiveBrake:
    case RobotState::SpokeEngaged:
    case RobotState::Push:
      StatusLed::setBoth(LedOwner::Fsm, LedState::Pushing);
      break;
    case RobotState::Backoff:
    case RobotState::ZoneEscape:
      StatusLed::setBoth(LedOwner::Fsm, LedState::ZoneEscape);
      break;
    case RobotState::LostTarget:
      StatusLed::setBoth(LedOwner::Fsm, LedState::CommunicationLost);
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
