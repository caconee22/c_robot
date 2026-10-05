#include "system/system_manager.hpp"

#include "hardware/reset_manager.hpp"
#include "interrupts/button_interrupts.hpp"
#include "motor/motor_controller.hpp"
#include "system/status_led.hpp"

namespace {

SystemStatus latestStatus;
FaultCode sensorFault = FaultCode::Unknown;
FaultCode motorFault = FaultCode::Unknown;
FaultCode applicationFault = FaultCode::Unknown;
int32_t sensorFaultDetail = 0;
int32_t motorFaultDetail = 0;
int32_t applicationFaultDetail = 0;
uint32_t lastMotorWatchdogCount = 0;

SafetyState safetyStateFor(SystemState state) {
  if (state == SystemState::Running) return SafetyState::Running;
  if (state == SystemState::EmergencyStop) return SafetyState::EmergencyStop;
  if (state == SystemState::WaitingForStart) {
    return SafetyState::WaitingForStart;
  }
  if (state == SystemState::Calibration) return SafetyState::WaitingForStart;
  return SafetyState::ResetLockout;
}

uint32_t calculateFaultFlags() {
  uint32_t flags = SystemFaultNone;
  if (latestStatus.rearmRequired) flags |= SystemFaultResetRearm;
  if (!latestStatus.sensorsHealthy) flags |= SystemFaultSensor;
  if (!latestStatus.motorsHealthy) flags |= SystemFaultMotor;
  if (!latestStatus.applicationHealthy) flags |= SystemFaultApplication;
  if (ButtonInterrupts::emergencyLatched()) flags |= SystemFaultEmergencyStop;
  return flags;
}

SystemState selectState() {
  if (ButtonInterrupts::emergencyLatched()) return SystemState::EmergencyStop;
  if (!latestStatus.applicationHealthy) return SystemState::SystemFault;
  if (!latestStatus.motorsHealthy) return SystemState::MotorFault;
  if (!latestStatus.sensorsHealthy) return SystemState::SensorFault;
  if (latestStatus.calibrationActive) return SystemState::Calibration;
  if (!latestStatus.runRequested) return SystemState::WaitingForStart;
  return SystemState::Running;
}

void applyLedState(SystemState state) {
  StatusLed::clear(LedOwner::System);
  StatusLed::clear(LedOwner::Safety);
  StatusLed::clear(LedOwner::Sensors);
  StatusLed::clear(LedOwner::Motors);

  switch (state) {
    case SystemState::Booting:
      StatusLed::setBoth(LedOwner::System, LedState::Booting);
      break;
    case SystemState::WaitingForStart:
      StatusLed::setBoth(LedOwner::System, LedState::WaitingForStart);
      break;
    case SystemState::Running:
      StatusLed::setBoth(LedOwner::System, LedState::Ready);
      break;
    case SystemState::Calibration:
      StatusLed::setBoth(LedOwner::System, LedState::Charging);
      break;
    case SystemState::SensorFault:
      StatusLed::setBoth(LedOwner::Sensors, LedState::SensorFault);
      break;
    case SystemState::MotorFault:
      StatusLed::setBoth(LedOwner::Motors, LedState::MotorFault);
      break;
    case SystemState::SystemFault:
      StatusLed::setBoth(LedOwner::System, LedState::Fault);
      break;
    case SystemState::EmergencyStop:
      StatusLed::setEmergencyStop();
      break;
  }
}

void recordTransition(SystemState previous, SystemState next) {
  EventLogger::valueChanged(EventId::SystemStateChanged, LogSource::System,
                            static_cast<int32_t>(next));
  if (next == SystemState::Running) {
    EventLogger::log(EventId::RunStarted);
  } else if (previous == SystemState::Running) {
    EventLogger::log(EventId::RunStopped);
  }
  applyLedState(next);
}

void updateState(uint32_t nowMs) {
  const SystemState previous = latestStatus.state;
  const SystemState next = selectState();
  latestStatus.state = next;
  latestStatus.motorAllowed = next == SystemState::Running;
  latestStatus.faultFlags = calculateFaultFlags();
  latestStatus.safety.state = safetyStateFor(next);
  latestStatus.safety.motorAllowed = latestStatus.motorAllowed;
  latestStatus.safety.faultFlags = latestStatus.faultFlags;

  if (previous != next) {
    latestStatus.stateEnteredMs = nowMs;
    ++latestStatus.transitionCount;
    recordTransition(previous, next);
  }
}

void setHealth(bool healthy, bool& destination, FaultCode fault,
               int32_t detail, FaultCode& storedFault,
               int32_t& storedDetail, LogSource source) {
  const bool changed = destination != healthy ||
                       (!healthy &&
                        (storedFault != fault || storedDetail != detail));
  if (!changed) return;

  if (!destination) {
    EventLogger::faultChanged(source, storedFault, false, storedDetail);
  }
  destination = healthy;
  if (!healthy) {
    latestStatus.runRequested = false;
    storedFault = fault;
    storedDetail = detail;
    EventLogger::faultChanged(source, fault, true, detail);
    MotorController::brake();
  }
  latestStatus.faultFlags = calculateFaultFlags();
  latestStatus.safety.faultFlags = latestStatus.faultFlags;
  updateState(millis());
}

}  // namespace

void SystemManager::begin() {
  const MotorControllerStatus motorStatus = MotorController::status();
  latestStatus = SystemStatus{};
  latestStatus.state = SystemState::Booting;
  latestStatus.rearmRequired = ResetManager::requiresRearm();
  latestStatus.motorsHealthy = motorStatus.initialized;
  latestStatus.stateEnteredMs = millis();
  latestStatus.safety.state = SafetyState::ResetLockout;
  latestStatus.faultFlags = calculateFaultFlags();
  latestStatus.safety.faultFlags = latestStatus.faultFlags;

  sensorFault = FaultCode::Unknown;
  motorFault = motorStatus.initialized ? FaultCode::Unknown
                                       : FaultCode::Initialization;
  applicationFault = FaultCode::Unknown;
  sensorFaultDetail = 0;
  motorFaultDetail = 0;
  applicationFaultDetail = 0;
  lastMotorWatchdogCount = motorStatus.watchdogStopCount;

  if (!motorStatus.initialized) {
    EventLogger::faultChanged(LogSource::Motor, FaultCode::Initialization,
                              true, 0);
  }
  applyLedState(SystemState::Booting);
}

void SystemManager::update(uint32_t nowMs) {
  ButtonInterrupts::update(nowMs);
  if (ButtonInterrupts::emergencyLatched()) {
    latestStatus.state = SystemState::EmergencyStop;
    latestStatus.motorAllowed = false;
    latestStatus.safety.state = SafetyState::EmergencyStop;
    latestStatus.safety.motorAllowed = false;
    latestStatus.faultFlags = calculateFaultFlags();
    latestStatus.safety.faultFlags = latestStatus.faultFlags;
    applyLedState(SystemState::EmergencyStop);
    MotorController::emergencyStop();
    ButtonInterrupts::enterEmergencyStopLoop();
  }

  if (ButtonInterrupts::consumeStartEvent()) {
    if (latestStatus.calibrationActive || !latestStatus.motorsHealthy ||
        !latestStatus.applicationHealthy || !latestStatus.sensorsHealthy) {
      latestStatus.runRequested = false;
      EventLogger::log(EventId::SafetyBlocked, 1, 0);
    } else {
      latestStatus.runRequested = true;
      latestStatus.rearmRequired = false;
      ++latestStatus.startEventCount;
      latestStatus.lastStartEventMs = nowMs;
    }
  }

  const SystemState previous = latestStatus.state;
  updateState(nowMs);

  if (latestStatus.state == SystemState::Running &&
      previous != SystemState::Running) {
    if (!MotorController::clearEmergencyStop(latestStatus.safety)) {
      setMotorHealthy(false, FaultCode::Driver, 0);
      updateState(nowMs);
    }
  } else if (previous == SystemState::Running &&
             latestStatus.state != SystemState::Running) {
    MotorController::brake();
  }

  MotorController::update(nowMs, latestStatus.safety);
  const MotorControllerStatus motor = MotorController::status();
  if (motor.watchdogStopCount != lastMotorWatchdogCount) {
    lastMotorWatchdogCount = motor.watchdogStopCount;
    EventLogger::log(EventId::MotorWatchdogStop,
                     static_cast<int32_t>(lastMotorWatchdogCount));
  }
  if (motor.lastResult == MotorControlResult::DriverFailure ||
      !motor.initialized) {
    setMotorHealthy(false, FaultCode::Driver, 0);
  }
}

void SystemManager::requestStop() {
  latestStatus.runRequested = false;
  updateState(millis());
  MotorController::brake();
}

void SystemManager::setCalibrationActive(bool active) {
  if (latestStatus.calibrationActive == active) return;
  latestStatus.calibrationActive = active;
  if (active) {
    latestStatus.runRequested = false;
    MotorController::brake();
  }
  updateState(millis());
}

void SystemManager::setSensorHealthy(bool healthy, FaultCode fault,
                                     int32_t detail) {
  setHealth(healthy, latestStatus.sensorsHealthy, fault, detail, sensorFault,
            sensorFaultDetail, LogSource::Sensor);
}

void SystemManager::setMotorHealthy(bool healthy, FaultCode fault,
                                    int32_t detail) {
  setHealth(healthy, latestStatus.motorsHealthy, fault, detail, motorFault,
            motorFaultDetail, LogSource::Motor);
}

void SystemManager::setApplicationHealthy(bool healthy, FaultCode fault,
                                          int32_t detail) {
  setHealth(healthy, latestStatus.applicationHealthy, fault, detail,
            applicationFault, applicationFaultDetail, LogSource::System);
}

SystemStatus SystemManager::status() { return latestStatus; }

SafetyStatus SystemManager::safetyStatus() { return latestStatus.safety; }

SystemState SystemManager::state() { return latestStatus.state; }

bool SystemManager::running() { return state() == SystemState::Running; }

bool SystemManager::motorAllowed() { return latestStatus.motorAllowed; }

const char* SystemManager::stateName(SystemState state) {
  switch (state) {
    case SystemState::Booting: return "booting";
    case SystemState::WaitingForStart: return "waiting_for_start";
    case SystemState::Running: return "running";
    case SystemState::Calibration: return "calibration";
    case SystemState::SensorFault: return "sensor_fault";
    case SystemState::MotorFault: return "motor_fault";
    case SystemState::SystemFault: return "system_fault";
    case SystemState::EmergencyStop: return "emergency_stop";
    default: return "unknown";
  }
}
