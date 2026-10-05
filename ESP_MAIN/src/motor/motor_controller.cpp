#include "motor/motor_controller.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "motor/bts7960_driver.hpp"
#include "motor/motor_config.hpp"
#include "pins.hpp"

namespace {
Bts7960Driver leftMotor({pins::MOTOR1_LPWM, pins::MOTOR1_RPWM,
                         pins::MOTOR1_LEN, pins::MOTOR1_REN,
                         config::motor::MOTOR1_LPWM_CHANNEL,
                         config::motor::MOTOR1_RPWM_CHANNEL});
Bts7960Driver rightMotor({pins::MOTOR2_LPWM, pins::MOTOR2_RPWM,
                          pins::MOTOR2_LEN, pins::MOTOR2_REN,
                          config::motor::MOTOR2_LPWM_CHANNEL,
                          config::motor::MOTOR2_RPWM_CHANNEL});

StaticSemaphore_t motorMutexStorage;
SemaphoreHandle_t motorMutex = nullptr;
MotorControllerStatus latestStatus;
StaticTask_t watchdogTaskStorage;
StackType_t watchdogStack[2048 / sizeof(StackType_t)];
TaskHandle_t watchdogTask = nullptr;

void motorWatchdog(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(config::motor::WATCHDOG_POLL_MS));
    MotorController::checkTimeout(millis());
  }
}

class MotorLock {
 public:
  MotorLock() {
    if (motorMutex != nullptr) {
      locked_ = xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE;
    }
  }

  ~MotorLock() {
    if (locked_) {
      xSemaphoreGive(motorMutex);
    }
  }

 private:
  bool locked_ = false;
};

bool deadlinePassed(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

int16_t clampCommand(int32_t value) {
  return static_cast<int16_t>(constrain(
      value, -config::motor::COMMAND_MAX, config::motor::COMMAND_MAX));
}

int16_t applyLimit(int16_t value, int16_t limit) {
  return static_cast<int16_t>(constrain(value, -limit, limit));
}

int16_t positiveMagnitude(int16_t value) {
  int32_t magnitude = value;
  if (magnitude < 0) {
    magnitude = -magnitude;
  }
  return clampCommand(magnitude);
}

void refreshDriverStatus() {
  latestStatus.leftDriver = leftMotor.status();
  latestStatus.rightDriver = rightMotor.status();
  latestStatus.outputsActive =
      latestStatus.leftDriver.appliedPermille != 0 ||
      latestStatus.rightDriver.appliedPermille != 0;
}

void stopLocked(StopMode mode) {
  leftMotor.stop(mode);
  rightMotor.stop(mode);
  latestStatus.stopMode = mode;
  latestStatus.appliedLeftPermille = 0;
  latestStatus.appliedRightPermille = 0;
  latestStatus.commandValidUntilMs = 0;
  refreshDriverStatus();
}

void rejectLocked(MotorControlResult result, StopMode mode) {
  stopLocked(mode);
  latestStatus.lastResult = result;
  ++latestStatus.rejectedCommandCount;
}
}

bool MotorController::begin() {
  if (motorMutex == nullptr) {
    motorMutex = xSemaphoreCreateMutexStatic(&motorMutexStorage);
  }
  if (motorMutex == nullptr) {
    latestStatus = MotorControllerStatus{};
    latestStatus.lastResult = MotorControlResult::NotInitialized;
    Bts7960Driver::initializeDriveInhibit();
    return false;
  }
  MotorLock lock;

  latestStatus = MotorControllerStatus{};
  latestStatus.outputLimitPermille = config::motor::DEFAULT_OUTPUT_LIMIT;
  latestStatus.emergencyStopLatched = true;
  Bts7960Driver::initializeDriveInhibit();

  const bool leftReady = leftMotor.begin();
  const bool rightReady = rightMotor.begin();
  latestStatus.initialized = leftReady && rightReady;
  stopLocked(StopMode::Brake);
  latestStatus.lastResult = latestStatus.initialized
                                ? MotorControlResult::EmergencyStopLatched
                                : MotorControlResult::DriverFailure;
  if (latestStatus.initialized && watchdogTask == nullptr) {
    // ESP-IDF task stack size is in bytes. This task only checks motor expiry.
    watchdogTask = xTaskCreateStatic(motorWatchdog, "motor_watchdog",
                                    sizeof(watchdogStack), nullptr, 2,
                                    watchdogStack, &watchdogTaskStorage);
    if (watchdogTask == nullptr) {
      latestStatus.initialized = false;
      latestStatus.lastResult = MotorControlResult::NotInitialized;
    }
  }
  return latestStatus.initialized;
}

bool MotorController::apply(const MotorCommand& command,
                            const SafetyStatus& safety) {
  MotorLock lock;
  const uint32_t nowMs = millis();
  const uint32_t effectiveDeadline =
      !command.deadlineSet && command.validUntilMs == 0
          ? nowMs + config::motor::DEFAULT_COMMAND_TIMEOUT_MS
          : command.validUntilMs;
  latestStatus.requestedLeftPermille = command.leftPermille;
  latestStatus.requestedRightPermille = command.rightPermille;
  latestStatus.lastCommandMs = nowMs;
  latestStatus.commandValidUntilMs = effectiveDeadline;
  latestStatus.lastSafetyState = safety.state;

  if (!latestStatus.initialized) {
    rejectLocked(MotorControlResult::NotInitialized, StopMode::Brake);
    return false;
  }
  if (latestStatus.emergencyStopLatched) {
    rejectLocked(MotorControlResult::EmergencyStopLatched, StopMode::Brake);
    return false;
  }
  if (!safety.motorAllowed || safety.state != SafetyState::Running) {
    rejectLocked(MotorControlResult::SafetyBlocked, StopMode::Brake);
    return false;
  }
  if (deadlinePassed(nowMs, effectiveDeadline)) {
    rejectLocked(MotorControlResult::CommandExpired, StopMode::Brake);
    return false;
  }

  const int16_t left = applyLimit(
      clampCommand(command.leftPermille), latestStatus.outputLimitPermille);
  const int16_t right = applyLimit(
      clampCommand(command.rightPermille), latestStatus.outputLimitPermille);

  if (left == 0 && right == 0) {
    stopLocked(command.stopMode);
    latestStatus.lastResult = MotorControlResult::Accepted;
    ++latestStatus.acceptedCommandCount;
    return true;
  }

  const int16_t physicalLeft =
      config::motor::LEFT_MOTOR_INVERTED ? -left : left;
  const int16_t physicalRight =
      config::motor::RIGHT_MOTOR_INVERTED ? -right : right;
  bool leftOk = true;
  bool rightOk = true;
  if (physicalLeft == 0) {
    leftMotor.stop(command.stopMode);
  } else {
    leftOk = leftMotor.drive(physicalLeft);
  }
  if (physicalRight == 0) {
    rightMotor.stop(command.stopMode);
  } else {
    rightOk = rightMotor.drive(physicalRight);
  }
  refreshDriverStatus();

  if (!leftOk || !rightOk) {
    rejectLocked(MotorControlResult::DriverFailure, StopMode::Brake);
    return false;
  }

  // Controller-level applied values stay in logical robot coordinates. The
  // nested driver values expose the physical values after inversion.
  latestStatus.appliedLeftPermille = left;
  latestStatus.appliedRightPermille = right;
  latestStatus.stopMode = command.stopMode;
  latestStatus.lastResult = MotorControlResult::Accepted;
  ++latestStatus.acceptedCommandCount;
  return true;
}

bool MotorController::setTank(int16_t leftPermille, int16_t rightPermille,
                              const SafetyStatus& safety,
                              uint32_t timeoutMs) {
  const uint32_t nowMs = millis();
  if (timeoutMs == 0) {
    timeoutMs = config::motor::DEFAULT_COMMAND_TIMEOUT_MS;
  }
  MotorCommand command;
  command.leftPermille = leftPermille;
  command.rightPermille = rightPermille;
  command.stopMode = StopMode::Brake;
  command.validUntilMs = nowMs + timeoutMs;
  command.deadlineSet = true;
  return apply(command, safety);
}

bool MotorController::setArcade(int16_t throttlePermille,
                                int16_t steeringPermille,
                                const SafetyStatus& safety,
                                uint32_t timeoutMs) {
  const int16_t throttle = clampCommand(throttlePermille);
  const int16_t steering = clampCommand(steeringPermille);
  return setTank(clampCommand(static_cast<int32_t>(throttle) + steering),
                 clampCommand(static_cast<int32_t>(throttle) - steering),
                 safety, timeoutMs);
}

bool MotorController::forward(int16_t permille, const SafetyStatus& safety,
                              uint32_t timeoutMs) {
  const int16_t speed = positiveMagnitude(permille);
  return setTank(speed, speed, safety, timeoutMs);
}

bool MotorController::reverse(int16_t permille, const SafetyStatus& safety,
                              uint32_t timeoutMs) {
  const int16_t speed = positiveMagnitude(permille);
  return setTank(static_cast<int16_t>(-speed),
                 static_cast<int16_t>(-speed), safety, timeoutMs);
}

bool MotorController::rotateLeft(int16_t permille,
                                 const SafetyStatus& safety,
                                 uint32_t timeoutMs) {
  const int16_t speed = positiveMagnitude(permille);
  return setTank(static_cast<int16_t>(-speed), speed, safety, timeoutMs);
}

bool MotorController::rotateRight(int16_t permille,
                                  const SafetyStatus& safety,
                                  uint32_t timeoutMs) {
  const int16_t speed = positiveMagnitude(permille);
  return setTank(speed, static_cast<int16_t>(-speed), safety, timeoutMs);
}

void MotorController::stop(StopMode mode) {
  MotorLock lock;
  stopLocked(mode);
  latestStatus.lastResult = latestStatus.initialized
                                ? MotorControlResult::Accepted
                                : MotorControlResult::NotInitialized;
}

void MotorController::coast() { stop(StopMode::Coast); }

void MotorController::brake() { stop(StopMode::Brake); }

void MotorController::emergencyStop() {
  MotorLock lock;
  Bts7960Driver::latchDriveInhibit();
  latestStatus.emergencyStopLatched = true;
  if (latestStatus.initialized) {
    stopLocked(StopMode::Brake);
  }
  latestStatus.lastResult = MotorControlResult::EmergencyStopLatched;
}

bool MotorController::clearEmergencyStop(const SafetyStatus& safety) {
  MotorLock lock;
  if (!latestStatus.initialized || !safety.motorAllowed ||
      safety.state != SafetyState::Running) {
    latestStatus.lastResult = !latestStatus.initialized
                                  ? MotorControlResult::NotInitialized
                                  : MotorControlResult::SafetyBlocked;
    return false;
  }

  if (!Bts7960Driver::clearDriveInhibit()) {
    latestStatus.lastResult = MotorControlResult::EmergencyStopLatched;
    return false;
  }

  stopLocked(StopMode::Brake);
  latestStatus.emergencyStopLatched = false;
  latestStatus.lastResult = MotorControlResult::Accepted;
  return true;
}

void MotorController::update(uint32_t nowMs, const SafetyStatus& safety) {
  MotorLock lock;
  latestStatus.lastSafetyState = safety.state;
  if (!latestStatus.initialized) {
    return;
  }
  if (safety.state == SafetyState::EmergencyStop) {
    Bts7960Driver::latchDriveInhibit();
    latestStatus.emergencyStopLatched = true;
    if (latestStatus.outputsActive) {
      stopLocked(StopMode::Brake);
    }
    latestStatus.lastResult = MotorControlResult::EmergencyStopLatched;
    return;
  }
  if (!safety.motorAllowed) {
    if (latestStatus.outputsActive) {
      stopLocked(StopMode::Brake);
    }
    latestStatus.lastResult = MotorControlResult::SafetyBlocked;
    return;
  }
  if (latestStatus.outputsActive &&
      deadlinePassed(nowMs, latestStatus.commandValidUntilMs)) {
    rejectLocked(MotorControlResult::CommandExpired, StopMode::Brake);
    ++latestStatus.watchdogStopCount;
  }
}

void MotorController::checkTimeout(uint32_t nowMs) {
  MotorLock lock;
  if (latestStatus.initialized && latestStatus.outputsActive &&
      deadlinePassed(nowMs, latestStatus.commandValidUntilMs)) {
    rejectLocked(MotorControlResult::CommandExpired, StopMode::Brake);
    ++latestStatus.watchdogStopCount;
  }
}

bool MotorController::setOutputLimit(int16_t limitPermille) {
  MotorLock lock;
  if (limitPermille < 0 ||
      limitPermille > config::motor::ABSOLUTE_OUTPUT_LIMIT) {
    latestStatus.lastResult = MotorControlResult::InvalidArgument;
    return false;
  }
  if (latestStatus.outputsActive) {
    stopLocked(StopMode::Brake);
  }
  latestStatus.outputLimitPermille = limitPermille;
  latestStatus.lastResult = MotorControlResult::Accepted;
  return true;
}

int16_t MotorController::outputLimit() {
  MotorLock lock;
  return latestStatus.outputLimitPermille;
}

MotorControllerStatus MotorController::status() {
  MotorLock lock;
  refreshDriverStatus();
  return latestStatus;
}

const char* MotorController::resultName(MotorControlResult result) {
  switch (result) {
    case MotorControlResult::Accepted:
      return "accepted";
    case MotorControlResult::NotInitialized:
      return "not_initialized";
    case MotorControlResult::SafetyBlocked:
      return "safety_blocked";
    case MotorControlResult::EmergencyStopLatched:
      return "emergency_stop_latched";
    case MotorControlResult::CommandExpired:
      return "command_expired";
    case MotorControlResult::InvalidArgument:
      return "invalid_argument";
    case MotorControlResult::DriverFailure:
      return "driver_failure";
    default:
      return "unknown";
  }
}
