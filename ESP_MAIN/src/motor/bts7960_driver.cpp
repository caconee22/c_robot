#include "motor/bts7960_driver.hpp"

#include <freertos/FreeRTOS.h>

namespace {
volatile bool driveInhibit = true;
volatile bool permanentDriveInhibit = false;
portMUX_TYPE driveInhibitMux = portMUX_INITIALIZER_UNLOCKED;
}

Bts7960Driver::Bts7960Driver(const Bts7960Pins& pins) : pins_(pins) {}

void Bts7960Driver::initializeDriveInhibit() {
  portENTER_CRITICAL(&driveInhibitMux);
  driveInhibit = true;
  permanentDriveInhibit = false;
  portEXIT_CRITICAL(&driveInhibitMux);
}

void Bts7960Driver::latchDriveInhibit() {
  portENTER_CRITICAL(&driveInhibitMux);
  driveInhibit = true;
  portEXIT_CRITICAL(&driveInhibitMux);
}

void Bts7960Driver::latchPermanentDriveInhibit() {
  portENTER_CRITICAL(&driveInhibitMux);
  driveInhibit = true;
  permanentDriveInhibit = true;
  portEXIT_CRITICAL(&driveInhibitMux);
}

void IRAM_ATTR Bts7960Driver::latchPermanentDriveInhibitFromIsr() {
  portENTER_CRITICAL_ISR(&driveInhibitMux);
  driveInhibit = true;
  permanentDriveInhibit = true;
  portEXIT_CRITICAL_ISR(&driveInhibitMux);
}

bool Bts7960Driver::clearDriveInhibit() {
  portENTER_CRITICAL(&driveInhibitMux);
  const bool allowed = !permanentDriveInhibit;
  if (allowed) {
    driveInhibit = false;
  }
  portEXIT_CRITICAL(&driveInhibitMux);
  return allowed;
}

bool Bts7960Driver::begin() {
  pinMode(pins_.len, OUTPUT);
  pinMode(pins_.ren, OUTPUT);
  digitalWrite(pins_.len, LOW);
  digitalWrite(pins_.ren, LOW);

  const double lpwmFrequency = ledcSetup(
      pins_.lpwmChannel, config::motor::PWM_FREQUENCY_HZ,
      config::motor::PWM_RESOLUTION_BITS);
  const double rpwmFrequency = ledcSetup(
      pins_.rpwmChannel, config::motor::PWM_FREQUENCY_HZ,
      config::motor::PWM_RESOLUTION_BITS);
  ledcAttachPin(pins_.lpwm, pins_.lpwmChannel);
  ledcAttachPin(pins_.rpwm, pins_.rpwmChannel);

  status_.initialized = lpwmFrequency > 0.0 && rpwmFrequency > 0.0;
  disable();
  return status_.initialized;
}

bool Bts7960Driver::drive(int16_t permille) {
  if (!status_.initialized) {
    disable();
    return false;
  }

  // ESTOP may already be holding EN high for a zero-PWM brake. Never write
  // a drive duty into that brake before checking the permanent inhibit.
  portENTER_CRITICAL(&driveInhibitMux);
  const bool inhibited = driveInhibit;
  portEXIT_CRITICAL(&driveInhibitMux);
  if (inhibited) return false;

  permille = constrain(permille, -config::motor::COMMAND_MAX,
                       config::motor::COMMAND_MAX);
  if (permille == 0) {
    coast();
    return true;
  }

  const bool directionChanged =
      (status_.appliedPermille > 0 && permille < 0) ||
      (status_.appliedPermille < 0 && permille > 0);
  if (directionChanged) {
    writePwm(0, 0);
    delayMicroseconds(config::motor::DIRECTION_CHANGE_DEADTIME_US);
  }

  const uint32_t magnitude = static_cast<uint32_t>(abs(permille));
  const uint32_t pwmDuty =
      (magnitude * config::motor::PWM_MAX_DUTY +
       config::motor::COMMAND_MAX / 2) /
      config::motor::COMMAND_MAX;

  // Set PWM before enabling: a previous brake must never become a drive pulse.
  if (status_.state == MotorElectricalState::Brake) setEnabled(false);
  if (permille > 0) writePwm(0, pwmDuty);
  else writePwm(pwmDuty, 0);
  portENTER_CRITICAL(&driveInhibitMux);
  if (driveInhibit) {
    portEXIT_CRITICAL(&driveInhibitMux);
    disable();
    return false;
  }
  setEnabled(true);
  portEXIT_CRITICAL(&driveInhibitMux);
  status_.state = permille > 0 ? MotorElectricalState::Forward
                              : MotorElectricalState::Reverse;
  status_.appliedPermille = permille;
  status_.pwmDuty = pwmDuty;
  return true;
}

void Bts7960Driver::stop(StopMode mode) {
  if (mode == StopMode::Brake) {
    brake();
  } else {
    coast();
  }
}

void Bts7960Driver::coast() {
  writePwm(0, 0);
  setEnabled(false);
  status_.appliedPermille = 0;
  status_.pwmDuty = 0;
  status_.state = status_.initialized ? MotorElectricalState::Coast
                                      : MotorElectricalState::Disabled;
}

void Bts7960Driver::brake() {
  if (!status_.initialized) { disable(); return; }
  // Keep EN low while replacing the last drive PWM with brake PWM.
  setEnabled(false);
  MotorElectricalState electricalState = MotorElectricalState::Brake;
  uint32_t brakePwmDuty = 0;
  switch (config::motor::BRAKE_STRATEGY) {
    case config::motor::BrakeStrategy::BothPwmLowEnableHigh:
      writePwm(0, 0);
      portENTER_CRITICAL(&driveInhibitMux);
      setEnabled(true);
      portEXIT_CRITICAL(&driveInhibitMux);
      break;

    case config::motor::BrakeStrategy::BothPwmHighEnableHigh:
      writePwm(config::motor::PWM_MAX_DUTY,
               config::motor::PWM_MAX_DUTY);
      portENTER_CRITICAL(&driveInhibitMux);
      setEnabled(true);
      portEXIT_CRITICAL(&driveInhibitMux);
      brakePwmDuty = config::motor::PWM_MAX_DUTY;
      break;

    case config::motor::BrakeStrategy::Coast:
    default:
      writePwm(0, 0);
      setEnabled(false);
      electricalState = MotorElectricalState::Coast;
      break;
  }

  status_.appliedPermille = 0;
  status_.pwmDuty = brakePwmDuty;
  status_.state = electricalState;
}

void Bts7960Driver::disable() {
  writePwm(0, 0);
  setEnabled(false);
  status_.appliedPermille = 0;
  status_.pwmDuty = 0;
  status_.state = MotorElectricalState::Disabled;
}

bool Bts7960Driver::isInitialized() const { return status_.initialized; }

Bts7960Status Bts7960Driver::status() const { return status_; }

void Bts7960Driver::writePwm(uint32_t lpwmDuty, uint32_t rpwmDuty) {
  ledcWrite(pins_.lpwmChannel,
            min(lpwmDuty, config::motor::PWM_MAX_DUTY));
  ledcWrite(pins_.rpwmChannel,
            min(rpwmDuty, config::motor::PWM_MAX_DUTY));
}

void Bts7960Driver::setEnabled(bool enabled) {
  digitalWrite(pins_.len, enabled ? HIGH : LOW);
  digitalWrite(pins_.ren, enabled ? HIGH : LOW);
  status_.enabled = enabled;
}
