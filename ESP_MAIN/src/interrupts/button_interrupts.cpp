#include "interrupts/button_interrupts.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <soc/gpio_struct.h>

#include "config.hpp"
#include "motor/motor_config.hpp"
#include "motor/bts7960_driver.hpp"
#include "motor/motor_controller.hpp"
#include "pins.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"

namespace {
volatile bool emergencyIsLatched = false;
volatile bool emergencyEventPending = false;
volatile bool startEdgePending = false;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

bool startDebounceActive = false;
bool startRawPressed = false;
bool startStablePressed = false;
bool startPressQualified = false;
bool startHasSeenReleased = false;
uint32_t startRawChangedMs = 0;
uint32_t startLastStableChangeMs = 0;
volatile uint32_t pendingStartEvents = 0;

void IRAM_ATTR disableMotorEnPins() {
  // V1 EN pins span both register banks. Keep ISR masks tied to pins.hpp.
  GPIO.out_w1tc = (1UL << pins::MOTOR2_REN) | (1UL << pins::MOTOR2_LEN);
  GPIO.out1_w1tc.val = (1UL << (pins::MOTOR1_REN - 32)) |
                      (1UL << (pins::MOTOR1_LEN - 32));
}

void IRAM_ATTR onEmergencyPressed() {
  Bts7960Driver::latchPermanentDriveInhibitFromIsr();
  disableMotorEnPins();
  portENTER_CRITICAL_ISR(&buttonMux);
  emergencyIsLatched = true;
  emergencyEventPending = true;
  portEXIT_CRITICAL_ISR(&buttonMux);
}

void IRAM_ATTR onStartChanged() {
  portENTER_CRITICAL_ISR(&buttonMux);
  startEdgePending = true;
  portEXIT_CRITICAL_ISR(&buttonMux);
}

bool readStartPressed() {
  return digitalRead(pins::START_SWITCH) == LOW;
}
}

void ButtonInterrupts::begin() {
  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);

  const uint32_t nowMs = millis();
  startRawPressed = readStartPressed();
  startStablePressed = startRawPressed;
  startPressQualified = false;
  startHasSeenReleased = !startRawPressed;
  startRawChangedMs = nowMs;
  startLastStableChangeMs = nowMs;
  pendingStartEvents = 0;
  startEdgePending = false;
  startDebounceActive = false;

  emergencyIsLatched = digitalRead(pins::SAFETY_SWITCH) == LOW;
  emergencyEventPending = emergencyIsLatched;
  if (emergencyIsLatched) {
    Bts7960Driver::latchPermanentDriveInhibit();
    disableMotorEnPins();
  }

  attachInterrupt(digitalPinToInterrupt(pins::SAFETY_SWITCH),
                  onEmergencyPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::START_SWITCH), onStartChanged,
                  CHANGE);
}

void ButtonInterrupts::update(uint32_t nowMs) {
  // Poll the real level too: an edge can happen while boot is attaching ISR.
  if (digitalRead(pins::SAFETY_SWITCH) == LOW && !emergencyLatched()) {
    Bts7960Driver::latchPermanentDriveInhibit();
    disableMotorEnPins();
    portENTER_CRITICAL(&buttonMux);
    emergencyIsLatched = true;
    emergencyEventPending = true;
    portEXIT_CRITICAL(&buttonMux);
  }
  portENTER_CRITICAL(&buttonMux);
  const bool edgePending = startEdgePending;
  startEdgePending = false;
  portEXIT_CRITICAL(&buttonMux);

  if (edgePending || readStartPressed() != startRawPressed) {
    startDebounceActive = true;
    if (edgePending) startRawChangedMs = nowMs;
  }
  if (!startDebounceActive) {
    return;
  }

  const bool rawPressed = readStartPressed();
  if (rawPressed != startRawPressed) {
    startRawPressed = rawPressed;
    startRawChangedMs = nowMs;
  }

  if (startStablePressed == startRawPressed ||
      nowMs - startRawChangedMs < config::switches::DEBOUNCE_MS) {
    if (startStablePressed == startRawPressed &&
        nowMs - startRawChangedMs >= config::switches::DEBOUNCE_MS) {
      startDebounceActive = false;
      if (!startRawPressed) startHasSeenReleased = true;
    }
    return;
  }

  portENTER_CRITICAL(&buttonMux);
  startStablePressed = startRawPressed;
  startLastStableChangeMs = nowMs;
  portEXIT_CRITICAL(&buttonMux);
  startDebounceActive = false;

  if (startRawPressed) {
    // A press is valid only after the switch has previously been released.
    portENTER_CRITICAL(&buttonMux);
    startPressQualified = startHasSeenReleased;
    portEXIT_CRITICAL(&buttonMux);
    return;
  }

  startHasSeenReleased = true;
  portENTER_CRITICAL(&buttonMux);
  const bool qualified = startPressQualified;
  startPressQualified = false;
  portEXIT_CRITICAL(&buttonMux);
  if (!qualified) {
    return;
  }

  portENTER_CRITICAL(&buttonMux);
  if (pendingStartEvents < UINT32_MAX) {
    ++pendingStartEvents;
  }
  portEXIT_CRITICAL(&buttonMux);
}

bool ButtonInterrupts::emergencyLatched() {
  portENTER_CRITICAL(&buttonMux);
  const bool result = emergencyIsLatched;
  portEXIT_CRITICAL(&buttonMux);
  return result;
}

bool ButtonInterrupts::consumeEmergencyEvent() {
  portENTER_CRITICAL(&buttonMux);
  const bool pending = emergencyEventPending;
  emergencyEventPending = false;
  portEXIT_CRITICAL(&buttonMux);
  return pending;
}

bool ButtonInterrupts::consumeStartEvent() {
  portENTER_CRITICAL(&buttonMux);
  const bool pending = pendingStartEvents > 0;
  if (pending) {
    --pendingStartEvents;
  }
  portEXIT_CRITICAL(&buttonMux);
  return pending;
}

bool ButtonInterrupts::startPressed() {
  portENTER_CRITICAL(&buttonMux);
  const bool result = startStablePressed;
  portEXIT_CRITICAL(&buttonMux);
  return result;
}

void ButtonInterrupts::cancelStartGesture() {
  const bool released = !readStartPressed() && !startStablePressed &&
                        !startDebounceActive;
  portENTER_CRITICAL(&buttonMux);
  pendingStartEvents = 0;
  startPressQualified = false;
  // A held or not-yet-debounced press cannot arm a later mode on release.
  startHasSeenReleased = released;
  portEXIT_CRITICAL(&buttonMux);
}

ButtonStatus ButtonInterrupts::status() {
  ButtonStatus result;
  portENTER_CRITICAL(&buttonMux);
  result.emergencyLatched = emergencyIsLatched;
  result.pendingStartEvents = pendingStartEvents;
  result.startPressed = startStablePressed;
  result.startPressQualified = startPressQualified;
  result.lastStartChangeMs = startLastStableChangeMs;
  portEXIT_CRITICAL(&buttonMux);
  return result;
}

[[noreturn]] void ButtonInterrupts::enterEmergencyStopLoop() {
  portENTER_CRITICAL(&buttonMux);
  emergencyIsLatched = true;
  portEXIT_CRITICAL(&buttonMux);
  detachInterrupt(digitalPinToInterrupt(pins::SAFETY_SWITCH));
  detachInterrupt(digitalPinToInterrupt(pins::START_SWITCH));

  disableMotorEnPins();
  ledcWrite(config::motor::MOTOR1_LPWM_CHANNEL, 0);
  ledcWrite(config::motor::MOTOR1_RPWM_CHANNEL, 0);
  ledcWrite(config::motor::MOTOR2_LPWM_CHANNEL, 0);
  ledcWrite(config::motor::MOTOR2_RPWM_CHANNEL, 0);
  MotorController::emergencyStop();
  EventLogger::enterEmergencyMode();
  StatusLed::setEmergencyStop();

  uint32_t lastLogMs = millis() - config::switches::ESTOP_LOG_PERIOD_MS;
  while (true) {
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= config::switches::ESTOP_LOG_PERIOD_MS) {
      EventLogger::writeEmergencyStopNow();
      lastLogMs = nowMs;
    }
    StatusLed::update(nowMs);
    delay(1);
  }
}
