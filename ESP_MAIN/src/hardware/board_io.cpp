#include "hardware/board_io.hpp"

#include <Arduino.h>

#include "pins.hpp"

namespace {
void setOutputLow(gpio_num_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
}

void BoardIo::setEarlySafeState() {
  setOutputLow(pins::MOTOR1_LPWM);
  setOutputLow(pins::MOTOR1_RPWM);
  setOutputLow(pins::MOTOR1_LEN);
  setOutputLow(pins::MOTOR1_REN);
  setOutputLow(pins::MOTOR2_LPWM);
  setOutputLow(pins::MOTOR2_RPWM);
  setOutputLow(pins::MOTOR2_LEN);
  setOutputLow(pins::MOTOR2_REN);
}

void BoardIo::configurePins() {
  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);
}
