#include <Arduino.h>
#include "pins.hpp"

// Contract: standalone lifted-wheel diagnostic, HW_VERSION=2 only.
// No controller, sensors, switches, arming or command watchdog are linked.
// Both motors run at fixed 20% PWM after boot. Cut motor power to stop.
#if HW_VERSION != 2
#error "Raw motor diagnostic requires V2 wiring"
#endif

namespace {
constexpr uint32_t kFrequencyHz = 20000;
constexpr uint8_t kResolutionBits = 8;
constexpr uint32_t kDuty = 51;  // 20% of 255; no starting boost.
constexpr uint8_t kChannels[] = {0, 1, 2, 3};
constexpr int kPwmPins[] = {pins::MOTOR1_LPWM, pins::MOTOR1_RPWM,
                            pins::MOTOR2_LPWM, pins::MOTOR2_RPWM};
bool pwmReady = false;
}

void setup() {
  pinMode(pins::MOTOR1_EN, OUTPUT);
  pinMode(pins::MOTOR2_EN, OUTPUT);
  digitalWrite(pins::MOTOR1_EN, LOW);
  digitalWrite(pins::MOTOR2_EN, LOW);
  for (int pin : kPwmPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  Serial.begin(57600);
  pwmReady = true;
  for (unsigned i = 0; i < 4; ++i) {
    if (ledcSetup(kChannels[i], kFrequencyHz, kResolutionBits) <= 0) {
      pwmReady = false;
    }
    ledcAttachPin(kPwmPins[i], kChannels[i]);
    ledcWrite(kChannels[i], 0);
  }
  if (!pwmReady) {
    Serial.println("RAW_MOTOR ERROR: PWM setup failed; EN stays LOW");
    return;
  }
  // Same electrical direction as a positive command in the bench firmware.
  ledcWrite(1, kDuty);
  ledcWrite(3, kDuty);
  digitalWrite(pins::MOTOR1_EN, HIGH);
  digitalWrite(pins::MOTOR2_EN, HIGH);
}

void loop() {
  if (pwmReady) {
    Serial.printf("RAW_MOTOR RUN duty=%lu/255 EN=%d,%d PWM=%lu,%lu,%lu,%lu\n",
                  static_cast<unsigned long>(kDuty),
                  digitalRead(pins::MOTOR1_EN), digitalRead(pins::MOTOR2_EN),
                  static_cast<unsigned long>(ledcRead(0)),
                  static_cast<unsigned long>(ledcRead(1)),
                  static_cast<unsigned long>(ledcRead(2)),
                  static_cast<unsigned long>(ledcRead(3)));
  }
  delay(1000);  // Hardware PWM continues independently of this log interval.
}
