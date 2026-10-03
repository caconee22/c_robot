#include <Arduino.h>

#include "pins.hpp"

namespace {

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint8_t TEST_DUTY = 38;  // About 15 percent of 255.
constexpr uint32_t MOTOR_RUN_MS = 3000;
constexpr uint32_t BETWEEN_MOTORS_MS = 1500;
constexpr uint32_t SWITCH_DEBOUNCE_MS = 30;

constexpr uint8_t MOTOR1_LPWM_CHANNEL = 0;
constexpr uint8_t MOTOR1_RPWM_CHANNEL = 1;
constexpr uint8_t MOTOR2_LPWM_CHANNEL = 2;
constexpr uint8_t MOTOR2_RPWM_CHANNEL = 3;

struct MotorPins {
  gpio_num_t lpwm;
  gpio_num_t rpwm;
  gpio_num_t len;
  gpio_num_t ren;
  uint8_t lpwmChannel;
  uint8_t rpwmChannel;
};

constexpr MotorPins MOTOR1{
    pins::MOTOR1_LPWM,
    pins::MOTOR1_RPWM,
    pins::MOTOR1_LEN,
    pins::MOTOR1_REN,
    MOTOR1_LPWM_CHANNEL,
    MOTOR1_RPWM_CHANNEL,
};

constexpr MotorPins MOTOR2{
    pins::MOTOR2_LPWM,
    pins::MOTOR2_RPWM,
    pins::MOTOR2_LEN,
    pins::MOTOR2_REN,
    MOTOR2_LPWM_CHANNEL,
    MOTOR2_RPWM_CHANNEL,
};

class DebouncedActiveLowSwitch {
 public:
  explicit DebouncedActiveLowSwitch(gpio_num_t pin) : pin_(pin) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    const bool initial = readRaw();
    rawState_ = initial;
    stableState_ = initial;
    lastRawChangeMs_ = millis();
  }

  void update(uint32_t nowMs) {
    const bool current = readRaw();
    if (current != rawState_) {
      rawState_ = current;
      lastRawChangeMs_ = nowMs;
    }

    fell_ = false;
    rose_ = false;
    if (rawState_ != stableState_ &&
        nowMs - lastRawChangeMs_ >= SWITCH_DEBOUNCE_MS) {
      const bool previous = stableState_;
      stableState_ = rawState_;
      fell_ = !previous && stableState_;
      rose_ = previous && !stableState_;
    }
  }

  bool active() const { return stableState_; }
  bool pressed() const { return fell_; }
  bool released() const { return rose_; }

 private:
  bool readRaw() const { return digitalRead(pin_) == LOW; }

  gpio_num_t pin_;
  bool rawState_ = false;
  bool stableState_ = false;
  bool fell_ = false;
  bool rose_ = false;
  uint32_t lastRawChangeMs_ = 0;
};

class SafetySwitch {
 public:
  explicit SafetySwitch(gpio_num_t pin) : pin_(pin) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    active_ = digitalRead(pin_) == LOW;
    highSinceMs_ = millis();
  }

  void update(uint32_t nowMs) {
    if (digitalRead(pin_) == LOW) {
      active_ = true;
      highSinceMs_ = nowMs;
      return;
    }

    if (active_ && nowMs - highSinceMs_ >= SWITCH_DEBOUNCE_MS) {
      active_ = false;
    }
  }

  bool active() const { return active_; }

 private:
  gpio_num_t pin_;
  bool active_ = true;
  uint32_t highSinceMs_ = 0;
};

enum class TestState {
  WaitForStartRelease,
  Ready,
  Motor1Running,
  Pause,
  Motor2Running,
  Completed,
  SafetyStopped,
};

DebouncedActiveLowSwitch startSwitch(pins::START_SWITCH);
SafetySwitch safetySwitch(pins::SAFETY_SWITCH);
TestState testState = TestState::WaitForStartRelease;
uint32_t stateStartedMs = 0;

void configureMotor(const MotorPins& motor) {
  pinMode(motor.len, OUTPUT);
  pinMode(motor.ren, OUTPUT);
  digitalWrite(motor.len, LOW);
  digitalWrite(motor.ren, LOW);

  ledcSetup(motor.lpwmChannel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(motor.rpwmChannel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(motor.lpwm, motor.lpwmChannel);
  ledcAttachPin(motor.rpwm, motor.rpwmChannel);
  ledcWrite(motor.lpwmChannel, 0);
  ledcWrite(motor.rpwmChannel, 0);
}

void coastStop(const MotorPins& motor) {
  ledcWrite(motor.lpwmChannel, 0);
  ledcWrite(motor.rpwmChannel, 0);
  digitalWrite(motor.len, LOW);
  digitalWrite(motor.ren, LOW);
}

void stopAllMotors() {
  coastStop(MOTOR1);
  coastStop(MOTOR2);
}

void runPositive(const MotorPins& motor, uint8_t duty) {
  ledcWrite(motor.lpwmChannel, 0);
  ledcWrite(motor.rpwmChannel, 0);
  digitalWrite(motor.len, HIGH);
  digitalWrite(motor.ren, HIGH);
  ledcWrite(motor.rpwmChannel, duty);
}

void enterState(TestState next, uint32_t nowMs) {
  testState = next;
  stateStartedMs = nowMs;
}

void abortForSafety(uint32_t nowMs) {
  stopAllMotors();
  if (testState != TestState::SafetyStopped) {
    Serial.println("SAFETY ACTIVE: motors disabled");
  }
  enterState(TestState::SafetyStopped, nowMs);
}

}  // namespace

void setup() {
  Serial.begin(DEBUG_BAUD, SERIAL_8N1, pins::UART0_RX, pins::UART0_TX);

  configureMotor(MOTOR1);
  configureMotor(MOTOR2);
  stopAllMotors();

  safetySwitch.begin();
  startSwitch.begin();

  Serial.println();
  Serial.println("ESP32-S3 BTS7960 low-speed motor test");
  Serial.println("Release SW1, release SW2, then press SW2 once.");
  Serial.println("Motor 1 runs for 3 s, then motor 2 runs for 3 s.");
}

void loop() {
  const uint32_t nowMs = millis();
  safetySwitch.update(nowMs);
  startSwitch.update(nowMs);

  if (safetySwitch.active()) {
    abortForSafety(nowMs);
    return;
  }

  if (testState == TestState::SafetyStopped) {
    stopAllMotors();
    if (!startSwitch.active()) {
      Serial.println("Safety released. Ready for a new SW2 press.");
      enterState(TestState::Ready, nowMs);
    }
    return;
  }

  switch (testState) {
    case TestState::WaitForStartRelease:
      stopAllMotors();
      if (!startSwitch.active()) {
        Serial.println("Ready. Press SW2 to start the test.");
        enterState(TestState::Ready, nowMs);
      }
      break;

    case TestState::Ready:
      stopAllMotors();
      if (startSwitch.pressed()) {
        Serial.println("Motor 1: low-speed positive direction for 3 s");
        runPositive(MOTOR1, TEST_DUTY);
        enterState(TestState::Motor1Running, nowMs);
      }
      break;

    case TestState::Motor1Running:
      if (nowMs - stateStartedMs >= MOTOR_RUN_MS) {
        stopAllMotors();
        Serial.println("Motor 1 stopped");
        enterState(TestState::Pause, nowMs);
      }
      break;

    case TestState::Pause:
      if (nowMs - stateStartedMs >= BETWEEN_MOTORS_MS) {
        Serial.println("Motor 2: low-speed positive direction for 3 s");
        runPositive(MOTOR2, TEST_DUTY);
        enterState(TestState::Motor2Running, nowMs);
      }
      break;

    case TestState::Motor2Running:
      if (nowMs - stateStartedMs >= MOTOR_RUN_MS) {
        stopAllMotors();
        Serial.println("Motor 2 stopped. Test complete.");
        enterState(TestState::Completed, nowMs);
      }
      break;

    case TestState::Completed:
      stopAllMotors();
      if (!startSwitch.active()) {
        Serial.println("Release detected. Press SW2 to run again.");
        enterState(TestState::Ready, nowMs);
      }
      break;

    case TestState::SafetyStopped:
      break;
  }
}
