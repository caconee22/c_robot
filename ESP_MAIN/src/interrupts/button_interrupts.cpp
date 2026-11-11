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
constexpr uint8_t SW1 = 0;   // pins::SAFETY_SWITCH
constexpr uint8_t SW2 = 1;   // pins::START_SWITCH
constexpr uint8_t BUTTON_COUNT = 2;

// One debounced button. Both buttons start (stopped) and stop (running).
struct Button {
  volatile bool edgePending = false;
  bool debounceActive = false;
  bool rawPressed = false;
  bool stablePressed = false;
  bool pressQualified = false;    // this press may become a start event
  bool hasSeenReleased = false;   // a press counts only after one release
  uint32_t rawChangedMs = 0;
  uint32_t stableChangedMs = 0;   // press start while pressed
};

Button buttons[BUTTON_COUNT];
volatile uint32_t pendingStartEvents = 0;
bool dualGesture = false;     // both pressed: no start until both released
bool resetArmed = false;      // e-stop + both released once: reset may follow
bool resetRequested = false;

volatile bool emergencyIsLatched = false;
volatile bool emergencyArmed = false;   // running: SW1 hold is an e-stop
volatile bool emergencyEdgePending = false;
bool emergencyHolding = false;
uint32_t emergencyLowSinceMs = 0;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
bool benchStartPressed = false;
#endif

// EN pins can sit in either GPIO register bank (0..31 or 32..48), and the
// bank differs between V1 and V2. Masks are built from pins.hpp at compile
// time so the stop always clears exactly the configured EN pins.
constexpr uint32_t lowBankBit(gpio_num_t pin) {
  return pin < 32 ? (1UL << pin) : 0UL;
}
constexpr uint32_t highBankBit(gpio_num_t pin) {
  return pin >= 32 ? (1UL << (pin - 32)) : 0UL;
}
constexpr uint32_t EN_MASK_LOW_BANK =
    lowBankBit(pins::MOTOR1_LEN) | lowBankBit(pins::MOTOR1_REN) |
    lowBankBit(pins::MOTOR2_LEN) | lowBankBit(pins::MOTOR2_REN);
constexpr uint32_t EN_MASK_HIGH_BANK =
    highBankBit(pins::MOTOR1_LEN) | highBankBit(pins::MOTOR1_REN) |
    highBankBit(pins::MOTOR2_LEN) | highBankBit(pins::MOTOR2_REN);
static_assert(EN_MASK_LOW_BANK != 0 || EN_MASK_HIGH_BANK != 0,
              "the emergency stop must cut at least one motor EN pin");

void IRAM_ATTR disableMotorEnPins() {
  GPIO.out_w1tc = EN_MASK_LOW_BANK;
  GPIO.out1_w1tc.val = EN_MASK_HIGH_BANK;
}

// Any SW1 edge restarts the hold, so bounces from an impact never add up.
void IRAM_ATTR onSafetyChanged() {
  portENTER_CRITICAL_ISR(&buttonMux);
  emergencyEdgePending = true;
  buttons[SW1].edgePending = true;
  portEXIT_CRITICAL_ISR(&buttonMux);
}

void IRAM_ATTR onStartChanged() {
  portENTER_CRITICAL_ISR(&buttonMux);
  buttons[SW2].edgePending = true;
  portEXIT_CRITICAL_ISR(&buttonMux);
}

bool readPressed(uint8_t index) {
  if (index == SW1) return digitalRead(pins::SAFETY_SWITCH) == LOW;
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  if (benchStartPressed) return true;
#endif
  return digitalRead(pins::START_SWITCH) == LOW;
}

void latchEmergency() {
  Bts7960Driver::latchPermanentDriveInhibit();
  disableMotorEnPins();
  portENTER_CRITICAL(&buttonMux);
  emergencyIsLatched = true;
  emergencyArmed = false;
  pendingStartEvents = 0;
  portEXIT_CRITICAL(&buttonMux);
  // The buttons that caused the stop are still down: no reset or start from them.
  resetArmed = false;
  resetRequested = false;
  for (auto& b : buttons) b.pressQualified = false;
}

// Debounce one button. Returns nothing; queues a start event on a qualified release.
void step(uint8_t index, uint32_t nowMs) {
  Button& b = buttons[index];
  portENTER_CRITICAL(&buttonMux);
  const bool edge = b.edgePending;
  b.edgePending = false;
  portEXIT_CRITICAL(&buttonMux);

  if (edge || readPressed(index) != b.rawPressed) {
    b.debounceActive = true;
    if (edge) b.rawChangedMs = nowMs;
  }
  if (!b.debounceActive) return;

  const bool raw = readPressed(index);
  if (raw != b.rawPressed) {
    b.rawPressed = raw;
    b.rawChangedMs = nowMs;
  }
  if (b.stablePressed == b.rawPressed || nowMs - b.rawChangedMs < config::switches::DEBOUNCE_MS) {
    if (b.stablePressed == b.rawPressed && nowMs - b.rawChangedMs >= config::switches::DEBOUNCE_MS) {
      b.debounceActive = false;
      if (!b.rawPressed) b.hasSeenReleased = true;
    }
    return;
  }

  portENTER_CRITICAL(&buttonMux);
  const uint32_t heldMs = nowMs - b.stableChangedMs;  // press length on release
  b.stablePressed = b.rawPressed;
  b.stableChangedMs = nowMs;
  portEXIT_CRITICAL(&buttonMux);
  b.debounceActive = false;

  if (b.rawPressed) {
    // A press is valid only after the button has been released once.
    b.pressQualified = b.hasSeenReleased && !emergencyIsLatched;
    return;
  }
  b.hasSeenReleased = true;
  const bool qualified = b.pressQualified && !dualGesture && !emergencyIsLatched &&
                         heldMs >= config::switches::HOLD_MS;
  b.pressQualified = false;
  if (!qualified) return;
  portENTER_CRITICAL(&buttonMux);
  if (pendingStartEvents < UINT32_MAX) ++pendingStartEvents;
  portEXIT_CRITICAL(&buttonMux);
}

// Both buttons: block starts while both are down, and the e-stop reset.
void stepDual(uint32_t nowMs) {
  const Button& a = buttons[SW1];
  const Button& b = buttons[SW2];
  if (a.stablePressed && b.stablePressed) {
    if (!dualGesture) {
      dualGesture = true;
      buttons[SW1].pressQualified = buttons[SW2].pressQualified = false;
      portENTER_CRITICAL(&buttonMux);
      pendingStartEvents = 0;
      portEXIT_CRITICAL(&buttonMux);
    }
    const uint32_t gap = a.stableChangedMs > b.stableChangedMs ? a.stableChangedMs - b.stableChangedMs
                                                                : b.stableChangedMs - a.stableChangedMs;
    const uint32_t later = a.stableChangedMs > b.stableChangedMs ? a.stableChangedMs : b.stableChangedMs;
    if (emergencyIsLatched && resetArmed && gap <= config::switches::DUAL_PRESS_WINDOW_MS &&
        static_cast<int32_t>(nowMs - later) >= static_cast<int32_t>(config::switches::HOLD_MS)) {
      resetRequested = true;
      resetArmed = false;
    }
    return;
  }
  if (!a.stablePressed && !b.stablePressed && !a.debounceActive && !b.debounceActive) {
    dualGesture = false;
    if (emergencyIsLatched) resetArmed = true;
  }
}
}  // namespace

void ButtonInterrupts::begin() {
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  benchStartPressed = false;
#endif
  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);

  const uint32_t nowMs = millis();
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    Button& b = buttons[i];
    b = Button{};
    b.rawPressed = b.stablePressed = readPressed(i);
    // Down at boot: never a start until released (11/11: SW1 at boot no longer
    // latches; nothing can drive before a start anyway).
    b.hasSeenReleased = !b.rawPressed;
    b.rawChangedMs = b.stableChangedMs = nowMs;
  }
  pendingStartEvents = 0;
  dualGesture = buttons[SW1].stablePressed && buttons[SW2].stablePressed;
  resetArmed = resetRequested = false;
  emergencyIsLatched = false;
  emergencyArmed = false;
  emergencyEdgePending = false;
  emergencyHolding = false;

  attachInterrupt(digitalPinToInterrupt(pins::SAFETY_SWITCH), onSafetyChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::START_SWITCH), onStartChanged, CHANGE);
}

void ButtonInterrupts::setEmergencyArmed(bool armed) {
  portENTER_CRITICAL(&buttonMux);
  emergencyArmed = armed && !emergencyIsLatched;
  if (!emergencyArmed) emergencyHolding = false;
  portEXIT_CRITICAL(&buttonMux);
}

void ButtonInterrupts::pollEmergencyHold(uint32_t nowMs) {
  const bool low = digitalRead(pins::SAFETY_SWITCH) == LOW;
  portENTER_CRITICAL(&buttonMux);
  const bool edge = emergencyEdgePending;
  emergencyEdgePending = false;
  if (!low || !emergencyArmed) {
    emergencyHolding = false;
  } else if (edge || !emergencyHolding) {
    emergencyHolding = true;
    emergencyLowSinceMs = nowMs;
  }
  // Signed: the loop and the watchdog task may pass slightly different times.
  const bool trip = low && emergencyArmed && emergencyHolding && !emergencyIsLatched &&
      static_cast<int32_t>(nowMs - emergencyLowSinceMs) >=
          static_cast<int32_t>(config::switches::HOLD_MS);
  portEXIT_CRITICAL(&buttonMux);
  if (trip) latchEmergency();
}

void ButtonInterrupts::triggerEmergencyStop() { latchEmergency(); }

void ButtonInterrupts::update(uint32_t nowMs) {
  pollEmergencyHold(nowMs);
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) step(i, nowMs);
  stepDual(nowMs);
}

bool ButtonInterrupts::emergencyLatched() {
  portENTER_CRITICAL(&buttonMux);
  const bool result = emergencyIsLatched;
  portEXIT_CRITICAL(&buttonMux);
  return result;
}

bool ButtonInterrupts::consumeStartEvent() {
  portENTER_CRITICAL(&buttonMux);
  const bool pending = pendingStartEvents > 0 && !emergencyIsLatched;
  if (pending) --pendingStartEvents;
  portEXIT_CRITICAL(&buttonMux);
  return pending;
}

bool ButtonInterrupts::startPressed() {
  portENTER_CRITICAL(&buttonMux);
  const bool result = buttons[SW2].stablePressed;
  portEXIT_CRITICAL(&buttonMux);
  return result;
}

uint32_t ButtonInterrupts::startHeldMs(uint32_t nowMs) {
  portENTER_CRITICAL(&buttonMux);
  const int32_t held = buttons[SW2].stablePressed
      ? static_cast<int32_t>(nowMs - buttons[SW2].stableChangedMs) : 0;
  portEXIT_CRITICAL(&buttonMux);
  return held > 0 ? static_cast<uint32_t>(held) : 0;
}

uint32_t ButtonInterrupts::anyHeldMs(uint32_t nowMs) {
  uint32_t longest = 0;
  portENTER_CRITICAL(&buttonMux);
  for (const auto& b : buttons) {
    const int32_t held = b.stablePressed ? static_cast<int32_t>(nowMs - b.stableChangedMs) : 0;
    if (held > 0 && static_cast<uint32_t>(held) > longest) longest = static_cast<uint32_t>(held);
  }
  portEXIT_CRITICAL(&buttonMux);
  return longest;
}

void ButtonInterrupts::cancelStartGesture() {
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    Button& b = buttons[i];
    const bool released = !readPressed(i) && !b.stablePressed && !b.debounceActive;
    b.pressQualified = false;
    // A held or not-yet-debounced press cannot arm a later mode on release.
    b.hasSeenReleased = released;
  }
  portENTER_CRITICAL(&buttonMux);
  pendingStartEvents = 0;
  portEXIT_CRITICAL(&buttonMux);
}

ButtonStatus ButtonInterrupts::status() {
  ButtonStatus result;
  portENTER_CRITICAL(&buttonMux);
  result.emergencyLatched = emergencyIsLatched;
  result.pendingStartEvents = pendingStartEvents;
  result.startPressed = buttons[SW2].stablePressed;
  result.startPressQualified = buttons[SW2].pressQualified;
  result.safetyPressed = buttons[SW1].stablePressed;
  result.lastStartChangeMs = buttons[SW2].stableChangedMs;
  portEXIT_CRITICAL(&buttonMux);
  result.dualGesture = dualGesture;
  result.resetArmed = resetArmed;
  return result;
}

#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
void ButtonInterrupts::benchSetStartPressed(bool pressed) {
  if (benchStartPressed == pressed) return;
  benchStartPressed = pressed;
  portENTER_CRITICAL(&buttonMux);
  buttons[SW2].edgePending = true;
  portEXIT_CRITICAL(&buttonMux);
}

void ButtonInterrupts::benchEmergencyStop() { latchEmergency(); }
#endif

void ButtonInterrupts::waitInEmergencyStop() {
  portENTER_CRITICAL(&buttonMux);
  emergencyIsLatched = true;
  emergencyArmed = false;
  portEXIT_CRITICAL(&buttonMux);
  Bts7960Driver::latchPermanentDriveInhibit();
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
    update(nowMs);   // debounce only: nothing here can drive a motor
    if (resetRequested) break;
    StatusLed::update(nowMs);
    delay(1);
  }

  // Two-button soft reset: clear the stop latches. EN stays LOW and the
  // drive inhibit stays set until the next normal start clears it.
  resetRequested = false;
  for (auto& b : buttons) {
    b.pressQualified = false;
    b.hasSeenReleased = false;   // the reset buttons are still down: release first
  }
  dualGesture = true;
  Bts7960Driver::clearPermanentDriveInhibit();
  portENTER_CRITICAL(&buttonMux);
  emergencyIsLatched = false;
  emergencyHolding = false;
  pendingStartEvents = 0;
  portEXIT_CRITICAL(&buttonMux);
  EventLogger::exitEmergencyMode();
}
