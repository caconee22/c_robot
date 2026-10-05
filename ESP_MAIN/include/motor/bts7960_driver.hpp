#pragma once

#include <Arduino.h>

#include "motor/motor_config.hpp"
#include "robot_types.hpp"

struct Bts7960Pins {
  gpio_num_t lpwm;
  gpio_num_t rpwm;
  gpio_num_t len;
  gpio_num_t ren;
  uint8_t lpwmChannel;
  uint8_t rpwmChannel;
};

enum class MotorElectricalState : uint8_t {
  Uninitialized,
  Disabled,
  Coast,
  Brake,
  Forward,
  Reverse,
};

struct Bts7960Status {
  bool initialized = false;
  bool enabled = false;
  int16_t appliedPermille = 0;
  uint32_t pwmDuty = 0;
  MotorElectricalState state = MotorElectricalState::Uninitialized;
};

class Bts7960Driver {
 public:
  explicit Bts7960Driver(const Bts7960Pins& pins);

  // Shared drive inhibit for both BTS7960 units. The permanent ISR latch can
  // only be cleared by initializeDriveInhibit(), which is called on boot.
  static void initializeDriveInhibit();
  static void latchDriveInhibit();
  static void latchPermanentDriveInhibit();
  static void IRAM_ATTR latchPermanentDriveInhibitFromIsr();
  static bool clearDriveInhibit();

  bool begin();
  bool drive(int16_t permille);
  void stop(StopMode mode);
  void coast();
  void brake();
  void disable();

  bool isInitialized() const;
  Bts7960Status status() const;

 private:
  void writePwm(uint32_t lpwmDuty, uint32_t rpwmDuty);
  void setEnabled(bool enabled);

  Bts7960Pins pins_;
  Bts7960Status status_;
};
