#pragma once

#include "motor/bts7960_driver.hpp"
#include "robot_types.hpp"

enum class MotorControlResult : uint8_t {
  Accepted,
  NotInitialized,
  SafetyBlocked,
  EmergencyStopLatched,
  CommandExpired,
  InvalidArgument,
  DriverFailure,
};

struct MotorControllerStatus {
  bool initialized = false;
  bool emergencyStopLatched = true;
  bool outputsActive = false;
  int16_t requestedLeftPermille = 0;
  int16_t requestedRightPermille = 0;
  int16_t appliedLeftPermille = 0;
  int16_t appliedRightPermille = 0;
  int16_t outputLimitPermille = 0;
  StopMode stopMode = StopMode::Brake;
  MotorControlResult lastResult = MotorControlResult::NotInitialized;
  uint32_t lastCommandMs = 0;
  uint32_t commandValidUntilMs = 0;
  uint32_t acceptedCommandCount = 0;
  uint32_t rejectedCommandCount = 0;
  uint32_t watchdogStopCount = 0;
  SafetyState lastSafetyState = SafetyState::ResetLockout;
  Bts7960Status leftDriver;
  Bts7960Status rightDriver;
};

class MotorController {
 public:
  static bool begin();

  // Low-level tank command. Values are -1000..+1000 and are limited again by
  // the current runtime output limit.
  static bool apply(const MotorCommand& command, const SafetyStatus& safety);

  // timeoutMs=0 uses the default; values above INT32_MAX are rejected.
  static bool setTank(int16_t leftPermille, int16_t rightPermille,
                      const SafetyStatus& safety,
                      uint32_t timeoutMs = 0);
  static bool setArcade(int16_t throttlePermille, int16_t steeringPermille,
                        const SafetyStatus& safety,
                        uint32_t timeoutMs = 0);
  static bool forward(int16_t permille, const SafetyStatus& safety,
                      uint32_t timeoutMs = 0);
  static bool reverse(int16_t permille, const SafetyStatus& safety,
                      uint32_t timeoutMs = 0);
  static bool rotateLeft(int16_t permille, const SafetyStatus& safety,
                         uint32_t timeoutMs = 0);
  static bool rotateRight(int16_t permille, const SafetyStatus& safety,
                          uint32_t timeoutMs = 0);

  static void stop(StopMode mode = StopMode::Brake);
  static void coast();
  static void brake();
  static void emergencyStop();
  static bool clearEmergencyStop(const SafetyStatus& safety);

  // Call periodically. It stops the motors if safety changes or a command
  // is not refreshed before its deadline.
  static void update(uint32_t nowMs, const SafetyStatus& safety);
  // Also called by the small watchdog task if loop() is delayed.
  static void checkTimeout(uint32_t nowMs);

  static bool setOutputLimit(int16_t limitPermille);
  static int16_t outputLimit();
  static MotorControllerStatus status();
  static const char* resultName(MotorControlResult result);
};
