#pragma once

#include <Arduino.h>

#include "robot_types.hpp"
#include "system/event_logger.hpp"

enum class SystemState : uint8_t {
  Booting,
  WaitingForStart,
  Running,
  Calibration,  // Reserved: calibration is a separate firmware now.
  SensorFault,
  MotorFault,
  SystemFault,
  EmergencyStop,
};

enum SystemFaultFlag : uint32_t {
  SystemFaultNone = 0,
  SystemFaultResetRearm = 1UL << 0,
  SystemFaultSensor = 1UL << 1,
  SystemFaultMotor = 1UL << 2,
  SystemFaultApplication = 1UL << 3,
  SystemFaultEmergencyStop = 1UL << 4,
};

struct SystemStatus {
  SystemState state = SystemState::Booting;
  SafetyStatus safety;
  bool runRequested = false;
  bool motorAllowed = false;
  bool rearmRequired = false;
  bool sensorsHealthy = true;
  bool motorsHealthy = true;
  bool applicationHealthy = true;
  bool startHeld = false;  // SW2 ignored while a power-on check is pending.
  uint32_t faultFlags = SystemFaultNone;
  uint32_t stateEnteredMs = 0;
  uint32_t transitionCount = 0;
  uint32_t startEventCount = 0;
  uint32_t lastStartEventMs = 0;
};

// Contract
// Role: single owner of "may the motors run" (motorAllowed / SafetyStatus).
//   Combines SW1, SW2 start and aggregate health.
// Called from: loop() twice per pass (before and after sensor/UART work);
//   other modules read status()/safetyStatus() and never decide on their own.
// Failure: unhealthy -> immediate stop, start request dropped, no auto restart.
//   Raspberry Pi state never changes motorAllowed.
// Safety code: yes. Do not merge or drop the repeated update() calls.
class SystemManager {
 public:
  static void begin();
  static void update(uint32_t nowMs);

  // SW2 only starts operation. A software stop must be requested explicitly.
  static void requestStop();

  // Health setters log only an actual change. The detail value is normally a
  // motor number, sensor channel, or module-specific short identifier.
  static void setSensorHealthy(bool healthy,
                               FaultCode fault = FaultCode::Unknown,
                               int32_t detail = 0);
  static void setMotorHealthy(bool healthy,
                              FaultCode fault = FaultCode::Unknown,
                              int32_t detail = 0);
  static void setApplicationHealthy(bool healthy,
                                    FaultCode fault = FaultCode::Unknown,
                                    int32_t detail = 0);

  // While held, SW2 presses are dropped like a fault would drop them; a new
  // press is needed after release. Not a fault: state and LEDs are unchanged.
  static void holdStart(bool held);

  static SystemStatus status();
  static SafetyStatus safetyStatus();
  static SystemState state();
  static bool running();
  static bool motorAllowed();
  static const char* stateName(SystemState state);
};
