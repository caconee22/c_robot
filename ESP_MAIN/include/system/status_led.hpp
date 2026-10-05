#pragma once

#include <Arduino.h>

enum class LedState : uint8_t {
  Off,
  Booting,
  WaitingForStart,
  Ready,
  Searching,
  Tracking,
  Aligning,
  Approaching,
  Charging,
  Pushing,
  WallEscape,
  ZoneEscape,
  CommunicationLost,
  SensorFault,
  MotorFault,
  Fault,
  EmergencyStop,
};

enum class LedTarget : uint8_t {
  Internal,
  External,
  Both,
};

// Each owner has one request slot per LED. Priority is compared across owners.
enum class LedOwner : uint8_t {
  System,
  Safety,
  Communication,
  Sensors,
  Motors,
  Fsm,
  User,
  Count,
};

struct LedUnitStatus {
  LedState state = LedState::Off;
  uint8_t priority = 0;
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  bool outputOn = false;
};

struct StatusLedSnapshot {
  bool initialized = false;
  LedUnitStatus internal;
  LedUnitStatus external;
  uint32_t lastUpdateMs = 0;
};

class StatusLed {
 public:
  static void begin();

  // ttlMs == 0 keeps the request active until the same owner changes or
  // clears it. Different owners are resolved by state priority.
  static bool request(LedOwner owner, LedTarget target, LedState state,
                      uint32_t ttlMs = 0);
  static void clear(LedOwner owner, LedTarget target = LedTarget::Both);

  static void setInternal(LedOwner owner, LedState state,
                          uint32_t ttlMs = 0);
  static void setExternal(LedOwner owner, LedState state,
                          uint32_t ttlMs = 0);
  static void setBoth(LedOwner owner, LedState state, uint32_t ttlMs = 0);
  static void setEmergencyStop();

  // Call from a low-priority loop or task. WS2812 writes occur only here.
  static void update(uint32_t nowMs);

  static LedState state(LedTarget target);
  static uint8_t priority(LedState state);
  static const char* stateName(LedState state);
  static StatusLedSnapshot status();
};
