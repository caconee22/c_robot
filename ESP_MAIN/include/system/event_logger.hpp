#pragma once

#include <Arduino.h>

#include "config.hpp"

enum class LogSeverity : uint8_t {
  Debug = 0,
  Info = 1,
  Warning = 2,
  Error = 3,
  Fatal = 4,
};

enum class LogSource : uint8_t {
  System = 0,
  State = 1,
  Safety = 2,
  Motor = 3,
  Sensor = 4,
  Communication = 5,
  User = 6,
};

// Stable event-number ranges:
// 100 system, 200 state/mode, 300 communication, 400 sensor,
// 500 motor, 600 safety, 900 user/custom, 990 logger internal.
enum class EventId : uint16_t {
  BootBegin = 100,
  BootComplete = 101,
  BootFailed = 102,
  ResetReason = 103,
  SystemFaultRaised = 110,
  SystemFaultCleared = 111,

  StateChanged = 200,
  ModeChanged = 201,
  RunStarted = 202,
  RunStopped = 203,
  SystemStateChanged = 204,

  InvalidPacket = 302,
  VisionStatusChanged = 303,

  SensorFaultRaised = 400,
  SensorFaultCleared = 401,
  SensorRangeFault = 402,

  MotorFaultRaised = 500,
  MotorFaultCleared = 501,
  MotorCommandRejected = 502,
  MotorWatchdogStop = 503,

  EmergencyStop = 600,
  SafetyBlocked = 601,

  Custom = 900,
};

enum class FaultCode : uint16_t {
  Unknown = 0,
  Initialization = 1,
  Timeout = 2,
  Range = 3,
  InvalidData = 4,
  Communication = 5,
  Driver = 6,
  Watchdog = 7,
  OverLimit = 8,
  Disconnected = 9,
};

struct EventRecord {
  uint32_t timestampMs = 0;
  uint16_t eventNumber = 0;
  LogSeverity severity = LogSeverity::Info;
  LogSource source = LogSource::System;
  int32_t value1 = 0;
  int32_t value2 = 0;
  char text[config::event_log::TEXT_LENGTH] = {};
};

struct EventLoggerStatus {
  bool initialized = false;
  bool emergencyOnly = false;
  uint16_t queued = 0;
  uint32_t written = 0;
  uint32_t dropped = 0;
  uint32_t suppressed = 0;
};

class EventLogger {
 public:
  static void begin();

  // Convenient predefined event call. Repeated identical events are limited
  // to once per DEFAULT_REPEAT_LIMIT_MS.
  static bool log(
      EventId id, int32_t value1 = 0, int32_t value2 = 0,
      uint32_t minIntervalMs = config::event_log::DEFAULT_REPEAT_LIMIT_MS);

  // Numeric event call for modules that keep their own event-number table.
  static bool log(uint16_t eventNumber, LogSeverity severity, LogSource source,
                  int32_t value1 = 0, int32_t value2 = 0,
                  uint32_t minIntervalMs =
                      config::event_log::DEFAULT_REPEAT_LIMIT_MS);

  // Manual short-text event. Text is copied into the queue; no String or
  // caller-owned buffer is retained. Use minIntervalMs=0 to force every call.
  static bool logText(
      uint16_t eventNumber, LogSeverity severity, LogSource source,
      const char* text, int32_t value1 = 0, int32_t value2 = 0,
      uint32_t minIntervalMs = config::event_log::DEFAULT_REPEAT_LIMIT_MS);

  // These helpers emit only when the supplied value actually changes.
  static bool stateChanged(uint16_t newState);
  static bool modeChanged(uint16_t newMode);
  static bool valueChanged(EventId id, LogSource source, int32_t newValue,
                           int32_t detail = 0);
  static bool faultChanged(LogSource source, FaultCode fault, bool active,
                           int32_t detail = 0);

  // Low-priority queue drain. At most MAX_RECORDS_PER_PROCESS records are
  // written per call.
  static void process();
  static EventLoggerStatus status();

  static const char* severityName(LogSeverity severity);
  static const char* sourceName(LogSource source);

  static void enterEmergencyMode();
  static void writeEmergencyStopNow();
};
