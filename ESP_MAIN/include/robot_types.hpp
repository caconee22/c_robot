#pragma once

#include <Arduino.h>

#include "sensors/color_sensor_types.hpp"

enum class StopMode : uint8_t {
  Coast,
  Brake,
};

enum class BootState : uint8_t {
  NotStarted,
  Initializing,
  Ready,
  Failed,
};

enum class SafetyState : uint8_t {
  ResetLockout,
  WaitingForStart,
  Running,
  EmergencyStop,
};

enum class RobotState : uint8_t {
  Boot,
  WaitStart,
  Search,
  TargetConfirm,
  TrackAlign,
  Approach,
  ContactCheck,
  DiveReady,
  DiveBrake,
  SpokeEngaged,
  UndercutDrive,
  Push,
  ScoreHold,
  Backoff,
  LostTarget,
  WallEscape,
  ZoneEscape,
  StuckRecovery,
  EmergencyStop,
  Stop,
};

enum class DriveIntentType : uint8_t {
  Brake,
  Coast,
  SearchLeft,
  SearchRight,
  AlignLeft,
  AlignRight,
  Approach,
  AttackCharge,
  SpokeInsert,
  Push,
  DiveBrake,
  Reverse,
  EscapeLeft,
  EscapeRight,
};

struct BootResult {
  BootState state = BootState::NotStarted;
  bool safeToRun = false;
  uint32_t faultFlags = 0;
};

struct SensorSnapshot {
  uint32_t timestampMs = 0;
  ColorRawSample colorRaw[COLOR_SENSOR_COUNT];
  FloorColorResult floorColor[COLOR_SENSOR_COUNT];
  uint8_t colorSensorReadyMask = 0;
  bool colorCalibrationValid = false;
};

struct VisionSnapshot {
  uint32_t receivedMs = 0;
  uint32_t frameUpdatedMs = 0;
  uint8_t sequence = 0;
  uint8_t statusBits = 0;
  uint16_t targetX = 0;
  uint16_t targetY = 0;
  uint16_t targetWidth = 0;
  uint16_t targetHeight = 0;
  bool linkValid = false;
  bool frameFresh = false;
  bool targetValid = false;
  bool trackStable = false;
  bool multipleTargets = false;
  bool boxClipped = false;
  bool cameraOk = false;
  bool pipelineOk = false;
};

struct SafetyStatus {
  SafetyState state = SafetyState::ResetLockout;
  bool motorAllowed = false;
  uint32_t faultFlags = 0;
};

struct DriveIntent {
  DriveIntentType type = DriveIntentType::Brake;
  int16_t steering = 0;
  uint16_t requestedDuty = 0;
};

struct MotorCommand {
  // Signed permille command: -1000 is full reverse, +1000 is full forward.
  int16_t leftPermille = 0;
  int16_t rightPermille = 0;
  StopMode stopMode = StopMode::Brake;
  uint32_t validUntilMs = 0;
  bool deadlineSet = false;
};
