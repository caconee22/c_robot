#pragma once

#include <Arduino.h>

// Contract
// Role: V2 MPU6050 bring-up check. Resets the IMU, confirms WHO_AM_I and
//   reports whether new samples keep arriving (data-ready flag and changing
//   values). Raw counts only; no attitude or fusion. Bench diagnostics and,
//   in the match firmware, the TiltGuard input (since 11/07).
// Called from: setup() (begin, after SensorManager::begin started Wire) and
//   loop() (update). Each update does at most one short I2C transaction
//   (bounded by config::sensors::I2C_TIMEOUT_MS) and never waits.
// Failure: after BUS_ERRORS_TO_FAULT consecutive bus errors, or a WHO_AM_I
//   that is not an MPU6050 family value, the state becomes Fault and polling
//   stops. It is never retried on its own; only requestReset() (an explicit
//   operator command) starts again. Never touches SystemManager or motors.
// Values: register settings are unverified starting values (±8 g, ±1000 dps,
//   DLPF 44 Hz, 100 Hz) and need a check on the real board.

namespace config {
namespace imu {
constexpr uint8_t ADDRESS = 0x68;                // AD0 low
constexpr uint32_t RESET_WAIT_MS = 100;          // datasheet: device reset time
constexpr uint32_t WAKE_WAIT_MS = 10;            // PLL clock settle
constexpr uint32_t POLL_PERIOD_MS = 10;          // 100 Hz, same as SMPLRT_DIV
constexpr uint32_t STALE_MS = 100;               // no data-ready for this long
constexpr uint32_t FROZEN_MS = 500;              // values identical this long
constexpr uint8_t BUS_ERRORS_TO_FAULT = 5;
// Every 50th poll (0.5 s) reads PWR_MGMT_1 instead of data, so a chip that
// reset itself and went back to sleep shows up in the log.
constexpr uint8_t POWER_CHECK_POLLS = 50;
constexpr int16_t ACCEL_LSB_PER_G = 4096;        // ±8 g
constexpr int16_t GYRO_LSB_PER_10DPS = 328;      // ±1000 dps: 32.8 LSB/dps
}  // namespace imu
}  // namespace config

enum class ImuState : uint8_t { Off, Resetting, Waking, Ready, Fault };
enum class ImuError : uint8_t { None, Bus, WhoAmI };
// Why dataFresh is false: NoData = no data-ready for STALE_MS,
// Frozen = samples arrive but the bytes did not change for FROZEN_MS.
enum class ImuStale : uint8_t { Fresh, NotReady, NoData, Frozen };

struct ImuSample {
  int16_t accel[3] = {};   // x, y, z raw counts
  int16_t gyro[3] = {};    // x, y, z raw counts
  int16_t temperature = 0; // raw counts
  uint32_t timestampMs = 0;
};

struct ImuStatus {
  ImuState state = ImuState::Off;
  ImuError lastError = ImuError::None;
  uint8_t whoAmI = 0;
  bool dataFresh = false;      // Ready, data-ready seen recently, values moving
  ImuStale stale = ImuStale::NotReady;  // reason when dataFresh is false
  uint8_t powerMgmt = 0;       // PWR_MGMT_1 read back: 0x01 running, 0x40 asleep (chip reset)
  uint32_t samples = 0;        // data-ready samples read since the last reset
  uint32_t busErrors = 0;      // total failed transactions
  uint32_t resets = 0;
  uint32_t lastSampleMs = 0;
  uint32_t lastChangeMs = 0;
};

namespace ImuMonitor {
void begin(uint32_t nowMs);
void requestReset(uint32_t nowMs);
void update(uint32_t nowMs);
ImuStatus status();
ImuSample latest();
const char* stateName(ImuState state);
}  // namespace ImuMonitor
