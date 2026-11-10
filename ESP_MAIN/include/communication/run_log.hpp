#pragma once

#include <Arduino.h>

// Contract
// Role: short run log from the ESP32 to the Raspberry Pi over the existing
//   UART1 wire (ESP TX GPIO17 -> Pi RX), the opposite direction of the vision
//   packets (11/10). Only on state changes and decisions, plus a 5 Hz alive
//   frame while driving. The Pi (scripts/esp_log_receiver.py) opens a file on
//   Start and closes it on Stop or when the alive frames stop for 1 s (SW1 /
//   SW2 emergency stop enters the permanent ESTOP loop, which sends nothing:
//   the safety path is not changed for logging).
// Frame: the vision packet layout, 14 bytes, little endian, no CRC (same
//   policy as the vision link). Version byte 0x81 marks a log frame; the
//   vision packet V1 (Pi -> ESP32) is unchanged.
//     AA 55 | 81 | 0A | sequence | kind | a | b | c | d   (a..d uint16)
// Called from: AlignFsm/loop push into a small queue (pure); loop() calls
//   process() at low priority, which writes only what fits in the UART1 TX
//   buffer and never waits. A full queue drops the newest frame (counted).
// Failure: the Pi missing or not reading changes nothing on the robot.

namespace config {
namespace run_log {
constexpr uint8_t VERSION = 0x81;
constexpr uint8_t PAYLOAD_LENGTH = 10;
constexpr uint8_t FRAME_SIZE = 14;
constexpr uint8_t QUEUE_SIZE = 16;
constexpr uint32_t ALIVE_PERIOD_MS = 200;
}  // namespace run_log
}  // namespace config

enum class RunLogKind : uint8_t {
  Start = 1,    // a scale permille, b trim L, c trim R, d battery cells
  State = 2,    // a new RobotState, b old RobotState, c tower x, d tower height
  Escape = 3,   // a first HazardSide, b corner mask, c FloorColor, d was charging
  Bypass = 4,   // a turn right, b tower height, c tower x
  Alive = 5,    // a RobotState, b tower x (0 = not seen), c DriveIntentType, d duty
  Stop = 6,     // a SafetyState
  Pose = 7,     // a heading 0.1 deg, b x mm, c y mm (int16 as uint16), d IMU ok
                // (control/pose_estimate.hpp, rough; sent right after each Alive)
};

namespace RunLog {
void push(RunLogKind kind, uint16_t a = 0, uint16_t b = 0, uint16_t c = 0, uint16_t d = 0);
// 5 Hz Alive frame while driving; call every loop pass. True when it pushed one.
bool alive(uint32_t nowMs, bool driving, uint8_t state, uint16_t towerX,
           uint8_t intentType, uint16_t duty);
void process();
uint32_t dropped();
void reset();
}  // namespace RunLog
