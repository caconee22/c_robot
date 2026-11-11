#pragma once

#include <Arduino.h>

#include "communication/raspberry_link.hpp"
#include "robot_types.hpp"

// Contract
// Role: the old V1 sumo robot (11/10) has a HuskyLens on UART1 instead of the
//   Raspberry Pi. This module asks the HuskyLens for its color blocks and turns
//   the tower block into the same VisionSnapshot the Pi link gives, so the FSM
//   is unchanged. Selected by the build flag VISION_HUSKYLENS=1
//   (communication/vision_link.hpp); the Pi packet format is not touched.
// Protocol (HuskyLens UART, DFRobot): every frame is
//   55 AA 11 len cmd data[len] checksum (checksum = low byte of the sum of all
//   bytes before it). Same flow as the DFRobot HUSKYLENS Arduino example
//   (11/10): begin() = knock 0x2C until 0x2E OK, then each pass request() 0x20
//   -> 0x29 info (block+arrow count, frame number) then one 0x2A block
//   (x center, y center, width, height, id; u16 little endian) or 0x2B arrow
//   per result. The algorithm is the one chosen on the HuskyLens (Color
//   Recognition, tower learned). Done without the library's blocking waits.
// Baud: the example and the HuskyLens default are 9600. Until the first OK the
//   link alternates 9600 / 115200 every BAUD_SWITCH_KNOCKS knocks, then keeps
//   the one that answered.
// Coordinates: HuskyLens is 320x240; they are scaled to the 1920x1080 frame of
//   vision_protocol.hpp (x*6, y*4.5) so the FSM pixel values keep their
//   meaning as a share of the frame. The lens is different, so box-height
//   thresholds (fork 400, bypass 250) must be re-measured on the old robot.
// Called from: VisionLink (begin once, poll every loop pass). Never blocks:
//   a request is written only when the TX buffer has room, at most
//   config::huskylens::MAX_BYTES_PER_POLL bytes are read per pass.
// Failure: a missing or silent HuskyLens only clears linkValid / frameFresh,
//   like a missing Pi (the robot keeps searching). Never a system fault.
namespace config {
namespace huskylens {
constexpr uint32_t BAUD = 9600;          // DFRobot example / HuskyLens default (tried first)
constexpr uint32_t BAUD_OTHER = 115200;  // then these, in turn, until the HuskyLens answers
constexpr uint32_t BAUD_FAST = 1000000;
// Settings HUSKY PROBE tries (3 bauds x normal/swapped RX-TX).
constexpr uint8_t PROBE_STEPS = 6;
constexpr uint16_t SOURCE_WIDTH = 320;
constexpr uint16_t SOURCE_HEIGHT = 240;
// Learned ID of the tower color. 0 = the largest learned block of any ID
// (only learned colors are reported, so with just the tower learned it is it).
constexpr uint16_t TARGET_ID = 0;
// begin(): knock every KNOCK_RETRY_MS until OK; switch baud after this many.
constexpr uint32_t KNOCK_RETRY_MS = 200;
constexpr uint8_t BAUD_SWITCH_KNOCKS = 3;
// No answer for this long after being connected: knock again (HuskyLens reset).
constexpr uint32_t RECONNECT_MS = 1000;
// One request at a time; a new one after the answer, at most this often, or
// after RESPONSE_TIMEOUT_MS with no full answer. At 9600 baud one 16-byte
// frame takes ~17 ms, so info + a few blocks is ~50-100 ms (about 10-15 Hz).
constexpr uint32_t REQUEST_PERIOD_MS = 20;
// Reply timeout, restarted by every received byte (the library waits 100 ms per reply).
constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
// Like the Pi link's freshness rules, longer for the slower 9600 answers.
constexpr uint32_t LINK_TIMEOUT_MS = 300;
constexpr uint32_t FRAME_STALE_MS = 300;
constexpr size_t MAX_BYTES_PER_POLL = 128;
constexpr uint8_t MAX_DATA_LENGTH = 16;
}  // namespace huskylens
}  // namespace config

// One line of the bench wiring probe: UART setting tried and what came back.
struct HuskyProbeResult {
  uint32_t baud;
  bool swapped;      // true = ESP RX on GPIO17, TX on GPIO18 (wires crossed the other way)
  uint32_t bytes;    // raw bytes received
  uint32_t frames;   // frames with a correct checksum
};

class HuskyLensLink {
 public:
  static void begin();
  static void poll();
  static VisionSnapshot latest();
  // Same fields as the Pi link: lastSequence = HuskyLens frame number (low
  // byte), validPackets = full answers, invalidPackets = checksum/format
  // errors and timeouts, duplicatePackets = answers with an unchanged frame.
  static RaspberryLinkStatus status();
  // Bench diagnostic (HUSKY PROBE): tries 115200/9600/1000000 x normal/swapped RX-TX,
  // 0.6 s each, knocking the HuskyLens; keeps the best one until reboot.
  static void startProbe();
  // Bench HUSKY OFF / ON: stop all UART1 traffic to the HuskyLens (pins left
  // as inputs) so it can be set up alone; ON starts the knock again. Until reboot.
  static void setOff(bool off);
  static bool off();
  static bool probing();
  // Results once, after the probe ends (0 while running or already taken).
  static uint8_t takeProbeResults(HuskyProbeResult* out, uint8_t max);
  static uint32_t rawBytesReceived();
  static uint32_t activeBaud();
  static bool activeSwapped();
  static bool connected();           // knock answered at the current setting
  static uint32_t busyCount();       // 0x3D BUSY answers since begin
};
