#include "communication/run_log.hpp"

namespace {
using namespace config::run_log;

uint8_t queue[QUEUE_SIZE][FRAME_SIZE];
uint8_t head = 0, count = 0;
uint8_t sequence = 0;
uint32_t droppedFrames = 0;
uint32_t lastAliveMs = 0;
bool aliveStarted = false;

void put16(uint8_t* at, uint16_t value) {
  at[0] = static_cast<uint8_t>(value & 0xFF);
  at[1] = static_cast<uint8_t>(value >> 8);
}
}  // namespace

void RunLog::push(RunLogKind kind, uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
  if (count >= QUEUE_SIZE) {
    ++droppedFrames;
    return;
  }
  uint8_t* frame = queue[(head + count) % QUEUE_SIZE];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = VERSION;
  frame[3] = PAYLOAD_LENGTH;
  frame[4] = sequence++;
  frame[5] = static_cast<uint8_t>(kind);
  put16(frame + 6, a);
  put16(frame + 8, b);
  put16(frame + 10, c);
  put16(frame + 12, d);
  ++count;
}

bool RunLog::alive(uint32_t nowMs, bool driving, uint8_t state, uint16_t towerX,
                   uint8_t intentType, uint16_t duty) {
  if (!driving) {
    aliveStarted = false;
    return false;
  }
  if (aliveStarted && nowMs - lastAliveMs < ALIVE_PERIOD_MS) return false;
  aliveStarted = true;
  lastAliveMs = nowMs;
  push(RunLogKind::Alive, state, towerX, intentType, duty);
  return true;
}

void RunLog::process() {
  while (count > 0 && Serial1.availableForWrite() >= FRAME_SIZE) {
    Serial1.write(queue[head], FRAME_SIZE);
    head = (head + 1) % QUEUE_SIZE;
    --count;
  }
}

uint32_t RunLog::dropped() { return droppedFrames; }

void RunLog::reset() {
  head = count = 0;
  sequence = 0;
  droppedFrames = 0;
  aliveStarted = false;
}
