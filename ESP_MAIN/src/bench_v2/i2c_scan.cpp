#include "bench_v2/i2c_scan.hpp"

#include <Wire.h>
#include <cstdio>
#include <cstring>

namespace {
constexpr uint8_t FIRST = 0x08, LAST = 0x77;  // 7-bit range minus reserved addresses
constexpr uint8_t MUX_FIRST = 0x70, MUX_LAST = 0x77;
constexpr uint8_t PROBES_PER_STEP = 8;
constexpr uint8_t ERRORS_TO_ABORT = 3;
constexpr uint8_t NAK_ADDRESS = 2;  // Wire::endTransmission: no device at the address

enum class Phase : uint8_t { Idle, Mux, Main, Channel, Report };
Phase phase = Phase::Idle;
uint8_t next = 0, channel = 0, mux = 0, errors = 0, errorAddress = 0, reportLine = 0;
uint8_t muxMask = 0;  // bit n = 0x70 + n answered
uint32_t mainFound[4] = {}, channelFound[8][4] = {};
bool failed = false;
uint32_t startedMs = 0, durationMs = 0;

void mark(uint32_t* set, uint8_t a) { set[a >> 5] |= 1UL << (a & 31); }
bool has(const uint32_t* set, uint8_t a) { return (set[a >> 5] >> (a & 31)) & 1UL; }

// Returns true when the device acknowledged. Bus errors count toward abort.
bool probe(uint8_t address) {
  Wire.beginTransmission(address);
  const uint8_t result = Wire.endTransmission();
  if (result == 0 || result == NAK_ADDRESS) {
    errors = 0;
    return result == 0;
  }
  errorAddress = address;
  if (++errors >= ERRORS_TO_ABORT) failed = true;
  return false;
}

// Every found mux off, then optionally one channel on the first mux.
void setMux(int8_t onChannel) {
  for (uint8_t i = 0; i < 8; ++i) {
    if (!(muxMask & (1U << i))) continue;
    Wire.beginTransmission(MUX_FIRST + i);
    Wire.write(static_cast<uint8_t>(onChannel >= 0 && MUX_FIRST + i == mux ? 1U << onChannel : 0U));
    Wire.endTransmission();
  }
}

void finish(uint32_t nowMs) {
  if (muxMask) setMux(-1);  // leave every channel off; the sensor code selects its own
  durationMs = nowMs - startedMs;
  phase = Phase::Report;
  reportLine = 0;
}

int listLine(char* line, size_t size, const char* head, const uint32_t* set, const uint32_t* skip) {
  int length = snprintf(line, size, "%s", head);
  for (uint16_t a = FIRST; a <= LAST && length > 0 && static_cast<size_t>(length) + 6 < size; ++a) {
    if (has(set, a) && !(skip && has(skip, a))) length += snprintf(line + length, size - length, " %02X", a);
  }
  return length;
}
}  // namespace

void BenchI2cScan::start(uint32_t nowMs) {
  phase = Phase::Mux;
  next = MUX_FIRST;
  channel = mux = errors = muxMask = 0;
  failed = false;
  memset(mainFound, 0, sizeof(mainFound));
  memset(channelFound, 0, sizeof(channelFound));
  startedMs = nowMs;
}

bool BenchI2cScan::active() { return phase != Phase::Idle; }

void BenchI2cScan::step(uint32_t nowMs) {
  if (phase == Phase::Idle || phase == Phase::Report) return;
  if (phase == Phase::Mux) {
    // Find muxes first, so the main-bus pass can switch every channel off.
    for (; next <= MUX_LAST && !failed; ++next) {
      if (probe(next)) muxMask |= 1U << (next - MUX_FIRST);
    }
    for (uint8_t i = 0; i < 8 && !mux; ++i) if (muxMask & (1U << i)) mux = MUX_FIRST + i;
    phase = Phase::Main;
    next = FIRST;
  } else {
    setMux(phase == Phase::Channel ? static_cast<int8_t>(channel) : -1);
    uint32_t* found = phase == Phase::Channel ? channelFound[channel] : mainFound;
    for (uint8_t n = 0; n < PROBES_PER_STEP && next <= LAST && !failed; ++n, ++next) {
      if (probe(next)) mark(found, next);
    }
    if (next > LAST && !failed) {
      next = FIRST;
      if (phase == Phase::Main) {
        if (mux) phase = Phase::Channel;
        else finish(nowMs);
      } else if (++channel >= 8) {
        finish(nowMs);
      }
    }
  }
  if (failed && phase != Phase::Report) finish(nowMs);
}

void BenchI2cScan::report() {
  if (phase != Phase::Report) return;
  char line[112];
  int length = 0;
  if (reportLine == 0) {
    length = listLine(line, sizeof(line), "I2C MAIN", mainFound, nullptr);
  } else if (reportLine == 1) {
    length = mux ? snprintf(line, sizeof(line), "I2C MUX %02X", mux) : snprintf(line, sizeof(line), "I2C MUX NONE");
  } else if (reportLine <= 9) {
    if (!mux) { reportLine = 10; return; }
    char head[12];
    snprintf(head, sizeof(head), "I2C CH %u", reportLine - 2);
    // Main-bus devices show on every channel; list only what this channel adds.
    length = listLine(line, sizeof(line), head, channelFound[reportLine - 2], mainFound);
  } else {
    length = failed ? snprintf(line, sizeof(line), "I2C ERR BUS %02X", errorAddress)
                    : snprintf(line, sizeof(line), "I2C DONE %lu", static_cast<unsigned long>(durationMs));
  }
  if (length <= 0 || static_cast<size_t>(length) + 2 >= sizeof(line)) return;
  line[length++] = '\r';
  line[length++] = '\n';
  if (Serial.availableForWrite() < length) return;  // try again next loop, never wait
  Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  if (++reportLine > 10) phase = Phase::Idle;
}
