#pragma once
#include "Arduino.h"
#include <array>
namespace fake {
struct SensorDevice {
  bool connected = true, conversionStuck = false, smuxStuck = false;
  bool saturated = false;
  bool highBank = false;
  uint8_t reg[256] = {};
  uint32_t smuxStarted = 0, measurementStarted = 0;
  std::array<uint16_t, 6> low{{100,100,100,100,1000,10}};
  std::array<uint16_t, 6> high{{100,100,100,100,1000,10}};
  std::array<uint16_t, 4> rgbc{{1000,100,100,100}};
};
extern SensorDevice sensors[4];
extern bool tcaConnected;
extern uint32_t busTransactions;
extern uint32_t transactionDelayMs;
void floor(uint8_t sensor, uint8_t color);
}
struct TwoWire {
  uint8_t address = 0, selected = 0, pointer = 0;
  std::vector<uint8_t> outgoing;
  std::deque<uint8_t> incoming;
  void begin(int, int) {}
  void setClock(uint32_t) {}
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t addr) { address=addr; outgoing.clear(); }
  size_t write(uint8_t byte) { outgoing.push_back(byte); return 1; }
  size_t write(const uint8_t* data, size_t size) { outgoing.insert(outgoing.end(), data, data+size); return size; }
  uint8_t endTransmission(bool stop = true);
  size_t requestFrom(uint8_t address, uint8_t size);
  int read() { if (incoming.empty()) return -1; int v=incoming.front(); incoming.pop_front(); return v; }
};
extern TwoWire Wire;
