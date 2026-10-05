#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define FALLING 2
#define CHANGE 3
#define SERIAL_8N1 0
using gpio_num_t = int;
enum { GPIO_NUM_11=11, GPIO_NUM_12=12, GPIO_NUM_13=13, GPIO_NUM_14=14,
       GPIO_NUM_15=15, GPIO_NUM_16=16, GPIO_NUM_17=17, GPIO_NUM_18=18,
       GPIO_NUM_21=21, GPIO_NUM_38=38, GPIO_NUM_39=39, GPIO_NUM_40=40,
       GPIO_NUM_41=41, GPIO_NUM_42=42, GPIO_NUM_43=43, GPIO_NUM_44=44,
       GPIO_NUM_47=47, GPIO_NUM_48=48 };
using std::min;
using std::max;
template<class T, class A, class B> T constrain(T value, A low, B high) {
  return value < low ? static_cast<T>(low) : value > high ? static_cast<T>(high) : value;
}
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void pinMode(int pin, int mode);
int digitalRead(int pin);
void digitalWrite(int pin, int level);
inline int digitalPinToInterrupt(int pin) { return pin; }
void attachInterrupt(int pin, void (*handler)(), int mode);
void detachInterrupt(int pin);
double ledcSetup(uint8_t channel, double frequency, uint8_t bits);
void ledcAttachPin(int pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);

struct FakeSerial {
  std::deque<uint8_t> rx;
  std::string tx;
  int capacity = 4096;
  uint32_t baud = 0;
  void begin(uint32_t speed, int = 0, int = 0, int = 0) { baud = speed; }
  int available() const { return static_cast<int>(rx.size()); }
  int availableForWrite() const { return capacity; }
  int read() { if (rx.empty()) return -1; int v = rx.front(); rx.pop_front(); return v; }
  size_t write(const uint8_t* data, size_t size) { tx.append(reinterpret_cast<const char*>(data), size); return size; }
  void println(const char* text = "") { tx += text; tx += "\r\n"; }
  void flush() {}
  int printf(const char* format, ...) {
    char buffer[1024]; va_list args; va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args); va_end(args);
    if (n > 0) tx.append(buffer, std::min(static_cast<size_t>(n), sizeof(buffer)-1));
    return n;
  }
  void inject(const std::string& text) { for (unsigned char c : text) rx.push_back(c); }
  void inject(const std::vector<uint8_t>& data) { for (uint8_t c : data) rx.push_back(c); }
};
extern FakeSerial Serial, Serial1;

namespace fake {
extern uint32_t now;
extern int pins[64];
extern uint32_t pwm[16];
extern bool pwmSetupOk;
extern bool watchdogEnabled;
extern int criticalDepth;
extern std::function<void()> writeHook;
struct StopLoop {};
extern bool stopOnDelay;
void reset();
void advance(uint32_t ms);
void level(int pin, int value, bool interrupt = true);
void enterCritical();
void exitCritical();
}
