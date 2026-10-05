#include "Arduino.h"
#include "Wire.h"
#include "soc/gpio_struct.h"
#include "motor/motor_controller.hpp"
#include "sensors/sensor_config.hpp"

FakeSerial Serial, Serial1;
TwoWire Wire;
FakeGpio GPIO;
namespace {
void (*interrupts[64])() = {};
int interruptModes[64] = {};
std::vector<void (*)()> deferred;
}
namespace fake {
uint32_t now = 0;
int pins[64] = {};
uint32_t pwm[16] = {};
bool pwmSetupOk = true, watchdogEnabled = false, stopOnDelay = false;
int criticalDepth = 0;
std::function<void()> writeHook;
std::vector<uint8_t> nvs;
bool nvsWriteOk = true, nvsReadOk = true;
SensorDevice sensors[4];
bool tcaConnected = true;
uint32_t busTransactions = 0;
uint32_t transactionDelayMs = 0;
void reset() {
  now=0; std::fill(pins,pins+64,HIGH); std::fill(pwm,pwm+16,0);
  std::fill(interrupts,interrupts+64,nullptr); deferred.clear(); criticalDepth=0;
  Serial=FakeSerial{}; Serial1=FakeSerial{}; Wire=TwoWire{};
  writeHook=nullptr; stopOnDelay=false; pwmSetupOk=true;
  nvs.clear(); nvsReadOk=nvsWriteOk=true; tcaConnected=true; busTransactions=0;
  transactionDelayMs=0;
  for (auto& sensor : sensors) sensor=SensorDevice{};
}
void advance(uint32_t ms) {
  for (uint32_t i=0;i<ms;++i) {
    ++now;
    if (watchdogEnabled && now%5==0) MotorController::checkTimeout(now);
  }
}
void level(int pin, int value, bool interrupt) {
  int old=pins[pin]; pins[pin]=value;
  if (!interrupt || old==value || !interrupts[pin]) return;
  if (interruptModes[pin] != CHANGE && value != LOW) return;
  if (criticalDepth) deferred.push_back(interrupts[pin]);
  else interrupts[pin]();
}
void enterCritical() { ++criticalDepth; }
void exitCritical() {
  --criticalDepth;
  if (criticalDepth == 0 && !deferred.empty()) {
    auto pending=deferred; deferred.clear(); for(auto handler:pending) handler();
  }
}
void floor(uint8_t index, uint8_t color) {
  auto& s=sensors[index];
  if (color==3) { s.low={{10,10,10,10,100,10}}; s.high=s.low; s.rgbc={{100,10,10,10}}; }
  else if(color==0) { s.low={{50,80,80,200,1000,10}}; s.high={{200,200,700,700,1000,10}}; s.rgbc={{1000,700,200,80}}; }
  else if(color==1) { s.low={{50,80,80,550,1000,10}}; s.high={{550,650,550,550,1000,10}}; s.rgbc={{1000,550,550,80}}; }
  else { s.low={{50,700,700,100,1000,10}}; s.high={{100,100,100,100,1000,10}}; s.rgbc={{1000,100,100,700}}; }
}
}
uint32_t millis() { return fake::now; }
void delay(uint32_t ms) { if(fake::stopOnDelay) throw fake::StopLoop{}; fake::advance(ms); }
void delayMicroseconds(uint32_t) {}
void pinMode(int, int) {}
int digitalRead(int pin) { return fake::pins[pin]; }
void digitalWrite(int pin, int value) {
  fake::pins[pin]=value;
  if(fake::writeHook) { auto hook=std::move(fake::writeHook); fake::writeHook=nullptr; hook(); }
}
void attachInterrupt(int pin, void (*handler)(), int mode) { interrupts[pin]=handler; interruptModes[pin]=mode; }
void detachInterrupt(int pin) { interrupts[pin]=nullptr; }
double ledcSetup(uint8_t, double frequency, uint8_t) { return fake::pwmSetupOk ? frequency : 0; }
void ledcAttachPin(int, uint8_t) {}
void ledcWrite(uint8_t channel, uint32_t duty) { fake::pwm[channel]=duty; }

uint8_t TwoWire::endTransmission(bool) {
  fake::advance(fake::transactionDelayMs);
  ++fake::busTransactions;
  if(address>=0x70 && address<=0x77) {
    if(!fake::tcaConnected) return 2;
    if(outgoing.size()!=1) return 3;
    selected=0xFF;
    for(uint8_t i=0;i<4;++i) if(outgoing[0]==(1U<<i)) selected=i;
    return 0;
  }
  if(selected>=4 || !fake::sensors[selected].connected || outgoing.empty()) return 2;
  auto& s=fake::sensors[selected];
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  pointer=outgoing[0];
#else
  pointer=outgoing[0]&0x1F;
#endif
  for(size_t i=1;i<outgoing.size();++i) {
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
    if(pointer+i-1==0x93) s.reg[0x93]&=~outgoing[i];  // Write-one-to-clear.
    else
#endif
      s.reg[pointer+i-1]=outgoing[i];
  }
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  if(pointer==0 && outgoing.size()>4) s.highBank=s.reg[3]==0x40;
  const uint8_t enableReg=0x80;
#else
  const uint8_t enableReg=0;
#endif
  if(pointer==enableReg && outgoing.size()==2) {
    if(outgoing[1]&0x10) s.smuxStarted=millis();
    if(outgoing[1]&0x02) { s.measurementStarted=millis(); s.completedCycles=0; }
  }
  return 0;
}
size_t TwoWire::requestFrom(uint8_t, uint8_t size) {
  fake::advance(fake::transactionDelayMs);
  ++fake::busTransactions; incoming.clear();
  if(selected>=4 || !fake::sensors[selected].connected) return 0;
  auto& s=fake::sensors[selected];
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  s.reg[0x92]=0x24;
  s.reg[0x94]=s.saturated ? 0x80 : 0;
  if(pointer>=0x95 && pointer<=0xA0) return 0;  // Require ASTATUS first.
  if(!s.smuxStuck && millis()-s.smuxStarted>=1) s.reg[0x80]&=~0x10;
  s.reg[0xA3]=!s.conversionStuck && (s.reg[0x80]&2) &&
      millis()-s.measurementStarted>=config::sensors::AS7341_INTEGRATION_MS ? 0x40 : 0;
  if(!s.conversionStuck && (s.reg[0x80]&2)) {
    const uint32_t cycles=(millis()-s.measurementStarted)/config::sensors::AS7341_INTEGRATION_MS;
    if(cycles>s.completedCycles) {
      s.completedCycles=cycles;
      if(s.reg[0xF9]&4) s.reg[0x93]|=8;
    }
  }
  const std::array<uint16_t,6> floorWords{{s.low[1],s.low[2],s.high[0],s.high[1],s.high[2],s.high[4]}};
  const auto& words=s.reg[8]==0x60 ? floorWords : (s.highBank ? s.high : s.low);
  for(size_t i=0;i<6;++i) { s.reg[0x95+i*2]=words[i]&0xFF; s.reg[0x96+i*2]=words[i]>>8; }
#else
  s.reg[0x12]=0x44;
  s.reg[0x13]=!s.conversionStuck && (s.reg[0]&2) &&
      millis()-s.measurementStarted>=config::sensors::TCS34725_INTEGRATION_MS ? 1 : 0;
  for(size_t i=0;i<4;++i) { s.reg[0x14+i*2]=s.rgbc[i]&0xFF; s.reg[0x15+i*2]=s.rgbc[i]>>8; }
#endif
  for(uint8_t i=0;i<size;++i) incoming.push_back(s.reg[static_cast<uint8_t>(pointer+i)]);
  return size;
}
