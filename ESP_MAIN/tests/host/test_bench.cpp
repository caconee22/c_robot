#include <iostream>
#include <stdexcept>
#include "Arduino.h"
#include "Wire.h"
#include "pins.hpp"
#include "bench/bench_controller.hpp"
#include "bench/bench_config.hpp"
#include "motor/motor_controller.hpp"
#include "system/system_manager.hpp"
#include "sensors/sensor_config.hpp"

void setup();
void loop();
namespace {
unsigned checks = 0;
#define CHECK(expression) do { ++checks; if (!(expression)) throw std::runtime_error(#expression); } while (0)
bool stopped(const MotorCommand& command) { return !command.leftPermille && !command.rightPermille; }
VisionSnapshot target(uint32_t now = 0) {
  VisionSnapshot v;
  v.linkValid=v.frameFresh=v.targetValid=v.cameraOk=v.pipelineOk=true;
  v.targetX=960; v.targetHeight=200; v.targetWidth=100; v.frameUpdatedMs=now;
  return v;
}
SensorSnapshot floor(uint32_t now, FloorColor color = FloorColor::Black) {
  SensorSnapshot s; s.colorSensorReadyMask=15;
  for (auto& item:s.floorColor) { item.reliable=true; item.color=color; item.timestampMs=now; }
  return s;
}
void logicTests() {
  SensorSnapshot s; auto v=target(); BenchController::begin();
  CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(static_cast<BenchMode>(255));
  CHECK(BenchController::mode()==BenchMode::Stop);
  CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Data); CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Motor);
  auto c=BenchController::update(100,true,s,v); CHECK(c.leftPermille==120 && c.rightPermille==0);
  CHECK(stopped(BenchController::update(3100,true,s,v)));
  CHECK(BenchController::update(3600,true,s,v).leftPermille==-120);
  CHECK(BenchController::update(7100,true,s,v).rightPermille==120);
  CHECK(BenchController::update(10600,true,s,v).rightPermille==-120);
  CHECK(stopped(BenchController::update(14100,true,s,v))); CHECK(BenchController::finished());
  BenchController::select(BenchMode::Manual);
  CHECK(!BenchController::setManual(100,100,0,false));
  CHECK(BenchController::setManual(-200,200,100,true));
  CHECK(BenchController::update(349,true,s,v).leftPermille==-200);
  CHECK(stopped(BenchController::update(350,true,s,v)));
  CHECK(!BenchController::setManual(201,0,360,true));
  CHECK(stopped(BenchController::update(361,true,s,v)));
  CHECK(BenchController::setManual(100,100,UINT32_MAX-100,true));
  CHECK(!stopped(BenchController::update(50,true,s,v)));
  CHECK(stopped(BenchController::update(149,true,s,v)));
  BenchController::select(BenchMode::Align);
  CHECK(stopped(BenchController::update(0,true,s,v)));
  v.targetX=1300; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille<0);
  v.targetX=600; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille<0 && c.rightPermille>0);
  v.frameFresh=false; CHECK(stopped(BenchController::update(0,true,s,v)));
  v=target(); BenchController::select(BenchMode::Follow);
  c=BenchController::update(0,true,s,v); CHECK(c.leftPermille==160 && c.rightPermille==160);
  v.targetX=1100; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>c.rightPermille);
  v.targetHeight=420; CHECK(stopped(BenchController::update(0,true,s,v)));
  v=target(); v.boxClipped=true; CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Avoid); v=target(); s=floor(0);
  CHECK(!stopped(BenchController::update(0,true,s,v)));
  s.floorColor[0].reliable=false; CHECK(stopped(BenchController::update(0,true,s,v)));
  s=floor(0); s.colorSensorReadyMask=7; CHECK(stopped(BenchController::update(0,true,s,v)));
  s=floor(0); CHECK(stopped(BenchController::update(config::sensors::SAMPLE_STALE_MS+1,true,s,v)));
  s=floor(1000); s.floorColor[0].color=FloorColor::Red;
  c=BenchController::update(1000,true,s,v); CHECK(c.leftPermille<0 && c.rightPermille<0);
  c=BenchController::update(1250,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille<0);
  s=floor(1500); CHECK(!stopped(BenchController::update(1500,true,s,v)));
  BenchController::select(BenchMode::Avoid); s=floor(0); s.floorColor[2].color=FloorColor::Blue;
  c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille>0);
  s.floorColor[0].color=FloorColor::Yellow; CHECK(stopped(BenchController::update(1,true,s,v)));
  CHECK(stopped(BenchController::update(2,false,s,v)));
}
void ticks(uint32_t duration) {
  const uint32_t start=millis();
  while (millis()-start<duration) { fake::advance(1); loop(); }
}
void send(const std::string& line) { Serial.inject(line+"\n"); ticks(5); }
void startButton() {
  fake::level(pins::START_SWITCH,LOW); ticks(50);
  fake::level(pins::START_SWITCH,HIGH); ticks(50);
}
void camera(uint16_t x, uint16_t height, uint8_t sequence = 1) {
  std::vector<uint8_t> packet{0xAA,0x55,1,10,sequence,0x31};
  for (uint16_t value : {x, uint16_t{540}, uint16_t{100}, height}) {
    packet.push_back(value & 255); packet.push_back(value >> 8);
  }
  Serial1.inject(packet); ticks(8);
}
void integrationTests() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  setup(); ticks(30);
  CHECK(MotorController::outputLimit()==200); CHECK(!SystemManager::motorAllowed());
  send("MODE MANUAL"); send("DRIVE 100 100");
  CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  fake::level(pins::START_SWITCH,LOW); ticks(10);
  send("MODE FOLLOW"); ticks(50);
  fake::level(pins::START_SWITCH,HIGH); ticks(50);
  CHECK(!SystemManager::motorAllowed());
  send("MODE MANUAL");
  startButton(); CHECK(SystemManager::motorAllowed());
  send("DRIVE 100 -100"); CHECK(MotorController::status().appliedLeftPermille==100);
  ticks(260); CHECK(!MotorController::status().outputsActive);
  send("DRIVE 150 150"); CHECK(MotorController::status().outputsActive);
  send("MODE ALIGN"); CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  startButton(); ticks(20); CHECK(!MotorController::status().outputsActive);  // No Pi target.
  camera(1300,200); CHECK(MotorController::status().appliedLeftPermille>0);
  CHECK(MotorController::status().appliedRightPermille<0);
  ticks(130); CHECK(!MotorController::status().outputsActive);
  send("MODE FOLLOW"); startButton(); camera(960,200,2);
  CHECK(MotorController::status().appliedLeftPermille==160);
  CHECK(MotorController::status().appliedRightPermille==160);
  camera(960,420,3); CHECK(!MotorController::status().outputsActive);
  send("MODE AVOID"); startButton(); camera(960,200,4);
  CHECK(MotorController::status().appliedLeftPermille==160);
  fake::sensors[0].connected=false; ticks(20); CHECK(!MotorController::status().outputsActive);
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  send("DRIVE 99999999999999999999999999999999999999999999999999999 1");
  CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  Serial.inject("DRIVE 20"); ticks(300); CHECK(!SystemManager::motorAllowed());
  send("0 200"); CHECK(!MotorController::status().outputsActive);  // Discard remainder.
  send("MODE DATA"); ticks(150); CHECK(Serial.tx.find("D,")!=std::string::npos);
  CHECK(Serial.tx.find("S,")!=std::string::npos);
  Serial.capacity=0; const auto before=millis(); loop(); CHECK(millis()-before<=5);
  Serial.capacity=4096;
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  fake::level(pins::SAFETY_SWITCH,LOW); CHECK(fake::pins[pins::MOTOR1_LEN]==LOW);
  fake::stopOnDelay=true;
  try { loop(); CHECK(false); } catch (const fake::StopLoop&) {}
  CHECK(MotorController::status().emergencyStopLatched);
}
}
int main() {
  try {
    logicTests(); integrationTests();
    std::cout<<"BENCH MODEL "<<COLOR_SENSOR_MODEL<<": "<<checks<<" checks passed\n";
  } catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<"\n"; return 1; }
}
