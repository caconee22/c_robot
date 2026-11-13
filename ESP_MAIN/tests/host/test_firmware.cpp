#include "Arduino.h"
#include "Wire.h"
#include "Preferences.h"
#include "app/boot_manager.hpp"
#include "app/boot_mode.hpp"
#include "communication/huskylens_link.hpp"
#include "communication/raspberry_link.hpp"
#include "communication/run_log.hpp"
#include "communication/vision_protocol.hpp"
#include "control/align_fsm.hpp"
#include "control/opening_move.hpp"
#include "control/pose_estimate.hpp"
#include "control/escape_move.hpp"
#include "control/circle_bypass.hpp"
#include "control/fork_attack.hpp"
#include "control/control_config.hpp"
#include "control/drive_logic.hpp"
#include "control/robot_fsm.hpp"
#include "control/wander.hpp"
#include "interrupts/button_interrupts.hpp"
#include "motor/motor_controller.hpp"
#include "motor/motor_trim.hpp"
#include "pins.hpp"
#include "calibration/color_calibration.hpp"
#include "sensors/calibration_store.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/floor_color_classifier.hpp"
#include "sensors/sensor_config.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/battery_monitor.hpp"
#include "system/battery_setting.hpp"
#include "system/event_logger.hpp"
#include "system/reset_report.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"
#include "system/tilt_guard.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

void setup();
void loop();
namespace {
int assertions=0, groups=0;
#define CHECK(condition) do { ++assertions; if (!(condition)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #condition); } while(false)

SafetyStatus running() { SafetyStatus s; s.state=SafetyState::Running; s.motorAllowed=true; return s; }
void fixture() {
  fake::reset(); EventLogger::begin(); MotorController::begin();
  ButtonInterrupts::begin(); StatusLed::begin(); SystemManager::begin();
}
// SW2 held past debounce + the 0.5 s hold, then released.
void pressStart() {
  fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
  fake::advance(config::switches::HOLD_MS); SystemManager::update(millis());
  fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
}
std::vector<uint8_t> packet(uint8_t seq, uint16_t x=960, uint16_t height=200, uint8_t flags=0x33) {
  const uint16_t y=flags&1 ? 500 : 0, width=flags&1 ? 200 : 0;
  if(!(flags&1)) { x=0; height=0; }
  std::vector<uint8_t> p{0xAA,0x55,1,10,seq,flags};
  for(uint16_t value : {x,y,width,height}) { p.push_back(value&255); p.push_back(value>>8); }
  return p;
}
void feed(const std::vector<uint8_t>& bytes) {
  Serial1.inject(bytes); while(Serial1.available()) RaspberryLink::poll();
}
VisionSnapshot target(uint8_t seq, uint32_t frameMs, uint16_t x=960, uint16_t height=200) {  // below ATTACK_BOX_HEIGHT: approach
  VisionSnapshot v; v.sequence=seq; v.frameUpdatedMs=frameMs; v.receivedMs=frameMs;
  v.targetX=x; v.targetHeight=height; v.targetWidth=200; v.targetY=500;
  v.linkValid=v.frameFresh=v.targetValid=v.cameraOk=v.pipelineOk=true;
  return v;
}
void sensorTicks(uint32_t duration, bool manager=false) {
  for(uint32_t i=0;i<duration;++i) {
    fake::advance(1); const uint32_t before=millis();
    if(manager) SensorManager::update(); else ColorSensorManager::update(millis());
    CHECK(millis()==before);
  }
}
void command(const std::string& text) {
  Serial.inject(text+"\n");
  while(Serial.available()) ColorCalibration::update(millis());
}

void testMotor() {
  fixture(); CHECK(MotorController::status().initialized);
  CHECK(!MotorController::forward(300,running()));
  CHECK(MotorController::clearEmergencyStop(running()));
  CHECK(MotorController::setTank(-32768,32767,running()));
  auto s=MotorController::status(); CHECK(s.appliedLeftPermille==-config::motor::DEFAULT_OUTPUT_LIMIT && s.appliedRightPermille==config::motor::DEFAULT_OUTPUT_LIMIT);
  CHECK(!MotorController::setOutputLimit(1001)); CHECK(MotorController::setOutputLimit(500));
  CHECK(!MotorController::status().outputsActive);
  CHECK(MotorController::setArcade(300,80,running()));
  s=MotorController::status(); CHECK(s.appliedLeftPermille==380 && s.appliedRightPermille==220);
  fake::advance(99); CHECK(MotorController::status().outputsActive);
  MotorController::checkTimeout(millis()+1); CHECK(!MotorController::status().outputsActive);
  CHECK(MotorController::status().watchdogStopCount==1);
  MotorController::checkTimeout(millis()+100); CHECK(MotorController::status().watchdogStopCount==1);
  // A deadline of zero at millis rollover is an actual deadline.
  fake::now=UINT32_MAX-99; CHECK(MotorController::forward(200,running(),100));
  fake::advance(100); CHECK(!MotorController::status().outputsActive);
  CHECK(!MotorController::forward(200,running(),UINT32_MAX));
  CHECK(MotorController::status().lastResult==MotorControlResult::InvalidArgument);
  CHECK(!MotorController::status().outputsActive);
  SafetyStatus invalid=running(); invalid.state=SafetyState::WaitingForStart;
  CHECK(!MotorController::forward(200,invalid)); CHECK(!MotorController::clearEmergencyStop(invalid));
}

void testMotorTrim() {
  // Nothing saved: no trim.
  fixture(); CHECK(MotorTrim::left()==1000 && MotorTrim::right()==1000);
  CHECK(MotorController::clearEmergencyStop(running()));
  CHECK(MotorController::setTank(500,500,running()));
  auto s=MotorController::status(); CHECK(s.appliedLeftPermille==500 && s.appliedRightPermille==500);
  // Saved 950/1000 is loaded at begin and scales every command after the limit.
  fake::nvsU16["trimL"]=950; fake::nvsU16["trimR"]=1000;
  EventLogger::begin(); MotorController::begin(); CHECK(MotorTrim::left()==950);
  CHECK(MotorController::clearEmergencyStop(running()));
  CHECK(MotorController::setTank(500,-500,running()));
  s=MotorController::status(); CHECK(s.appliedLeftPermille==475 && s.appliedRightPermille==-500);
  CHECK(MotorController::setTank(1000,1000,running()));   // limit first (710), then trim
  s=MotorController::status();
  CHECK(s.appliedLeftPermille==config::motor::DEFAULT_OUTPUT_LIMIT*950/1000 &&
        s.appliedRightPermille==config::motor::DEFAULT_OUTPUT_LIMIT);
  // Reduce-only range; out-of-range saved values fall back to no trim.
  CHECK(!MotorTrim::set(799,1000) && !MotorTrim::set(1000,1001) && MotorTrim::left()==950);
  CHECK(MotorTrim::set(1000,900) && MotorTrim::save() && fake::nvsU16["trimR"]==900);
  fake::nvsU16["trimL"]=1200; MotorTrim::load(); CHECK(MotorTrim::left()==1000 && MotorTrim::right()==1000);
  fake::nvsWriteOk=false; CHECK(!MotorTrim::save());
}

void testButtonsAndEstop() {
  fixture(); fake::level(pins::START_SWITCH,LOW); ButtonInterrupts::update(millis());
  fake::advance(10); fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(50); ButtonInterrupts::update(millis()); CHECK(!ButtonInterrupts::consumeStartEvent());
  pressStart(); CHECK(SystemManager::running());
  // A short SW2 press (shorter than the hold) neither stops nor starts.
  fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(300); SystemManager::update(millis());
  fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
  CHECK(SystemManager::running() && !ButtonInterrupts::emergencyLatched());
  CHECK(SystemManager::status().startEventCount==1);
  CHECK(MotorController::forward(500,running()));
  // Inject the emergency edge while a motor write is in progress.
  fake::writeHook=[] { fake::level(pins::SAFETY_SWITCH,LOW); };
  MotorController::forward(400,running());
  // An impact-length SW1 press does nothing; a bounce restarts the hold.
  ButtonInterrupts::pollEmergencyHold(millis()); CHECK(!ButtonInterrupts::emergencyLatched());
  fake::advance(300); ButtonInterrupts::pollEmergencyHold(millis());
  fake::level(pins::SAFETY_SWITCH,HIGH); fake::level(pins::SAFETY_SWITCH,LOW);
  fake::advance(300); ButtonInterrupts::pollEmergencyHold(millis());
  CHECK(!ButtonInterrupts::emergencyLatched());
  // An older timestamp from the other caller must not trip it.
  ButtonInterrupts::pollEmergencyHold(millis()-400); CHECK(!ButtonInterrupts::emergencyLatched());
  // After the full hold the poll itself clears every configured EN pin. The
  // register bank masks are built from pins.hpp, so this covers V2 too.
  const gpio_num_t enPins[]={pins::MOTOR1_LEN,pins::MOTOR1_REN,pins::MOTOR2_LEN,pins::MOTOR2_REN};
  for(const auto en:enPins) fake::pins[en]=HIGH;
  fake::advance(config::switches::HOLD_MS-1); ButtonInterrupts::pollEmergencyHold(millis()); CHECK(!ButtonInterrupts::emergencyLatched());
  fake::advance(1); ButtonInterrupts::pollEmergencyHold(millis());
  CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!Bts7960Driver::clearDriveInhibit());
  for(const auto en:enPins) CHECK(fake::pins[en]==LOW);
  fake::stopOnDelay=true;
  try { SystemManager::update(millis()); CHECK(false); } catch(const fake::StopLoop&) {}
  fake::stopOnDelay=false;
  for(int i=0;i<4;++i) CHECK(fake::pwm[i]==0);
  CHECK(EventLogger::status().emergencyOnly);
  CHECK(Serial.tx.find("ESTOP")!=std::string::npos);
  fake::level(pins::SAFETY_SWITCH,HIGH);
  CHECK(!MotorController::clearEmergencyStop(running()));
  CHECK(!MotorController::forward(300,running()));
  // Boot with SW2 held: its first release must not start.
  fixture(); fake::level(pins::START_SWITCH,LOW,false); ButtonInterrupts::begin();
  fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis()); CHECK(!ButtonInterrupts::consumeStartEvent());
  pressStart(); CHECK(SystemManager::running());
  // SW2 held while running is an emergency stop.
  fixture(); pressStart(); CHECK(SystemManager::running());
  fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
  fake::advance(config::switches::HOLD_MS-1); SystemManager::update(millis());
  CHECK(!ButtonInterrupts::emergencyLatched());
  fake::advance(1); fake::stopOnDelay=true;
  try { SystemManager::update(millis()); CHECK(false); } catch(const fake::StopLoop&) {}
  fake::stopOnDelay=false; CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!MotorController::clearEmergencyStop(running()));
  // A start press shorter than the hold is ignored.
  fixture(); fake::level(pins::START_SWITCH,LOW); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis()); fake::advance(400);
  fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis()); CHECK(!ButtonInterrupts::consumeStartEvent());
  // A release bounce inside the debounce window restarts the timer.
  fixture(); fake::level(pins::START_SWITCH,LOW); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis()); fake::advance(config::switches::HOLD_MS);
  fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(35); fake::level(pins::START_SWITCH,LOW); fake::level(pins::START_SWITCH,HIGH);
  ButtonInterrupts::update(millis()); fake::advance(5); ButtonInterrupts::update(millis());
  CHECK(!ButtonInterrupts::consumeStartEvent()); fake::advance(35); ButtonInterrupts::update(millis());
  CHECK(ButtonInterrupts::consumeStartEvent()); CHECK(!ButtonInterrupts::consumeStartEvent());
}

void testSystem() {
  fixture(); SystemManager::update(millis()); CHECK(SystemManager::state()==SystemState::WaitingForStart);
  pressStart(); CHECK(SystemManager::motorAllowed());
  SystemManager::setMotorHealthy(false,FaultCode::Driver);
  CHECK(!SystemManager::motorAllowed()); CHECK(!SystemManager::status().runRequested);
  pressStart(); CHECK(!SystemManager::status().runRequested);
  SystemManager::setMotorHealthy(true); CHECK(!SystemManager::motorAllowed());
  pressStart(); CHECK(SystemManager::motorAllowed());
  CHECK(SystemManager::running()); SystemManager::requestStop(); CHECK(!SystemManager::running());
  // STOP cancels both a stable press and a press still being debounced.
  for (uint32_t held : {10U, 50U}) {
    fixture();
    fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
    fake::advance(held); SystemManager::update(millis());
    SystemManager::requestStop();
    fake::advance(50); SystemManager::update(millis());
    fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
    fake::advance(40); SystemManager::update(millis());
    CHECK(!SystemManager::running());
    pressStart(); CHECK(SystemManager::running());
  }
}

void testProtocol() {
  fixture(); RaspberryLink::begin(); feed(packet(255));
  CHECK(RaspberryLink::latest().targetValid); feed(packet(0)); CHECK(RaspberryLink::status().lastSequence==0);
  feed(packet(0)); CHECK(RaspberryLink::status().duplicatePackets==1);
  feed(packet(0,1000)); CHECK(RaspberryLink::status().invalidPackets==1);
  fake::advance(80); feed(packet(0)); fake::advance(50); feed(packet(0));
  CHECK(RaspberryLink::latest().linkValid && !RaspberryLink::latest().frameFresh);
  fake::advance(121); feed(packet(0,1000)); CHECK(RaspberryLink::latest().targetX==1000);
  feed(packet(1,1920)); CHECK(RaspberryLink::status().invalidPackets==2);
  feed(packet(2,960,300,0xB3)); CHECK(RaspberryLink::status().invalidPackets==3);
  auto partial=packet(3); partial.resize(8); feed(partial); fake::advance(21); RaspberryLink::poll();
  CHECK(RaspberryLink::status().invalidPackets==4);
  auto truncated=packet(4); truncated.erase(truncated.begin()+8); feed(truncated); feed(packet(5));
  CHECK(RaspberryLink::status().lastSequence==5);
  feed(packet(6,0,0,0x30)); CHECK(!RaspberryLink::latest().targetValid);
  fake::now=UINT32_MAX-20; feed(packet(7)); fake::advance(50); CHECK(RaspberryLink::latest().frameFresh);
  fake::advance(80); CHECK(!RaspberryLink::latest().linkValid);
  uint32_t random=12345;
  std::vector<uint8_t> noise;
  for(int i=0;i<50000;++i) { random=random*1664525U+1013904223U; noise.push_back(random>>24); }
  feed(noise); feed(packet(8)); feed(packet(9)); CHECK(RaspberryLink::status().lastSequence==9);
}

void testLockOnAndCharge();
void testFsm() {
  fixture(); RobotFsm::begin(); SensorSnapshot sensors; VisionSnapshot missing;
  CHECK(RobotFsm::update(0,sensors,missing,SafetyStatus{}).type==DriveIntentType::Brake);
  CHECK(RobotFsm::update(0,sensors,missing,running()).type==DriveIntentType::SearchLeft);
  RobotFsm::update(10,sensors,target(1,10),running()); CHECK(RobotFsm::state()==RobotState::TargetConfirm);
  RobotFsm::update(11,sensors,target(1,10),running()); CHECK(RobotFsm::state()==RobotState::TargetConfirm);
  auto drive=RobotFsm::update(30,sensors,target(2,30,1100),running());
  CHECK(RobotFsm::state()==RobotState::Approach);
  auto tank=DriveLogic::makeCommand(30,drive); CHECK(tank.leftPermille>tank.rightPermille);
  drive=RobotFsm::update(40,sensors,target(3,40,960,500),running());
  CHECK(drive.type==DriveIntentType::AttackCharge);
  CHECK(RobotFsm::update(259,sensors,missing,running()).type==DriveIntentType::AttackCharge);
  CHECK(RobotFsm::update(260,sensors,missing,running()).type==DriveIntentType::DiveBrake);
  CHECK(RobotFsm::update(349,sensors,missing,running()).type==DriveIntentType::DiveBrake);
  CHECK(RobotFsm::update(350,sensors,missing,running()).type==DriveIntentType::SpokeInsert);
  RobotFsm::update(699,sensors,target(4,699,960,600),running());
  CHECK(RobotFsm::update(700,sensors,missing,running()).type==DriveIntentType::Push);
  CHECK(RobotFsm::update(1198,sensors,missing,running()).type==DriveIntentType::Push);
  CHECK(RobotFsm::update(1199,sensors,missing,running()).type==DriveIntentType::Reverse);
  RobotFsm::update(1459,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  RobotFsm::update(1460,sensors,missing,SafetyStatus{});
  RobotFsm::update(1461,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  // Hazard preemption at every attack phase.
  for(uint32_t attackTime : {40U,260U,350U,700U}) {
    RobotFsm::begin(); RobotFsm::update(10,sensors,target(1,10,960,500),running());
    RobotFsm::update(30,sensors,target(2,30,960,500),running());
    if(attackTime>=260) RobotFsm::update(250,sensors,target(3,250,960,500),running());
    if(attackTime>=350) RobotFsm::update(340,sensors,target(4,340,960,500),running());
    if(attackTime>=700) RobotFsm::update(690,sensors,target(5,690,960,500),running());
    sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
    CHECK(RobotFsm::update(attackTime,sensors,missing,running()).type==DriveIntentType::Reverse);
    CHECK(RobotFsm::state()==RobotState::ZoneEscape); sensors=SensorSnapshot{};
  }
  for(uint8_t corner=0;corner<4;++corner) {
    RobotFsm::begin(); sensors=SensorSnapshot{};
    sensors.floorColor[corner].color=FloorColor::Blue; sensors.floorColor[corner].reliable=true;
    auto escape=RobotFsm::update(10,sensors,missing,running());
    CHECK(escape.type==(corner<2 ? DriveIntentType::Reverse : DriveIntentType::Approach));
    escape=RobotFsm::update(370,sensors,missing,running());
    CHECK(escape.type==((corner%2)==0 ? DriveIntentType::EscapeRight : DriveIntentType::EscapeLeft));
    RobotFsm::update(690,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::ZoneEscape);
    sensors=SensorSnapshot{}; RobotFsm::update(1370,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  }
  // millis wrap and frame timestamp zero are valid values.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  CHECK(RobotFsm::update(10,sensors,missing,running()).type==DriveIntentType::Reverse);
  sensors.floorColor[0]=FloorColorResult{};
  sensors.floorColor[2].color=FloorColor::Blue; sensors.floorColor[2].reliable=true;
  CHECK(RobotFsm::update(20,sensors,missing,running()).type==DriveIntentType::Brake);
  CHECK(RobotFsm::update(21,sensors,missing,running()).type==DriveIntentType::Approach);
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  CHECK(RobotFsm::update(22,sensors,missing,running()).type==DriveIntentType::Brake);
  // Whole robot over a groove: brake briefly, then drive out opposite to the
  // end that touched the hazard last, flipping if it is still inside.
  for(bool rearFirst : {false,true}) {
    const uint8_t first=rearFirst?2:0, second=rearFirst?0:2;
    const auto out=rearFirst?DriveIntentType::Approach:DriveIntentType::Reverse;
    const auto back=rearFirst?DriveIntentType::Reverse:DriveIntentType::Approach;
    for(int run=0;run<2;++run) {
      RobotFsm::begin(); sensors=SensorSnapshot{};
      sensors.floorColor[first].color=FloorColor::Red; sensors.floorColor[first].reliable=true;
      RobotFsm::update(10,sensors,missing,running());
      sensors.floorColor[second].color=FloorColor::Red; sensors.floorColor[second].reliable=true;
      CHECK(RobotFsm::update(20,sensors,missing,running()).type==DriveIntentType::Brake);
      CHECK(RobotFsm::update(169,sensors,missing,running()).type==DriveIntentType::Brake);
      CHECK(RobotFsm::update(170,sensors,missing,running()).type==out);
      CHECK(RobotFsm::state()==RobotState::ZoneEscape);
      if(run==0) {
        CHECK(RobotFsm::update(1669,sensors,missing,running()).type==out);
        CHECK(RobotFsm::update(1670,sensors,missing,running()).type==back);
      } else {
        // The end that went in last comes out first; keep moving the same way.
        sensors.floorColor[second]=FloorColorResult{};
        CHECK(RobotFsm::update(171,sensors,missing,running()).type==out);
        CHECK(RobotFsm::update(400,sensors,missing,running()).type==out);
        CHECK(RobotFsm::state()==RobotState::ZoneEscape);
      }
    }
  }
  // Both ends at once with no history: back out.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  sensors.floorColor[1].color=FloorColor::Red; sensors.floorColor[1].reliable=true;
  sensors.floorColor[3].color=FloorColor::Blue; sensors.floorColor[3].reliable=true;
  CHECK(RobotFsm::update(10,sensors,missing,running()).type==DriveIntentType::Brake);
  CHECK(RobotFsm::update(160,sensors,missing,running()).type==DriveIntentType::Reverse);
  sensors=SensorSnapshot{}; RobotFsm::begin();
  RobotFsm::update(UINT32_MAX-10,sensors,target(1,UINT32_MAX-10),running());
  RobotFsm::update(0,sensors,target(2,0),running());
  RobotFsm::update(20,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::LostTarget);
  RobotFsm::update(config::control::LOST_TARGET_HOLD_MS-1,sensors,missing,running());
  CHECK(RobotFsm::state()==RobotState::LostTarget);
  RobotFsm::update(config::control::LOST_TARGET_HOLD_MS,sensors,missing,running());
  CHECK(RobotFsm::state()==RobotState::Search);  // 11/09: Pi silent keeps the spin search (Wander off)
  // A sensor that read red and is then blinded by floor contact (invalid sample)
  // still counts as a hazard for HAZARD_CONTACT_HOLD_MS; a never-sampled one does not.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  sensors.colorRaw[0].sequence=5; sensors.colorRaw[0].valid=true;
  CHECK(RobotFsm::update(10,sensors,missing,running()).type==DriveIntentType::Reverse);
  sensors.floorColor[0]=FloorColorResult{}; sensors.colorRaw[0].sequence=6; sensors.colorRaw[0].valid=false;
  sensors.floorColor[2].color=FloorColor::Blue; sensors.floorColor[2].reliable=true;
  CHECK(RobotFsm::update(20,sensors,missing,running()).type==DriveIntentType::Brake);   // both ends = groove
  RobotFsm::begin(); sensors=SensorSnapshot{};
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  sensors.colorRaw[0].sequence=5; sensors.colorRaw[0].valid=true;
  RobotFsm::update(10,sensors,missing,running());
  sensors.floorColor[0]=FloorColorResult{}; sensors.colorRaw[0].valid=false;
  RobotFsm::update(10+config::control::HAZARD_CONTACT_HOLD_MS-1,sensors,missing,running());
  CHECK(RobotFsm::state()==RobotState::ZoneEscape);
  sensors.colorRaw[0].valid=true;   // contact over: a valid non-hazard reading ends the hold at once
  RobotFsm::update(10+config::control::ZONE_ESCAPE_REVERSE_MS+config::control::ZONE_ESCAPE_TURN_MS+1,
                   sensors,missing,running());
  CHECK(RobotFsm::state()==RobotState::Search);
  sensors=SensorSnapshot{};
  // The escape keeps the color that started it, even if another color shows up.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  sensors.floorColor[1].color=FloorColor::Blue; sensors.floorColor[1].reliable=true;
  sensors.colorRaw[1].sequence=5; sensors.colorRaw[1].valid=true;
  RobotFsm::update(10,sensors,missing,running());
  CHECK(RobotFsm::state()==RobotState::ZoneEscape && RobotFsm::escapeColor()==FloorColor::Blue);
  sensors.floorColor[1].color=FloorColor::Red;
  RobotFsm::update(20,sensors,missing,running());
  CHECK(RobotFsm::escapeColor()==FloorColor::Blue);
  sensors=SensorSnapshot{};
  // Lost on the right: spin right, and the search afterwards also starts right.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  VisionSnapshot noTower=target(1,10); noTower.targetValid=false;  // Pi alive, tower gone
  RobotFsm::update(10,sensors,target(1,10,1500),running());
  CHECK(RobotFsm::update(20,sensors,noTower,running()).type==DriveIntentType::SearchRight);
  const uint32_t searchAt=10+config::control::LOST_TARGET_HOLD_MS;
  CHECK(RobotFsm::update(searchAt,sensors,noTower,running()).type==DriveIntentType::SearchRight);
  CHECK(RobotFsm::update(searchAt+5000,sensors,noTower,
                         running()).type==DriveIntentType::SearchRight);  // 11/09: no left/right swing
  // Align spin ramps: slow just outside the window, fastest far off, never below the floor.
  RobotFsm::begin();
  RobotFsm::update(10,sensors,target(1,10,1900),running());
  RobotFsm::update(20,sensors,target(2,20,1900),running());
  auto far=RobotFsm::update(30,sensors,target(3,30,1900),running());
  CHECK(far.type==DriveIntentType::AlignRight && far.requestedDuty==config::control::ALIGN_MAX_DUTY);
  auto near=RobotFsm::update(40,sensors,target(4,40,960+config::control::TARGET_CENTER_DEADBAND+1),running());
  CHECK(near.type==DriveIntentType::AlignRight && near.requestedDuty==config::control::ALIGN_MIN_DUTY);
  CHECK(RobotFsm::update(50,sensors,target(5,50,960+config::control::TARGET_CENTER_DEADBAND),
                         running()).type==DriveIntentType::Approach);
  testLockOnAndCharge();
}

void testLockOnAndCharge() {
  using namespace config::control;
  SensorSnapshot sensors; VisionSnapshot missing;
  // Charge starts at ATTACK_BOX_HEIGHT (lowered 11/09), not one pixel below.
  RobotFsm::begin();
  RobotFsm::update(10,sensors,target(1,10),running()); RobotFsm::update(20,sensors,target(2,20),running());
  CHECK(RobotFsm::update(30,sensors,target(3,30,960,ATTACK_BOX_HEIGHT-1),running()).type==DriveIntentType::Approach);
  CHECK(RobotFsm::update(40,sensors,target(4,40,960,ATTACK_BOX_HEIGHT),running()).type==DriveIntentType::AttackCharge);
  // Push keeps going down to PUSH_MIN_BOX_HEIGHT (unchanged by the lower charge height).
  uint32_t ms=40+ATTACK_CHARGE_MS; RobotFsm::update(ms,sensors,missing,running());
  ms+=DIVE_BRAKE_MS; RobotFsm::update(ms,sensors,missing,running());
  ms+=SPOKE_INSERT_MS;
  CHECK(RobotFsm::update(ms,sensors,target(5,ms,960,PUSH_MIN_BOX_HEIGHT),running()).type==DriveIntentType::Push);
  CHECK(RobotFsm::state()==RobotState::Push);
  CHECK(RobotFsm::update(ms+10,sensors,target(6,ms+10,960,PUSH_MIN_BOX_HEIGHT-1),running()).type==DriveIntentType::Brake);
  CHECK(RobotFsm::state()==RobotState::TargetConfirm);

  // Lock-on: pushing the opponent, front hits red -> straight back (no turn),
  // target kept, then approach again without a new confirmation.
  RobotFsm::begin();
  RobotFsm::update(10,sensors,target(1,10),running()); RobotFsm::update(20,sensors,target(2,20),running());
  CHECK(RobotFsm::state()==RobotState::Approach);
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  CHECK(RobotFsm::update(30,sensors,target(3,30),running()).type==DriveIntentType::Reverse);
  CHECK(RobotFsm::state()==RobotState::ZoneEscape);
  CHECK(RobotFsm::update(200,sensors,target(4,200),running()).type==DriveIntentType::Reverse);  // still on red
  sensors=SensorSnapshot{};
  auto d=RobotFsm::update(200+LOCKON_BACKOFF_MS-1,sensors,target(5,200+LOCKON_BACKOFF_MS-1),running());
  CHECK(d.type==DriveIntentType::Reverse && d.requestedDuty==LOCKON_BACKOFF_DUTY);
  d=RobotFsm::update(200+LOCKON_BACKOFF_MS,sensors,target(6,200+LOCKON_BACKOFF_MS),running());
  CHECK(d.type==DriveIntentType::Approach && RobotFsm::state()==RobotState::Approach);   // pushes again
  // Lost during the back-off: spin toward the last side, no turn-away escape.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  RobotFsm::update(10,sensors,target(1,10,1100),running()); RobotFsm::update(20,sensors,target(2,20,1100),running());
  sensors.floorColor[1].color=FloorColor::Blue; sensors.floorColor[1].reliable=true;
  CHECK(RobotFsm::update(30,sensors,missing,running()).type==DriveIntentType::Reverse);
  sensors=SensorSnapshot{};
  CHECK(RobotFsm::update(30+LOCKON_BACKOFF_MS,sensors,missing,running()).type==DriveIntentType::SearchRight);
  CHECK(RobotFsm::state()==RobotState::LostTarget);
  // Rear hits a hazard while backing off: brake, then the normal escape (forward).
  RobotFsm::begin(); sensors=SensorSnapshot{};
  RobotFsm::update(10,sensors,target(1,10),running()); RobotFsm::update(20,sensors,target(2,20),running());
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  RobotFsm::update(30,sensors,target(3,30),running());
  sensors=SensorSnapshot{}; sensors.floorColor[2].color=FloorColor::Blue; sensors.floorColor[2].reliable=true;
  CHECK(RobotFsm::update(100,sensors,target(4,100),running()).type==DriveIntentType::Brake);
  CHECK(RobotFsm::update(110,sensors,target(5,110),running()).type==DriveIntentType::Approach);
  CHECK(RobotFsm::update(110+ZONE_ESCAPE_REVERSE_MS,sensors,target(6,110+ZONE_ESCAPE_REVERSE_MS),
                         running()).type==DriveIntentType::EscapeRight);   // normal escape turns away
  // Not tracking (tower not seen for LOCKON_RECENT_MS): normal escape with a turn.
  RobotFsm::begin(); sensors=SensorSnapshot{};
  RobotFsm::update(10,sensors,target(1,10),running()); RobotFsm::update(20,sensors,target(2,20),running());
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  CHECK(RobotFsm::update(20+LOCKON_RECENT_MS,sensors,missing,running()).type==DriveIntentType::Reverse);
  CHECK(RobotFsm::update(20+LOCKON_RECENT_MS+ZONE_ESCAPE_REVERSE_MS,sensors,missing,
                         running()).type==DriveIntentType::EscapeRight);
}

void testColorsAndErrors() {
  fixture(); CHECK(SensorManager::begin());
  for(uint8_t i=0;i<4;++i) fake::floor(i,i);
  sensorTicks(300,true);
  auto snapshot=SensorManager::snapshot();
  for(uint8_t i=0;i<4;++i) {
    CHECK(snapshot.floorColor[i].reliable); CHECK(static_cast<uint8_t>(snapshot.floorColor[i].color)==i);
    CHECK(snapshot.floorColor[i].consecutiveMatches>=3);
  }
  // Invalid sample clears the old reliable color immediately; re-init only after
  // SENSOR_RETRY_PERIOD_MS, at most SENSOR_RETRY_LIMIT times.
  fake::sensors[0].connected=false; sensorTicks(20,true);
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::I2c);
  const uint32_t failures=ColorSensorManager::status().failedReads;
  fake::sensors[0].connected=true; sensorTicks(200,true);
  CHECK(!ColorSensorManager::available(0)); CHECK(ColorSensorManager::status().failedReads==failures);
  CHECK(SensorManager::snapshot().floorColor[1].reliable);
  sensorTicks(config::sensors::SENSOR_RETRY_PERIOD_MS,true);
  CHECK(ColorSensorManager::available(0)); CHECK(ColorSensorManager::status().error[0]==ColorSensorError::None);
  sensorTicks(300,true); CHECK(SensorManager::snapshot().floorColor[0].reliable);
  // Still broken: each retry fails, and after the limit it stays off for good.
  fake::sensors[0].connected=false; sensorTicks(20,true); CHECK(!ColorSensorManager::available(0));
  sensorTicks(config::sensors::SENSOR_RETRY_PERIOD_MS*(config::sensors::SENSOR_RETRY_LIMIT+1),true);
  CHECK(!ColorSensorManager::available(0));
  fake::sensors[0].connected=true; sensorTicks(config::sensors::SENSOR_RETRY_PERIOD_MS*3,true);
  CHECK(!ColorSensorManager::available(0));   // 1 used above + 2 failed = limit reached
  // Missing at boot: recovers once plugged in (no delay() in the loop path).
  fixture(); fake::sensors[2].connected=false; SensorManager::begin();
  CHECK(!ColorSensorManager::available(2)); fake::sensors[2].connected=true;
  sensorTicks(config::sensors::SENSOR_RETRY_PERIOD_MS+20,true); CHECK(ColorSensorManager::available(2));
  fixture(); fake::sensors[1].connected=false; SensorManager::begin();
  CHECK(ColorSensorManager::status().error[1]==ColorSensorError::Initialization);
  CHECK(SystemManager::status().sensorsHealthy);
  fixture(); ColorSensorManager::begin(); fake::sensors[0].conversionStuck=true; sensorTicks(300);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Timeout);
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  fixture(); ColorSensorManager::begin(); fake::sensors[0].smuxStuck=true; sensorTicks(100);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Timeout);
  fixture(); SensorManager::begin(); fake::sensors[0].saturated=true;
  sensorTicks(200,true); CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Range);
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
#endif
  fixture(); SensorManager::begin();
  fake::sensors[0].low.fill(65535); fake::sensors[0].high.fill(65535); fake::sensors[0].rgbc.fill(65535);
  sensorTicks(200,true); CHECK(!SensorManager::snapshot().floorColor[0].reliable);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Range);
  CHECK(ColorSensorManager::available(0));
  auto raw=ColorSensorManager::sample(1);
  raw.normalized[ColorSensorManager::model()==ColorSensorModel::AS7341 ? 1 : 0]=
      std::numeric_limits<float>::quiet_NaN();
  CHECK(!FloorColorClassifier::classify(1,raw,CalibrationStore::data()).reliable);
}

void testThreeSamples() {
  fixture(); SensorManager::begin(); fake::floor(0,0);
  uint32_t oldSequence=0; unsigned newSamples=0;
  for(int i=0;i<400 && newSamples<3;++i) {
    fake::advance(1); SensorManager::update(); auto snapshot=SensorManager::snapshot();
    if(snapshot.colorRaw[0].sequence!=oldSequence) {
      oldSequence=snapshot.colorRaw[0].sequence; ++newSamples;
      CHECK(snapshot.floorColor[0].reliable==(newSamples>=3));
    }
  }
  CHECK(newSamples==3);
  fake::floor(0,2); const uint32_t previousSequence=oldSequence;
  while(ColorSensorManager::sample(0).sequence==previousSequence) { fake::advance(1); SensorManager::update(); }
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
}

void testContinuousFloorMode() {
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  fixture(); CHECK(SensorManager::begin());
  fake::floor(0,0); sensorTicks(40);
  const auto before=ColorSensorManager::sample(0);
  CHECK(before.valid); CHECK(before.featureMask==0x76);
  CHECK(before.channel[1]==80 && before.channel[2]==80);
  CHECK(before.channel[4]==200 && before.channel[5]==200 && before.channel[6]==700);
  CHECK(before.channel[8]==1000 && before.channel[0]==0 && before.channel[9]==0);
  CHECK(fake::sensors[0].reg[0x80]==3);
  CHECK(fake::sensors[0].reg[0xBD]==0 && fake::sensors[0].reg[0xF9]==4);
  CHECK(fake::sensors[0].reg[0xD6]==255);
  const uint8_t route[20]={0x20,0,0,0,4,1,0x30,5,0x60,0x30,5,0,0x10,0,0x40,0x20,0,0x60,0,0};
  for(unsigned i=0;i<20;++i) CHECK(fake::sensors[0].reg[i]==route[i]);
  const uint32_t start=fake::sensors[0].measurementStarted;
  for(int i=0;i<100;++i) SensorManager::update();
  CHECK(ColorSensorManager::sample(0).sequence==before.sequence);
  // AVALID stays true, but no new completion event means no new sample.
  fake::sensors[0].conversionStuck=true;
  sensorTicks(25);
  CHECK(ColorSensorManager::sample(0).sequence==before.sequence);
  fake::sensors[0].conversionStuck=false; sensorTicks(15);
  CHECK(ColorSensorManager::sample(0).sequence>before.sequence);
  CHECK(fake::sensors[0].measurementStarted==start);  // No stop/restart per read.
  fake::sensors[0].saturated=true; sensorTicks(10);
  CHECK(!ColorSensorManager::sample(0).valid);
  fake::sensors[0].saturated=false; sensorTicks(10);
  CHECK(ColorSensorManager::sample(0).valid);
  auto raw=ColorSensorManager::sample(0);
  raw.brightness=0; raw.valid=false;
  CHECK(!FloorColorClassifier::classify(0,raw,CalibrationStore::data()).reliable);
#endif
}

void testCalibration() {
  fixture(); SensorManager::begin(); ColorCalibration::begin();
  command(std::string(80,'X')+"CAL START"); CHECK(!ColorCalibration::active());
  command("CAL START"); CHECK(ColorCalibration::active());
  command("READY"); CHECK(ColorCalibration::status().state==CalibrationState::Countdown);
  command("ABORT"); CHECK(!ColorCalibration::active());
  command("CAL START"); command("CAL MARGIN 20 40");
  command("CAL MARGIN 10 30 JUNK");
  command("CAL MARGIN 4294967316 4294967336");
  for(uint8_t step=0;step<ColorCalibration::STEP_COUNT;++step) {
    auto status=ColorCalibration::status();
    // Black/red/blue: level and lifted all four, pressed front then rear.
    // Yellow (11/11): every pose front pair then rear pair.
    if(status.color==FloorColor::Yellow) CHECK(step>=12 && status.sensorMask==(step%2==0 ? 0x03 : 0x0C));
    else CHECK(status.sensorMask==(status.pose!=CalibrationPose::Pressed ? 0x0F : step%2==0 ? 0x03 : 0x0C));
    // Model a different brightness for level/lifted/pressed references.
    const unsigned scale=status.pose==CalibrationPose::Level ? 2 :
                          status.pose==CalibrationPose::Lifted ? 1 : 3;
    for(uint8_t corner=0;corner<4;++corner) {
      if(!(status.sensorMask&(1U<<corner))) continue;
      fake::floor(corner,static_cast<uint8_t>(status.color));
      auto& device=fake::sensors[corner];
      for(auto& value:device.low) value=value*scale/4;
      for(auto& value:device.high) value=value*scale/4;
      for(auto& value:device.rgbc) value=value*scale/4;
    }
    command("READY");
    // Countdown then independent samples.
    for(int i=0;i<6100;++i) {
      fake::advance(1); SensorManager::update(); ColorCalibration::update(millis());
    }
    CHECK(ColorCalibration::status().state==CalibrationState::WaitingForAccept);
    CHECK(ColorCalibration::status().sampleCount>=config::sensors::CALIBRATION_MIN_SAMPLES);
    if(step==0) {
      command("RETRY"); CHECK(ColorCalibration::status().state==CalibrationState::WaitingForReady);
      command("READY");
      for(int i=0;i<6100;++i) { fake::advance(1); SensorManager::update(); ColorCalibration::update(millis()); }
    }
    command("ACCEPT");
    if(step==0) {
      command("BACK"); CHECK(ColorCalibration::status().step==0);
      command("READY");
      for(int i=0;i<6100;++i) { fake::advance(1); SensorManager::update(); ColorCalibration::update(millis()); }
      command("ACCEPT"); CHECK(ColorCalibration::status().step==1);
    }
  }
  CHECK(ColorCalibration::status().state==CalibrationState::Completed);
  command("STATUS"); CHECK(Serial.tx.find("step=18/18")!=std::string::npos);
  fake::nvsWriteOk=false; command("CAL SAVE"); CHECK(ColorCalibration::status().state==CalibrationState::Error);
  fake::nvsWriteOk=true; command("CAL SAVE"); CHECK(CalibrationStore::valid());
  CHECK(!ColorCalibration::active()); CHECK(!fake::nvs.empty());
  CHECK(CalibrationStore::data().ratioMarginPermille==200);
  CHECK(CalibrationStore::data().brightnessMarginPermille==400);
  for(uint8_t corner=0;corner<4;++corner) {
    for(uint8_t color=0;color<4;++color) {
      fake::floor(corner,color); sensorTicks(200);
      const auto raw=ColorSensorManager::sample(corner);
      const auto result=FloorColorClassifier::classify(corner,raw,CalibrationStore::data());
      CHECK(result.reliable); CHECK(static_cast<uint8_t>(result.color)==color);
    }
  }
  const auto saved=fake::nvs;
  CalibrationStore::load(); ColorCalibration::begin(); CHECK(CalibrationStore::valid());
  fake::nvsWriteOk=false; command("CAL ERASE"); CHECK(CalibrationStore::valid());
  fake::nvsWriteOk=true; command("CAL ERASE"); CHECK(!CalibrationStore::valid()); CHECK(fake::nvs.empty());
  fake::nvs=saved; auto corrupt=ColorCalibrationData{}; std::memcpy(&corrupt,fake::nvs.data(),sizeof(corrupt));
  corrupt.settingsSignature^=1; std::memcpy(fake::nvs.data(),&corrupt,sizeof(corrupt));
  CalibrationStore::load(); CHECK(!CalibrationStore::valid());
  fake::nvs=saved; std::memcpy(&corrupt,fake::nvs.data(),sizeof(corrupt));
  corrupt.reference[0][0][0].brightnessMean=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(fake::nvs.data(),&corrupt,sizeof(corrupt)); CalibrationStore::load(); CHECK(!CalibrationStore::valid());
  fake::nvs=saved; fake::nvs.pop_back(); CalibrationStore::load(); CHECK(!CalibrationStore::valid());
  fake::nvs=saved; fake::nvsReadOk=false; CalibrationStore::load(); CHECK(!CalibrationStore::valid());
}

void testLogsAndLeds() {
  fixture(); EventLogger::begin();
  CHECK(EventLogger::logText(900,LogSeverity::Error,LogSource::User,
                            "123456789012345678901234567890123456789"));
  CHECK(EventLogger::log(EventId::InvalidPacket,1)); CHECK(!EventLogger::log(EventId::InvalidPacket,1));
  EventLogger::stateChanged(2); CHECK(!EventLogger::stateChanged(2));
  auto before=EventLogger::status(); Serial.capacity=32; EventLogger::process();
  CHECK(EventLogger::status().queued==before.queued);
  Serial.capacity=4096; EventLogger::process(); CHECK(EventLogger::status().written==2);
  // While the battery check holds SW2, the LED says "preparing", not "press start".
  SystemManager::holdStart(true); SystemManager::update(millis());
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::Booting);
  SystemManager::holdStart(false);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::WaitingForStart);
  StatusLed::begin(); StatusLed::setBoth(LedOwner::Fsm,LedState::Searching);
  StatusLed::setBoth(LedOwner::Sensors,LedState::Fault,40);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::Fault);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::Searching);
  StatusLed::setEmergencyStop(); fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::EmergencyStop);
  fixture(); fake::now=UINT32_MAX-39; StatusLed::begin();
  StatusLed::setBoth(LedOwner::Fsm,LedState::Searching);
  StatusLed::setBoth(LedOwner::Sensors,LedState::Fault,40);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::Fault);
  fake::advance(20); StatusLed::update(millis()); CHECK(millis()==0);
  CHECK(StatusLed::state(LedTarget::Both)==LedState::Searching);
}

void testRealisticTimingAndBoot() {
  fixture(); fake::transactionDelayMs=1; SensorManager::begin();
  for(uint8_t i=0;i<4;++i) fake::floor(i,i);
  for(int i=0;i<800;++i) { fake::advance(1); SensorManager::update(); }
  for(uint8_t i=0;i<4;++i) CHECK(SensorManager::snapshot().floorColor[i].reliable);
  fake::reset(); for(auto& sensor:fake::sensors) sensor.connected=false;
  const auto boot=BootManager::begin(); CHECK(boot.safeToRun);
  CHECK(SensorManager::snapshot().colorSensorReadyMask==0);
  pressStart(); CHECK(SystemManager::motorAllowed());
  RobotFsm::begin();
  CHECK(RobotFsm::update(millis(),SensorManager::snapshot(),VisionSnapshot{},
                        SystemManager::safetyStatus()).type==DriveIntentType::SearchLeft);
  fake::reset(); fake::pwmSetupOk=false;
  CHECK(!BootManager::begin().safeToRun); CHECK(!SystemManager::motorAllowed());
  // 11/11: SW1 held at boot is no longer an emergency stop, and its first
  // release (it was already down) must not start the robot.
  fake::reset(); fake::level(pins::SAFETY_SWITCH,LOW,false);
  CHECK(BootManager::begin().safeToRun); CHECK(!ButtonInterrupts::emergencyLatched());
  fake::advance(config::switches::HOLD_MS+100); SystemManager::update(millis());
  fake::level(pins::SAFETY_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
  CHECK(!SystemManager::running() && !SystemManager::motorAllowed());
}

// 11/11 (학현): both buttons start when stopped, both stop when running, and
// both held together leave the emergency stop (soft reset, no re-init).
// 11/11 combined firmware: both buttons held 1 s at power-on -> bench.
void testBootMode() {
  fake::reset();
  uint32_t t0=millis(); CHECK(!BootMode::benchRequested());           // nothing held: match at once
  CHECK(millis()-t0<=config::boot_mode::POLL_MS*2);
  fake::reset(); fake::level(pins::SAFETY_SWITCH,LOW,false);          // one button: match
  CHECK(!BootMode::benchRequested());
  fake::reset(); fake::level(pins::SAFETY_SWITCH,LOW,false); fake::level(pins::START_SWITCH,LOW,false);
  t0=millis(); CHECK(BootMode::benchRequested());                    // both held: bench after the hold
  CHECK(millis()-t0>=config::boot_mode::BENCH_HOLD_MS);
  // Both down but one let go before the hold: match.
  fake::reset(); fake::level(pins::SAFETY_SWITCH,LOW,false); fake::level(pins::START_SWITCH,LOW,false);
  const uint32_t start=millis();
  fake::delayHook=[start]{ if(millis()-start>=500) fake::level(pins::START_SWITCH,HIGH,false); };
  CHECK(!BootMode::benchRequested()); fake::delayHook=nullptr;
  // Motor pins are put in the safe state first.
  CHECK(fake::pins[pins::MOTOR1_LEN]==LOW && fake::pins[pins::MOTOR2_LEN]==LOW);
}

int softResetCalls = 0;
void hold(int pin, uint32_t ms) {
  fake::level(pin,LOW); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
  fake::advance(ms); SystemManager::update(millis());
  fake::level(pin,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
}
// Runs SystemManager::update into the e-stop wait; script(t) sets the buttons
// at each 1 ms step (t = ms since the wait began). Gives up after limitMs.
bool runEstopWait(std::function<void(uint32_t)> script, uint32_t limitMs) {
  const uint32_t start=millis();
  fake::delayHook=[&]{ const uint32_t t=millis()-start; if(t>limitMs) throw fake::StopLoop{}; script(t); };
  bool returned=true;
  try { SystemManager::update(millis()); } catch(const fake::StopLoop&) { returned=false; }
  fake::delayHook=nullptr;
  return returned;
}
void testSoftReset() {
  fixture(); softResetCalls=0;
  SystemManager::setSoftResetHandler([]{ ++softResetCalls; });
  // SW1 is a start button while stopped (held 0.5 s, then released).
  hold(pins::SAFETY_SWITCH, config::switches::HOLD_MS);
  CHECK(SystemManager::running() && !ButtonInterrupts::emergencyLatched());
  // A short SW1 press while running does nothing; a 0.5 s one stops.
  hold(pins::SAFETY_SWITCH, 300); CHECK(SystemManager::running());
  fake::level(pins::SAFETY_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(config::switches::HOLD_MS+40); ButtonInterrupts::pollEmergencyHold(millis());
  CHECK(ButtonInterrupts::emergencyLatched());
  const gpio_num_t enPins[]={pins::MOTOR1_LEN,pins::MOTOR1_REN,pins::MOTOR2_LEN,pins::MOTOR2_REN};
  for(const auto en:enPins) CHECK(fake::pins[en]==LOW);
  // Still holding SW1, then SW2 joins: no reset until both were released once.
  bool back=runEstopWait([](uint32_t t){
    if(t==10) fake::level(pins::START_SWITCH,LOW);
  }, 1500);
  CHECK(!back && ButtonInterrupts::emergencyLatched() && softResetCalls==0);
  CHECK(EventLogger::status().emergencyOnly && Serial.tx.find("ESTOP")!=std::string::npos);
  // Presses too far apart (SW2 600 ms after SW1) are not a reset either.
  back=runEstopWait([](uint32_t t){
    if(t==1) { fake::level(pins::SAFETY_SWITCH,HIGH); fake::level(pins::START_SWITCH,HIGH); }
    if(t==200) fake::level(pins::SAFETY_SWITCH,LOW);
    if(t==800) fake::level(pins::START_SWITCH,LOW);
  }, 2000);
  CHECK(!back && softResetCalls==0 && !Bts7960Driver::clearDriveInhibit());
  // Released, then both pressed together and held 0.5 s: soft reset.
  back=runEstopWait([](uint32_t t){
    if(t==1) { fake::level(pins::SAFETY_SWITCH,HIGH); fake::level(pins::START_SWITCH,HIGH); }
    if(t==200) fake::level(pins::SAFETY_SWITCH,LOW);
    if(t==350) fake::level(pins::START_SWITCH,LOW);
  }, 3000);
  CHECK(back && softResetCalls==1 && SystemManager::softResetCount()>=1);
  CHECK(!ButtonInterrupts::emergencyLatched() && !EventLogger::status().emergencyOnly);
  CHECK(SystemManager::state()==SystemState::WaitingForStart && !SystemManager::motorAllowed());
  for(int i=0;i<4;++i) CHECK(fake::pwm[i]==0);   // braked (EN may be HIGH for the brake), no drive
  // Releasing the reset buttons must not start the robot.
  fake::advance(300); SystemManager::update(millis());
  fake::level(pins::SAFETY_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(30); fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
  CHECK(!SystemManager::running());
  // A new single-button start drives again (motors re-enabled by the start).
  hold(pins::START_SWITCH, config::switches::HOLD_MS);
  CHECK(SystemManager::running() && MotorController::forward(300,running()));
  // Both pressed while stopped never starts, even after a long hold.
  SystemManager::requestStop(); CHECK(!SystemManager::running());
  fake::level(pins::SAFETY_SWITCH,LOW); fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
  fake::advance(config::switches::HOLD_MS+200); SystemManager::update(millis());
  fake::level(pins::SAFETY_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
  fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(60); SystemManager::update(millis());
  CHECK(!SystemManager::running() && !ButtonInterrupts::emergencyLatched());
  // A software stop (tilt) uses the same latch and the same reset.
  hold(pins::START_SWITCH, config::switches::HOLD_MS); CHECK(SystemManager::running());
  SystemManager::triggerEmergencyStop();
  back=runEstopWait([](uint32_t t){
    if(t==100) { fake::level(pins::SAFETY_SWITCH,LOW); fake::level(pins::START_SWITCH,LOW); }
  }, 2000);
  CHECK(back && softResetCalls==2 && !ButtonInterrupts::emergencyLatched());
  fake::level(pins::SAFETY_SWITCH,HIGH); fake::level(pins::START_SWITCH,HIGH);
  SystemManager::setSoftResetHandler(nullptr);
}

#if HW_VERSION == 2
// Run the battery check until it decides or the window plus margin passes.
BatteryCheckResult runBatteryCheck(uint32_t packMv, uint32_t riseAtMs = 0) {
  fixture(); SystemManager::update(millis());
  fake::adcMillivolts[pins::BATTERY_ADC] = riseAtMs ? 0 : packMv / 11;
  BatteryMonitor::begin(millis());
  for (uint32_t t = 0; t < config::battery::CHECK_WINDOW_MS + 200; t += 10) {
    if (riseAtMs && t >= riseAtMs) fake::adcMillivolts[pins::BATTERY_ADC] = packMv / 11;
    BatteryMonitor::update(millis()); SystemManager::update(millis()); fake::advance(10);
  }
  return BatteryMonitor::status().result;
}
#endif
void testBattery() {
#if HW_VERSION == 2
  // Healthy 4S passes quickly and the robot can start.
  CHECK(runBatteryCheck(15200) == BatteryCheckResult::Ok);
  pressStart(); CHECK(SystemManager::motorAllowed());
  CHECK(BatteryMonitor::status().cells == 4);
  // Full and nearly empty 3S also pass.
  CHECK(runBatteryCheck(12600) == BatteryCheckResult::Ok);
  CHECK(BatteryMonitor::status().cells == 3);
  pressStart(); CHECK(SystemManager::motorAllowed());
  CHECK(runBatteryCheck(10000) == BatteryCheckResult::Ok);
  // No battery (USB only), 2S, a gap voltage (too high for 3S, too low for
  // 4S), and 5S are refused; start is blocked.
  CHECK(runBatteryCheck(0) == BatteryCheckResult::Missing);
  pressStart(); CHECK(!SystemManager::motorAllowed());
  CHECK(SystemManager::state() == SystemState::SystemFault);
  CHECK(runBatteryCheck(8000) == BatteryCheckResult::Low);
  pressStart(); CHECK(!SystemManager::motorAllowed());
  CHECK(runBatteryCheck(13000) == BatteryCheckResult::Low);
  pressStart(); CHECK(!SystemManager::motorAllowed());
  CHECK(runBatteryCheck(21000) == BatteryCheckResult::High);
  pressStart(); CHECK(!SystemManager::motorAllowed());
  // A pack connected 1 s after power-up still passes inside the window.
  CHECK(runBatteryCheck(16000, 1000) == BatteryCheckResult::Ok);
  pressStart(); CHECK(SystemManager::motorAllowed());
  // SW2 pressed while the pack is still being judged is dropped, and does
  // not start the robot later; a new press is needed after Ok.
  fixture(); SystemManager::update(millis());
  fake::adcMillivolts[pins::BATTERY_ADC] = 0;
  BatteryMonitor::begin(millis());
  pressStart(); CHECK(!SystemManager::motorAllowed());
  fake::adcMillivolts[pins::BATTERY_ADC] = 15200 / 11;
  for (int i = 0; i < 50; ++i) {
    BatteryMonitor::update(millis()); SystemManager::update(millis()); fake::advance(10);
  }
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Ok);
  CHECK(!SystemManager::motorAllowed());
  pressStart(); CHECK(SystemManager::motorAllowed());
  // Decided once: later voltage changes do not re-run the check.
  fake::adcMillivolts[pins::BATTERY_ADC] = 0;
  for (int i = 0; i < 50; ++i) { BatteryMonitor::update(millis()); fake::advance(20); }
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Ok);
  // Check switched off in NVS (set on the bench): no pack at all still starts.
  fixture(); SystemManager::update(millis());
  CHECK(BatterySetting::setCheckEnabled(false));
  fake::adcMillivolts[pins::BATTERY_ADC] = 0;   // no pack
  BatteryMonitor::begin(millis());
  for (int i = 0; i < 400; ++i) { BatteryMonitor::update(millis()); SystemManager::update(millis()); fake::advance(10); }
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Disabled);
  CHECK(SystemManager::status().applicationHealthy);
  pressStart(); CHECK(SystemManager::motorAllowed());
  // 11/10: switched off still measures, so a fitting 3S pack reports Ok + 3 cells
  // (the opening move needs it), and SW2 is not held while it measures.
  fixture(); SystemManager::update(millis());
  CHECK(BatterySetting::setCheckEnabled(false));   // fixture() cleared the fake NVS
  fake::adcMillivolts[pins::BATTERY_ADC] = 11800 / 11;
  BatteryMonitor::begin(millis());
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Checking);
  pressStart(); CHECK(SystemManager::motorAllowed());
  for (int i = 0; i < 100; ++i) { BatteryMonitor::update(millis()); SystemManager::update(millis()); fake::advance(10); }
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Ok && BatteryMonitor::status().cells == 3);
  CHECK(SystemManager::motorAllowed());
  CHECK(BatterySetting::setCheckEnabled(true));
#else
  // V1 has no battery input: the check is skipped and start is unaffected.
  fixture(); BatteryMonitor::begin(millis());
  for (int i = 0; i < 400; ++i) { BatteryMonitor::update(millis()); fake::advance(10); }
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::NotAvailable);
  SystemManager::update(millis()); pressStart(); CHECK(SystemManager::motorAllowed());
#endif
}

void testAlignFsm() {
  using namespace config::control;
  const uint32_t OPENING_MS=config::opening::TURN_LEFT_45_MS+config::opening::FORWARD_MS+
                            config::opening::TURN_LEFT_90_MS+3*config::opening::PAUSE_MS;
  SensorSnapshot floor; VisionSnapshot missing;
  auto hazard=[](SensorSnapshot& s,uint8_t corner,FloorColor c){
    s.floorColor[corner].color=c; s.floorColor[corner].reliable=true;
    s.colorRaw[corner].sequence=1; s.colorRaw[corner].valid=true; };
  auto upd=[&](uint32_t t,const SensorSnapshot& s,const VisionSnapshot& v,SafetyStatus safe){
    return AlignFsm::update(t,s,v,safe); };
  {
    // Battery compensation by pack type: 4S x1.0, 3S x THREE_CELL_SCALE, fixed at start.
    using namespace config::opening;
    auto firstDuty=[](uint8_t cells){ OpeningMove::setBatteryCells(cells); OpeningMove::start(0);
                                      const auto d=OpeningMove::intent(0); OpeningMove::cancel(); return d.requestedDuty; };
    CHECK(firstDuty(0)==TURN_DUTY && firstDuty(4)==TURN_DUTY);
    CHECK(firstDuty(3)==TURN_DUTY*THREE_CELL_SCALE_PERMILLE/1000);
    OpeningMove::setBatteryCells(3); OpeningMove::start(0);
    OpeningMove::setBatteryCells(4);                                      // later value: not this run
    CHECK(OpeningMove::intent(0).requestedDuty==TURN_DUTY*THREE_CELL_SCALE_PERMILLE/1000);
    OpeningMove::cancel(); OpeningMove::setBatteryCells(0);
  }
  // Opening move steps, even with the tower in view.
  fixture(); AlignFsm::begin();
  CHECK(upd(0,floor,missing,SafetyStatus{}).type==DriveIntentType::Brake && AlignFsm::state()==RobotState::WaitStart);
  {
    using namespace config::opening;
    auto at=[&](uint32_t t){ return upd(t,floor,target(static_cast<uint8_t>(t/10),t,1900),running()); };
    auto d=at(0); CHECK(AlignFsm::state()==RobotState::Opening && d.type==DriveIntentType::SearchLeft && d.requestedDuty==TURN_DUTY);
    CHECK(at(TURN_LEFT_45_MS).type==DriveIntentType::Brake);
    d=at(TURN_LEFT_45_MS+PAUSE_MS); CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==FORWARD_DUTY && d.steering==0);
    const uint32_t turn90=TURN_LEFT_45_MS+FORWARD_MS+2*PAUSE_MS;
    CHECK(at(turn90).type==DriveIntentType::SearchLeft && at(turn90+TURN_LEFT_90_MS-1).type==DriveIntentType::SearchLeft);
    CHECK(at(turn90+TURN_LEFT_90_MS).type==DriveIntentType::Brake && AlignFsm::state()==RobotState::Opening);
    if (!TRACK_AFTER) {
      CHECK(at(OPENING_MS).type==DriveIntentType::Brake && AlignFsm::state()==RobotState::Stop);
      return;
    }
    CHECK(at(OPENING_MS).type==DriveIntentType::AlignRight && AlignFsm::state()==RobotState::TrackAlign);
  }
  // Helper: a fresh run with the opening already done at time T0.
  const uint32_t T0=OPENING_MS+10;
  auto freshRun=[&]{ fixture(); AlignFsm::begin(); SensorSnapshot clear;
    for(uint32_t t=0;t<T0;t+=10) upd(t,clear,missing,running()); };

  // No tower: spin left; tower on the right: turn right, ramped.
  freshRun();
  auto d=upd(T0,floor,missing,running());
  CHECK(d.type==DriveIntentType::SearchLeft && d.requestedDuty==SEARCH_DUTY && AlignFsm::state()==RobotState::Search);
  d=upd(T0+10,floor,target(1,T0+10,1900),running());
  CHECK(d.type==DriveIntentType::AlignRight && d.requestedDuty==ALIGN_MAX_DUTY);
  {
    // Just outside the window with the tower not moving in the image: slowest turn.
    freshRun(); upd(T0,floor,target(1,T0,TARGET_CENTER_X+TARGET_CENTER_DEADBAND+1),running());
    d=upd(T0+20,floor,target(2,T0+20,TARGET_CENTER_X+TARGET_CENTER_DEADBAND+1),running());
    CHECK(d.type==DriveIntentType::AlignRight && d.requestedDuty==ALIGN_MIN_DUTY);
    // 11/10 rocking fix: the tower sweeps toward the center fast enough to reach
    // the window within ALIGN_LEAD_MS -> brake early instead of overshooting.
    freshRun(); upd(T0,floor,target(1,T0,1500),running());
    d=upd(T0+20,floor,target(2,T0+20,1300),running());      // 10 px/ms: 340 - 800 crosses
    CHECK(d.type==DriveIntentType::Brake && AlignFsm::state()==RobotState::TrackAlign);
    d=upd(T0+40,floor,target(3,T0+40,1290),running());      // slowed down: 330 - 40 stays out
    CHECK(d.type==DriveIntentType::AlignRight);
    // Settled inside the window: 200..260 px does not start a turn again (charges instead).
    freshRun(); upd(T0,floor,target(1,T0,TARGET_CENTER_X+100),running());
    upd(T0+20,floor,target(2,T0+20,TARGET_CENTER_X+100),running());   // settled -> charging
    freshRun(); upd(T0,floor,target(1,T0,TARGET_CENTER_X+100),running());
    CHECK(upd(T0+10,floor,target(1,T0,TARGET_CENTER_X+100),running()).type==DriveIntentType::Brake);  // settled, 1 frame
    CHECK(upd(T0+20,floor,target(2,T0+20,TARGET_CENTER_X+240),running()).type!=DriveIntentType::AlignRight);
    freshRun(); upd(T0,floor,target(1,T0,TARGET_CENTER_X+240),running());
    CHECK(upd(T0+20,floor,target(2,T0+20,TARGET_CENTER_X+240),running()).type==DriveIntentType::AlignRight);  // not settled: turns
    freshRun(); upd(T0,floor,missing,running());
    upd(T0+10,floor,target(1,T0+10,1900),running());
  }
  // Centered: brake while confirming, then charge with steering toward the tower.
  CHECK(upd(T0+30,floor,target(3,T0+30,TARGET_CENTER_X+100),running()).type==DriveIntentType::Brake);
  CHECK(upd(T0+31,floor,target(3,T0+30,TARGET_CENTER_X+100),running()).type==DriveIntentType::Brake);  // same frame
  d=upd(T0+40,floor,target(4,T0+40,TARGET_CENTER_X+100),running());
  CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==CHARGE_DUTY && d.steering>0);
  CHECK(AlignFsm::state()==RobotState::Approach);
  // 11/10 field log: 150..200 px off center was a dead band (no turn, no charge,
  // braked forever). Inside the align window it must always charge.
  freshRun();
  CHECK(upd(T0,floor,target(1,T0,TARGET_CENTER_X+184),running()).type==DriveIntentType::Brake);
  d=upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X+184),running());
  CHECK(d.type==DriveIntentType::Approach && d.steering>0);
  freshRun(); upd(T0,floor,target(1,T0,TARGET_CENTER_X-TARGET_CENTER_DEADBAND),running());
  CHECK(upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X-TARGET_CENTER_DEADBAND),running()).type==DriveIntentType::Approach);
  freshRun(); upd(T0,floor,target(1,T0,1900),running());
  upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X+100),running());
  upd(T0+20,floor,target(3,T0+20,TARGET_CENTER_X+100),running());
  // Charging keeps going while the tower drifts a little, aligns again past the abort error.
  CHECK(upd(T0+50,floor,target(5,T0+50,TARGET_CENTER_X-300),running()).steering<0);
  CHECK(upd(T0+60,floor,target(6,T0+60,TARGET_CENTER_X+CHARGE_ABORT_ERROR+1),running()).type==DriveIntentType::AlignRight);
  // Charge again, then a short drop keeps going straight, a long one searches.
  upd(T0+70,floor,target(7,T0+70),running()); upd(T0+80,floor,target(8,T0+80),running());
  CHECK(AlignFsm::state()==RobotState::Approach);
  d=upd(T0+80+CHARGE_LOST_HOLD_MS-1,floor,missing,running());
  CHECK(d.type==DriveIntentType::Approach && d.steering==0);
  CHECK(upd(T0+80+CHARGE_LOST_HOLD_MS,floor,missing,running()).type==DriveIntentType::SearchRight);  // last x 960: not left of center

  // Escape while charging a close tower (box 600, not the center circle): back off with a turn.
  freshRun();
  upd(T0,floor,target(1,T0,1200,600),running()); upd(T0+10,floor,target(2,T0+10,1100,600),running());
  upd(T0+20,floor,target(3,T0+20,1100,600),running()); CHECK(AlignFsm::state()==RobotState::Approach);
  SensorSnapshot fl; hazard(fl,0,FloorColor::Red);
  // Close opponent: push-through first (11/11), then the escape.
  if (PUSH_THROUGH_MS>0) { upd(T0+30,fl,target(4,T0+30,1100,600),running()); CHECK(AlignFsm::state()!=RobotState::ZoneEscape); }   // still attacking (fork)
  const uint32_t P=PUSH_THROUGH_MS;
  d=upd(T0+30+P,fl,target(5,T0+30+P,1100,600),running());
  CHECK(AlignFsm::state()==RobotState::ZoneEscape && AlignFsm::escapeColor()==FloorColor::Red);
  // 11/11: pushing a close opponent (box 600 >= the red limit) -> straight back-out.
  const int16_t S=STRAIGHT_BACK_ENABLED ? 0 : -config::escape::TURN_STEERING;
  CHECK(d.type==DriveIntentType::Reverse && d.requestedDuty==config::escape::REVERSE_DUTY && d.steering==S);
  hazard(fl,1,FloorColor::Red);              // right sees it after the 60 ms window: still front-left first
  CHECK(upd(T0+100+P,fl,target(6,T0+100+P,1100,600),running()).steering==S);
  CHECK(upd(T0+100+P+config::escape::CLEAR_MS-1,floor,missing,running()).type==DriveIntentType::Reverse);
  d=upd(T0+100+P+config::escape::CLEAR_MS,floor,missing,running());  // clear: lock-on kept
  // Tower last seen at T0+100+P: within ALIGN_LOST_HOLD_MS it brakes first, past it
  // (11/13: CLEAR_MS 500 > hold 300) it searches at once.
  if (config::escape::CLEAR_MS < ALIGN_LOST_HOLD_MS) {
    CHECK(AlignFsm::state()!=RobotState::ZoneEscape && d.type==DriveIntentType::Brake);
    d=upd(T0+100+P+config::escape::CLEAR_MS+ALIGN_LOST_HOLD_MS,floor,missing,running());
  }
  CHECK(AlignFsm::state()!=RobotState::ZoneEscape && d.type==DriveIntentType::SearchRight);   // last seen right of center
  if (STRAIGHT_BACK_ENABLED) {
    // 11/11 straight back-out: only for a close opponent, a closer one on red,
    // and at most STRAIGHT_REPEAT_MAX times in a row from the same side.
    SensorSnapshot fb; hazard(fb,0,FloorColor::Blue);
    SensorSnapshot fr; hazard(fr,0,FloorColor::Red);
    auto escapeAt=[&](uint32_t t0,const SensorSnapshot& hz,uint16_t h){
      // charge at a tower right of center (no fork: off its centre window), then the hazard
      upd(t0,floor,target(1,t0,TARGET_CENTER_X+300,h),running());
      upd(t0+10,floor,target(2,t0+10,TARGET_CENTER_X+190,h),running());
      upd(t0+20,floor,target(3,t0+20,TARGET_CENTER_X+190,h),running());
      CHECK(AlignFsm::state()==RobotState::Approach);
      upd(t0+30,hz,target(4,t0+30,TARGET_CENTER_X+190,h),running());
      const auto e=upd(t0+30+PUSH_THROUGH_MS,hz,target(5,t0+30+PUSH_THROUGH_MS,TARGET_CENTER_X+190,h),running());
      CHECK(AlignFsm::state()==RobotState::ZoneEscape && e.type==DriveIntentType::Reverse);
      for(uint32_t t=t0+40+PUSH_THROUGH_MS;t<t0+60+PUSH_THROUGH_MS+config::escape::CLEAR_MS;t+=10) upd(t,floor,missing,running());
      return e.steering; };
    freshRun();
    CHECK(escapeAt(T0,fr,STRAIGHT_MIN_HEIGHT_RED-1)==-config::escape::TURN_STEERING);   // red, not close enough: turn
    freshRun();
    CHECK(escapeAt(T0,fb,STRAIGHT_MIN_HEIGHT)==0);                                       // blue, close: straight
    uint32_t t=T0+1000;
    for(uint8_t i=1;i<STRAIGHT_REPEAT_MAX;++i,t+=600) CHECK(escapeAt(t,fb,STRAIGHT_MIN_HEIGHT)==0);
    CHECK(escapeAt(t,fb,STRAIGHT_MIN_HEIGHT)==-config::escape::TURN_STEERING);           // one too many: turn
    CHECK(escapeAt(t+600,fb,STRAIGHT_MIN_HEIGHT)==0);                                    // count starts again
  }
  {
    // 11/10 pushed-in exits: decided by the side that went in first, kept when fully in.
    using namespace config::escape;
    auto snap=[&](std::initializer_list<uint8_t> corners,FloorColor c){
      SensorSnapshot s; for(uint8_t i:corners) hazard(s,i,c); return s; };
    const SensorSnapshot all=snap({0,1,2,3},FloorColor::Red);
    // Front right first -> reverse, nose the other way from front left.
    freshRun(); d=upd(T0,snap({1},FloorColor::Blue),missing,running());
    CHECK(d.type==DriveIntentType::Reverse && d.steering==TURN_STEERING && AlignFsm::escapeColor()==FloorColor::Blue);
    // Rear touches later (outside the window): keep reversing; fully in: reverse harder.
    CHECK(upd(T0+100,snap({2},FloorColor::Blue),missing,running()).type==DriveIntentType::Reverse);
    d=upd(T0+110,all,missing,running());
    CHECK(d.type==DriveIntentType::Reverse && d.requestedDuty==std::min<uint16_t>(PUSHED_DUTY,MOTOR_LIMIT-TURN_STEERING) && d.steering==TURN_STEERING);  // pushed in: harder, turn never clipped   // was: forward, deeper in
    // Pushed backward in (rear both first): forward, and still forward when fully in.
    freshRun(); d=upd(T0,snap({2,3},FloorColor::Red),missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.steering==0 && d.requestedDuty==FORWARD_DUTY);
    d=upd(T0+100,all,missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==PUSHED_DUTY);   // was: reverse, deeper in
    // Not off after FLIP_MS: the other way.
    CHECK(upd(T0+FLIP_MS,all,missing,running()).type==DriveIntentType::Reverse);
    // Rear left first -> forward, nose right; rear right -> nose left.
    freshRun(); d=upd(T0,snap({2},FloorColor::Red),missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.steering==REAR_STEERING);
    freshRun(); CHECK(upd(T0,snap({3},FloorColor::Red),missing,running()).steering==-REAR_STEERING);
    // Pushed sideways: left side first -> forward hard right (also when the two
    // sensors confirm one pass apart); right side -> hard left.
    freshRun(); upd(T0,snap({0},FloorColor::Blue),missing,running());
    d=upd(T0+20,snap({0,2},FloorColor::Blue),missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.steering==SIDE_STEERING);
    freshRun(); d=upd(T0,snap({1,3},FloorColor::Blue),missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.steering==-SIDE_STEERING);
    // Unclear (diagonal): reverse if we were charging, else forward.
    freshRun(); CHECK(upd(T0,snap({0,3},FloorColor::Blue),missing,running()).type==DriveIntentType::Approach);
    freshRun(); upd(T0,floor,target(1,T0,960,600),running()); upd(T0+10,floor,target(2,T0+10,960,600),running());
    CHECK(AlignFsm::state()==RobotState::Approach);
    CHECK(upd(T0+20,snap({0,3},FloorColor::Blue),target(3,T0+20,960,600),running()).type==DriveIntentType::Reverse);
  }
  {
    // Red while locked on a far tower (box < TOWER_MAX_HEIGHT) = center circle: go around it.
    using namespace config::circle;
    auto lockFar=[&]{ freshRun();
      upd(T0,floor,target(1,T0,1000,TOWER_MAX_HEIGHT-1),running());
      upd(T0+10,floor,target(2,T0+10,1000,TOWER_MAX_HEIGHT-1),running()); };
    lockFar(); SensorSnapshot red; hazard(red,0,FloorColor::Red);       // circle on the left
    d=upd(T0+20,red,target(3,T0+20,1000,TOWER_MAX_HEIGHT-1),running());
    CHECK(AlignFsm::state()==RobotState::CircleBypass && d.type==DriveIntentType::Reverse &&
          d.requestedDuty==BACK_DUTY && d.steering==0);
    // 11/13: the reverse lasts at least BACK_MS and until the front has been
    // clear for BACK_CLEAR_MS (clear from T0+30 here, so it ends at E).
    static_assert(BACK_CLEAR_MS + 10 <= BACK_MS, "test timing");
    const uint32_t E=T0+20+BACK_MS;
    CHECK(upd(T0+30,floor,missing,running()).type==DriveIntentType::Reverse);   // minimum time
    CHECK(upd(E-1,floor,missing,running()).type==DriveIntentType::Reverse);
    d=upd(E,floor,missing,running());
    CHECK(d.type==DriveIntentType::SearchRight && d.requestedDuty==TURN_DUTY);   // away from the circle
    d=upd(E+TURN_MS,floor,missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==ARC_DUTY && d.steering==-ARC_STEERING);
    // Touching red again in the arc: next step (reverse again).
    CHECK(upd(E+TURN_MS+100,red,missing,running()).type==DriveIntentType::Brake);
    CHECK(upd(E+TURN_MS+110,red,missing,running()).type==DriveIntentType::Reverse);
    // Arc done without touching: back to tracking (tower still in view -> align/charge).
    const uint32_t t1=E+TURN_MS+100;
    upd(t1+20,floor,missing,running());
    upd(t1+BACK_MS,floor,missing,running());                                   // reverse ends, turn starts
    CHECK(upd(t1+BACK_MS+TURN_MS+ARC_MS,floor,target(4,t1+BACK_MS+TURN_MS+ARC_MS,1000,200),running()).type!=DriveIntentType::Reverse);
    CHECK(AlignFsm::state()==RobotState::TrackAlign);
    // Front still on the red after BACK_MS: keep backing (the lip), but never past BACK_MAX_MS.
    lockFar(); upd(T0+20,red,target(3,T0+20,1000,TOWER_MAX_HEIGHT-1),running());
    CHECK(upd(E,red,missing,running()).type==DriveIntentType::Reverse);
    CHECK(upd(T0+20+BACK_MAX_MS-1,red,missing,running()).type==DriveIntentType::Reverse);
    CHECK(upd(T0+20+BACK_MAX_MS,red,missing,running()).type==DriveIntentType::SearchRight);
    // Tower not seen during the bypass: it turned right, so search left at once
    // (the old x 1000 would have said right: 11/10 field bug).
    lockFar(); upd(T0+20,red,target(3,T0+20,1000,TOWER_MAX_HEIGHT-1),running());
    for(uint32_t t=T0+30;t<E+TURN_MS+ARC_MS;t+=10) upd(t,floor,missing,running());
    d=upd(E+TURN_MS+ARC_MS,floor,missing,running());
    CHECK(d.type==DriveIntentType::SearchLeft && AlignFsm::state()==RobotState::Search);
    // Seen during the arc: that newer position wins.
    lockFar(); upd(T0+20,red,target(3,T0+20,1000,TOWER_MAX_HEIGHT-1),running());
    upd(T0+30,floor,missing,running()); upd(E,floor,missing,running());
    upd(E+TURN_MS+50,floor,target(4,E+TURN_MS+50,1500,200),running());
    d=upd(E+TURN_MS+ARC_MS,floor,target(5,E+TURN_MS+ARC_MS,1500,200),running());
    CHECK(d.type==DriveIntentType::AlignRight);
    // Front right first -> turn left, arc right.
    lockFar(); SensorSnapshot redR; hazard(redR,1,FloorColor::Red);
    upd(T0+20,redR,missing,running()); upd(T0+30,floor,missing,running());
    CHECK(upd(E,floor,missing,running()).type==DriveIntentType::SearchLeft);
    CHECK(upd(E+TURN_MS,floor,missing,running()).steering==ARC_STEERING);
    // Rear on a hazard while reversing -> normal escape.
    lockFar(); upd(T0+20,red,missing,running());
    SensorSnapshot back; hazard(back,2,FloorColor::Red);
    upd(T0+30,back,missing,running()); CHECK(AlignFsm::state()==RobotState::ZoneEscape);
    // Blue (corner groove) or a close tower: normal escape, not the bypass.
    lockFar(); SensorSnapshot blue; hazard(blue,0,FloorColor::Blue);
    upd(T0+20,blue,missing,running()); CHECK(AlignFsm::state()==RobotState::ZoneEscape);
    freshRun(); upd(T0,floor,target(1,T0,1000,TOWER_MAX_HEIGHT),running());
    upd(T0+20,red,missing,running()); CHECK(AlignFsm::state()==RobotState::ZoneEscape);
    // Tower not seen recently: normal escape.
    lockFar(); upd(T0+10+LOCK_RECENT_MS,red,missing,running()); CHECK(AlignFsm::state()==RobotState::ZoneEscape);
  }
  {
    // A stop forgets the floor first-hit side: the next run classifies afresh.
    SensorSnapshot fl2; hazard(fl2,0,FloorColor::Red); SensorSnapshot rr; hazard(rr,3,FloorColor::Blue);
    freshRun(); upd(T0,fl2,missing,running());                       // front left first
    upd(T0+10,fl2,missing,SafetyStatus{});                          // stopped while on it
    for(uint32_t t=T0+20;t<T0+20+OPENING_MS+20;t+=10) upd(t,floor,missing,running());   // new run, opening done
    const auto e=upd(T0+30+OPENING_MS,rr,missing,running());        // rear right first now
    CHECK(e.type==DriveIntentType::Approach && e.steering==-config::escape::REAR_STEERING);
  }
  {
    // Fork attack: charging at a close centered tower -> brake, insert, burst, keep charging.
    using namespace config::fork;
    auto close=[&](uint8_t seq,uint32_t t,uint16_t x=TARGET_CENTER_X+50){ return target(seq,t,x,TRIGGER_HEIGHT); };
    freshRun();
    upd(T0,floor,target(1,T0,TARGET_CENTER_X,300),running()); upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X,300),running());
    CHECK(AlignFsm::state()==RobotState::Approach);                         // charging, still far
    d=upd(T0+20,floor,close(3,T0+20),running());
    CHECK(d.type==DriveIntentType::Brake && AlignFsm::state()==RobotState::DiveBrake);
    CHECK(upd(T0+20+BRAKE_MS-1,floor,missing,running()).type==DriveIntentType::Brake);   // blind: still brakes
    d=upd(T0+20+BRAKE_MS,floor,close(4,T0+20+BRAKE_MS),running());
    CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==INSERT_DUTY && d.steering>0 &&
          AlignFsm::state()==RobotState::SpokeEngaged);
    d=upd(T0+20+BRAKE_MS+INSERT_MS,floor,missing,running());
    CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==BURST_DUTY && d.steering==0 &&
          AlignFsm::state()==RobotState::Push);
    const uint32_t end=T0+20+BRAKE_MS+INSERT_MS+BURST_MS;
    d=upd(end,floor,close(5,end),running());                              // done: back to charging
    CHECK(d.type==DriveIntentType::Approach && d.requestedDuty==CHARGE_DUTY && AlignFsm::state()==RobotState::Approach);
    // Cooldown: no new attack right away, then again.
    CHECK(upd(end+10,floor,close(6,end+10),running()).requestedDuty==CHARGE_DUTY);
    CHECK(upd(end+COOLDOWN_MS,floor,close(7,end+COOLDOWN_MS),running()).type==DriveIntentType::Brake);
    // Not centered enough or not close enough: just charge.
    freshRun();
    upd(T0,floor,target(1,T0,TARGET_CENTER_X,300),running()); upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X,300),running());
    CHECK(upd(T0+20,floor,close(3,T0+20,TARGET_CENTER_X+TRIGGER_CENTER+1),running()).requestedDuty==CHARGE_DUTY);
    CHECK(upd(T0+30,floor,target(4,T0+30,TARGET_CENTER_X,TRIGGER_HEIGHT-1),running()).requestedDuty==CHARGE_DUTY);
    // A front hazard with the opponent close (11/11 push-through): keep
    // pushing PUSH_THROUGH_MS, then the escape wins and cancels the attack.
    freshRun();
    upd(T0,floor,target(1,T0,TARGET_CENTER_X,300),running()); upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X,300),running());
    upd(T0+20,floor,close(3,T0+20),running()); upd(T0+20+BRAKE_MS,floor,close(4,T0+20+BRAKE_MS),running());
    SensorSnapshot fl3; hazard(fl3,0,FloorColor::Blue);
    const uint32_t hit=T0+30+BRAKE_MS;
    if (PUSH_THROUGH_MS>0) {
      d=upd(hit,fl3,close(5,hit),running());
      CHECK(d.type==DriveIntentType::Approach && AlignFsm::state()!=RobotState::ZoneEscape);   // still pushing (charge; fork in cooldown)
      d=upd(hit+PUSH_THROUGH_MS-1,fl3,close(6,hit+PUSH_THROUGH_MS-1),running());
      CHECK(d.type==DriveIntentType::Approach && AlignFsm::state()!=RobotState::ZoneEscape);
    }
    CHECK(upd(hit+PUSH_THROUGH_MS,fl3,close(7,hit+PUSH_THROUGH_MS),running()).type==DriveIntentType::Reverse);
    CHECK(AlignFsm::state()==RobotState::ZoneEscape && ForkAttack::phase(hit+PUSH_THROUGH_MS+10)==ForkPhase::Idle);
    // Push-through only for a close opponent and a front-only hazard.
    {
      SensorSnapshot rl; hazard(rl,2,FloorColor::Blue);
      SensorSnapshot fl4; hazard(fl4,0,FloorColor::Blue);
      auto charge=[&](uint16_t h){
        freshRun();
        upd(T0,floor,target(1,T0,TARGET_CENTER_X+300,h),running());
        upd(T0+10,floor,target(2,T0+10,TARGET_CENTER_X+150,h),running());
        upd(T0+20,floor,target(3,T0+20,TARGET_CENTER_X+150,h),running());
        CHECK(AlignFsm::state()==RobotState::Approach); };
      charge(PUSH_THROUGH_MIN_HEIGHT);                                   // close: front hit waits
      upd(T0+30,fl4,target(4,T0+30,TARGET_CENTER_X+150,PUSH_THROUGH_MIN_HEIGHT),running());
      CHECK(PUSH_THROUGH_MS==0 || AlignFsm::state()==RobotState::Approach);
      charge(PUSH_THROUGH_MIN_HEIGHT);                                   // close but rear hit: at once
      upd(T0+30,rl,target(4,T0+30,TARGET_CENTER_X+150,PUSH_THROUGH_MIN_HEIGHT),running());
      CHECK(AlignFsm::state()==RobotState::ZoneEscape);
      charge(PUSH_THROUGH_MIN_HEIGHT-1);                                 // far: at once
      upd(T0+30,fl4,target(4,T0+30,TARGET_CENTER_X+150,PUSH_THROUGH_MIN_HEIGHT-1),running());
      CHECK(AlignFsm::state()==RobotState::ZoneEscape);
    }
  }
  // A hazard during the opening move cancels it and escapes.
  fixture(); AlignFsm::begin(); upd(0,floor,missing,running());
  CHECK(AlignFsm::state()==RobotState::Opening);
  d=upd(config::opening::TURN_LEFT_45_MS+config::opening::PAUSE_MS+100,fl,missing,running());
  CHECK(AlignFsm::state()==RobotState::ZoneEscape && d.type==DriveIntentType::Reverse);
  for(uint32_t t=config::opening::TURN_LEFT_45_MS+config::opening::PAUSE_MS+110;t<5000;t+=10) upd(t,floor,missing,running());
  CHECK(AlignFsm::state()==RobotState::Search);                     // opening not resumed
  // Safety stop wins.
  SafetyStatus estop; estop.state=SafetyState::EmergencyStop;
  CHECK(upd(6000,fl,target(9,6000,1900),estop).type==DriveIntentType::Brake);
  CHECK(AlignFsm::state()==RobotState::EmergencyStop);
}

void testRunLog() {
  fixture(); RunLog::reset(); Serial1.tx.clear();
  RunLog::push(RunLogKind::State,0x1234,2,0xFFFF,0); RunLog::process();
  const std::string f=Serial1.tx;
  CHECK(f.size()==14);
  const uint8_t expect[14]={0xAA,0x55,0x81,0x0A,0,2,0x34,0x12,2,0,0xFF,0xFF,0,0};
  CHECK(std::memcmp(f.data(),expect,14)==0);
  // TX buffer full: frames wait in the queue, never block; a full queue drops.
  Serial1.tx.clear(); Serial1.capacity=13; RunLog::push(RunLogKind::Stop,1); RunLog::process();
  CHECK(Serial1.tx.empty());
  Serial1.capacity=4096; RunLog::process(); CHECK(Serial1.tx.size()==14 && static_cast<uint8_t>(Serial1.tx[4])==1);
  for(int i=0;i<config::run_log::QUEUE_SIZE+3;++i) RunLog::push(RunLogKind::Alive);
  CHECK(RunLog::dropped()==3);
  Serial1.tx.clear(); RunLog::process(); CHECK(Serial1.tx.size()==14u*config::run_log::QUEUE_SIZE);
  // Alive: 5 Hz while driving only.
  RunLog::reset(); Serial1.tx.clear();
  for(uint32_t t=0;t<1000;t+=10) RunLog::alive(t,true,3,960,6,500);
  RunLog::alive(1000,false,0,0,0,0); RunLog::alive(1010,false,0,0,0,0);
  RunLog::process(); CHECK(Serial1.tx.size()==14u*5);
}

// 11/13: reset reason + the state before it -> one Boot frame to the Pi.
void testResetReport() {
  fixture(); RunLog::reset(); Serial1.tx.clear();
  auto frame=[](){ RunLog::process(); std::string f=Serial1.tx; Serial1.tx.clear(); return f; };
  auto u16=[](const std::string& f,int i){ return static_cast<uint16_t>(static_cast<uint8_t>(f[i])|static_cast<uint8_t>(f[i+1])<<8); };
  ResetReport::begin(1,1000);                              // power-on, nothing recorded yet
  std::string f=frame();
  CHECK(f.size()==14 && static_cast<uint8_t>(f[5])==8 && (u16(f,6)&0xFF)==1 && u16(f,8)==0xFFFF);
  const uint16_t first=u16(f,6);                           // reason | boot id << 8
  // Repeats every REPEAT_MS until the first start, the same payload each time.
  ResetReport::update(1000+config::reset_report::REPEAT_MS-1,false); CHECK(frame().empty());
  ResetReport::update(1000+config::reset_report::REPEAT_MS,false);
  f=frame(); CHECK(f.size()==14 && u16(f,6)==first && u16(f,8)==0xFFFF);
  ResetReport::update(1000+2*config::reset_report::REPEAT_MS,true); CHECK(frame().empty());   // started
  ResetReport::update(1000+9*config::reset_report::REPEAT_MS,false); CHECK(frame().empty());  // never again
  ResetReport::record(12345,true,5,6,800);                 // charging at 800, 12.3 s after boot
  ResetReport::begin(9,0);                                 // brownout restart
  f=frame();
  CHECK((u16(f,6)&0xFF)==9 && u16(f,8)==0x105 && u16(f,10)==123 && u16(f,12)==((6<<12)|800));
  ResetReport::begin(1,0);                                 // the RTC copy is used once
  f=frame(); CHECK(u16(f,8)==0xFFFF);
  CHECK(ResetReport::abnormal(9) && ResetReport::abnormal(4) && ResetReport::abnormal(6));
  CHECK(!ResetReport::abnormal(1) && !ResetReport::abnormal(2));
  // 11/13 auto resume: only a watchdog reset while driving.
  ResetReport::record(5000,true,2,2,500); ResetReport::begin(6,0); CHECK(ResetReport::resumeRequested());
  ResetReport::record(5000,true,2,2,500); ResetReport::begin(9,0); CHECK(!ResetReport::resumeRequested());   // brownout
  ResetReport::record(5000,true,2,2,500); ResetReport::begin(1,0); CHECK(!ResetReport::resumeRequested());   // power-on
  ResetReport::record(5000,false,1,0,0);  ResetReport::begin(6,0); CHECK(ResetReport::resumeRequested());    // waiting too (11/13 2nd)
  ResetReport::record(5000,true,2,2,500); ResetReport::noteStopped(); ResetReport::begin(5,0);
  CHECK(!ResetReport::resumeRequested());                                                                    // ESTOP before
  ResetReport::begin(6,0); CHECK(ResetReport::resumeRequested());                                            // no RTC copy: still resume
}

void testHuskyLens() {
  // Old V1 robot (11/10): HuskyLens color blocks -> the same VisionSnapshot as the Pi.
  fixture(); HuskyLensLink::begin();
  auto frameOf=[](uint8_t cmd,std::vector<uint8_t> data){
    std::vector<uint8_t> f={0x55,0xAA,0x11,static_cast<uint8_t>(data.size()),cmd};
    f.insert(f.end(),data.begin(),data.end());
    uint8_t s=0; for(uint8_t b:f) s=static_cast<uint8_t>(s+b); f.push_back(s); return f; };
  auto u16=[](std::vector<uint8_t>& v,uint16_t x){ v.push_back(x&0xFF); v.push_back(x>>8); };
  auto block=[&](uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t id){
    std::vector<uint8_t> d; u16(d,x); u16(d,y); u16(d,w); u16(d,h); u16(d,id); return frameOf(0x2A,d); };
  auto info=[&](uint16_t n,uint16_t frameNo){
    std::vector<uint8_t> d; u16(d,n); u16(d,1); u16(d,frameNo); u16(d,0); u16(d,0); return frameOf(0x29,d); };
  auto txBytes=[](){ return std::vector<uint8_t>(Serial1.tx.begin(),Serial1.tx.end()); };
  // Like the DFRobot example: knock (0x2C) at 9600 until OK, then request() 0x20.
  fake::now=1000; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x2C,{}) && Serial1.baud==9600);
  CHECK(!HuskyLensLink::latest().linkValid);
  Serial1.tx.clear(); Serial1.inject(frameOf(0x2E,{})); fake::now=1005; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x20,{}));                 // then: request blocks
  // Answer: three learned blocks (TARGET_ID 0 = any ID): the largest is the tower.
  Serial1.tx.clear();
  Serial1.inject(info(3,7)); Serial1.inject(block(40,60,10,10,1));
  Serial1.inject(block(160,120,40,80,1)); Serial1.inject(block(300,10,20,20,2));
  fake::now=1010; HuskyLensLink::poll();
  VisionSnapshot v=HuskyLensLink::latest();
  CHECK(v.linkValid && v.frameFresh && v.targetValid && v.cameraOk && v.pipelineOk);
  CHECK(v.targetX==960 && v.targetY==540 && v.targetWidth==240 && v.targetHeight==360);
  CHECK(v.multipleTargets && !v.boxClipped && v.sequence==7);
  // Waits REQUEST_PERIOD_MS before the next request, never two at once.
  CHECK(Serial1.tx.empty());
  fake::now=1010+config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x20,{}));
  // Same frame number again: link alive, not a new frame.
  Serial1.tx.clear(); Serial1.inject(info(0,7)); fake::now+=5; HuskyLensLink::poll();
  CHECK(HuskyLensLink::status().duplicatePackets==1 && HuskyLensLink::latest().targetValid);
  // New frame with no tower: target cleared. Clipped box flagged.
  fake::now+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  Serial1.inject(info(0,8)); fake::now+=5; HuskyLensLink::poll();
  CHECK(HuskyLensLink::latest().linkValid && !HuskyLensLink::latest().targetValid);
  fake::now+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  Serial1.inject(info(1,9)); Serial1.inject(block(5,120,20,40,1)); fake::now+=5; HuskyLensLink::poll();
  CHECK(HuskyLensLink::latest().boxClipped && HuskyLensLink::latest().targetX==30);
  // A bad checksum is dropped and counted.
  const uint32_t bad=HuskyLensLink::status().invalidPackets;
  auto broken=info(0,10); broken.back()^=0xFF;
  fake::now+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  Serial1.inject(broken); fake::now+=5; HuskyLensLink::poll();
  CHECK(HuskyLensLink::status().invalidPackets==bad+1);
  // HuskyLens silent: link drops after LINK_TIMEOUT_MS, target gone (robot searches).
  fake::now+=config::huskylens::LINK_TIMEOUT_MS+1; HuskyLensLink::poll();
  CHECK(!HuskyLensLink::latest().linkValid && !HuskyLensLink::latest().targetValid);
  // An arrow result is counted but never becomes the tower.
  fake::now+=config::huskylens::RECONNECT_MS; HuskyLensLink::poll();   // silent -> knock again
  CHECK(txBytes().size()>=6);
  Serial1.tx.clear(); Serial1.inject(frameOf(0x2E,{})); fake::now+=5; HuskyLensLink::poll();
  { std::vector<uint8_t> d; u16(d,10); u16(d,20); u16(d,30); u16(d,40); u16(d,1);
    Serial1.inject(info(1,11)); Serial1.inject(frameOf(0x2B,d)); fake::now+=5; HuskyLensLink::poll(); }
  CHECK(HuskyLensLink::latest().linkValid && !HuskyLensLink::latest().targetValid);
  // No answer at 9600: after BAUD_SWITCH_KNOCKS knocks it tries 115200, and back.
  fixture(); HuskyLensLink::begin();
  uint32_t t=1000;
  for(int i=0;i<config::huskylens::BAUD_SWITCH_KNOCKS;++i){ fake::now=t; HuskyLensLink::poll(); t+=config::huskylens::RESPONSE_TIMEOUT_MS+config::huskylens::KNOCK_RETRY_MS; }
  CHECK(Serial1.baud==9600);
  fake::now=t; HuskyLensLink::poll();
  CHECK(Serial1.baud==115200 && HuskyLensLink::activeBaud()==115200);
  // It answers at 115200: stays there and requests blocks.
  Serial1.tx.clear(); Serial1.inject(frameOf(0x2E,{})); fake::now=t+5; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x20,{}) && Serial1.baud==115200);
  // Review fixes (11/10).
  // 1) Knock search covers 1000000 too: 9600 -> 115200 -> 1000000 -> 9600.
  fixture(); HuskyLensLink::begin();
  t=1000;
  auto knockRound=[&](){ for(int i=0;i<config::huskylens::BAUD_SWITCH_KNOCKS;++i){ fake::now=t; HuskyLensLink::poll(); t+=config::huskylens::RESPONSE_TIMEOUT_MS+config::huskylens::KNOCK_RETRY_MS; } };
  knockRound(); fake::now=t; HuskyLensLink::poll(); CHECK(Serial1.baud==115200);
  knockRound(); fake::now=t; HuskyLensLink::poll(); CHECK(Serial1.baud==1000000);
  knockRound(); fake::now=t; HuskyLensLink::poll(); CHECK(Serial1.baud==9600);
  // Connect at 9600 for the next checks.
  Serial1.tx.clear(); Serial1.inject(frameOf(0x2E,{})); fake::now=t+=5; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x20,{}));
  // 2) A long answer is not cut off: bytes keep arriving past RESPONSE_TIMEOUT_MS.
  Serial1.inject(info(10,20));
  for(int i=0;i<9;++i){ Serial1.inject(block(10,10,4,4,2)); fake::now=t+=40; HuskyLensLink::poll(); }
  CHECK(!HuskyLensLink::latest().frameFresh || HuskyLensLink::latest().sequence!=20);
  Serial1.inject(block(160,120,40,80,1)); fake::now=t+=40; HuskyLensLink::poll();
  CHECK(HuskyLensLink::latest().sequence==20 && HuskyLensLink::latest().targetValid &&
        HuskyLensLink::latest().targetX==960);
  // 3) ID 0 (seen, not learned) is never the tower, even when it is the biggest.
  fake::now=t+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  Serial1.inject(info(1,21)); Serial1.inject(block(160,120,200,200,0)); fake::now=t+=5; HuskyLensLink::poll();
  CHECK(HuskyLensLink::latest().sequence==21 && !HuskyLensLink::latest().targetValid);
  // 4) 0x3D BUSY: no waiting out the timeout, asks again after REQUEST_PERIOD_MS.
  fake::now=t+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  Serial1.tx.clear(); Serial1.inject(frameOf(0x3D,{})); fake::now=t+=5; HuskyLensLink::poll();
  fake::now=t+=config::huskylens::REQUEST_PERIOD_MS; HuskyLensLink::poll();
  CHECK(txBytes()==frameOf(0x20,{}));
  // The run log stays silent only on HuskyLens builds; this host build uses the Pi.
  CHECK(!config::uart::VISION_FROM_HUSKYLENS);
}

void testPoseEstimate() {
  // Log-only pose: gyro heading about the learned "up" axis, commanded distance.
  PoseEstimate::reset();
  ImuStatus imu; imu.dataFresh=true; ImuSample s; uint32_t t=0;
  auto feed=[&](int16_t gz,bool run,int16_t l,int16_t r,int16_t ax=0,int16_t az=4096){
    t+=10; s.timestampMs=t; s.accel[0]=ax; s.accel[1]=0; s.accel[2]=az;
    s.gyro[0]=5; s.gyro[1]=-3; s.gyro[2]=gz; PoseEstimate::update(t,run,imu,s,l,r); };
  for(int i=0;i<100;++i) feed(20,false,0,0);          // at rest: bias (5,-3,20) learned, z up
  feed(20,true,0,0);                                   // SW2: pose zeroed
  CHECK(PoseEstimate::pose().imuOk && std::abs(PoseEstimate::pose().headingDeg)<0.01f);
  // Left turn at 90 deg/s for 0.5 s -> +45 deg (32.8 LSB per deg/s, plus the bias).
  for(int i=0;i<50;++i) feed(static_cast<int16_t>(20+90*32.8f),true,-300,300);
  CHECK(std::abs(PoseEstimate::pose().headingDeg-45.0f)<1.0f);
  CHECK(std::abs(PoseEstimate::pose().xMm)<1.0f);      // spin in place: no distance
  // 1 s straight at 500 -> ~500 mm along 45 deg.
  for(int i=0;i<100;++i) feed(20,true,500,500);
  CHECK(std::abs(PoseEstimate::pose().xMm-353.6f)<15 && std::abs(PoseEstimate::pose().yMm-353.6f)<15);
  // Stop and a new run start from zero again; stale IMU holds the heading.
  feed(20,false,0,0); feed(20,true,0,0);
  CHECK(std::abs(PoseEstimate::pose().xMm)<0.01f && std::abs(PoseEstimate::pose().headingDeg)<0.01f);
  imu.dataFresh=false; for(int i=0;i<20;++i) feed(2000,true,0,0);
  CHECK(!PoseEstimate::pose().imuOk && std::abs(PoseEstimate::pose().headingDeg)<0.01f);
}

void testMainIntegration() {
  fake::reset(); setup();
  for(uint8_t i=0;i<4;++i) fake::floor(i,3);
  for(int i=0;i<300;++i) loop();
  CHECK(!MotorController::status().outputsActive);
  // Calibration is a separate firmware: production ignores its commands.
  Serial.inject("CAL START\n"); for(int i=0;i<10;++i) loop();
  CHECK(!SystemManager::motorAllowed()); CHECK(Serial.tx.find("[CAL]")==std::string::npos);
  fake::level(pins::START_SWITCH,LOW);
  for(const uint32_t pressed=millis(); millis()-pressed<config::switches::HOLD_MS+60;) loop();
  fake::level(pins::START_SWITCH,HIGH); for(int i=0;i<50;++i) loop();
  CHECK(SystemManager::motorAllowed()); CHECK(MotorController::status().outputsActive);
  const auto fsmState=[]{ return config::control::ALIGN_ONLY_FSM ? AlignFsm::state() : RobotFsm::state(); };
  if (config::control::ALIGN_ONLY_FSM) {   // run log: a Start frame went to the Pi on SW2
    const std::string start{static_cast<char>(0xAA),0x55,static_cast<char>(0x81),0x0A};
    bool found=false;
    for(size_t at=Serial1.tx.find(start);at!=std::string::npos;at=Serial1.tx.find(start,at+1))
      found|=at+14<=Serial1.tx.size() && Serial1.tx[at+5]==1;
    CHECK(found);
  }
  if (config::control::ALIGN_ONLY_FSM && config::opening::ENABLED) {
    CHECK(fsmState()==RobotState::Opening);                   // fixed opening move first
    for(int i=0;i<5000 && fsmState()==RobotState::Opening;++i) loop();
    if (!config::opening::TRACK_AFTER) {   // tracker off: stays braked
      CHECK(fsmState()==RobotState::Stop);
      Serial1.inject(packet(1)); loop(); Serial1.inject(packet(2)); loop();
      CHECK(fsmState()==RobotState::Stop);
      return;
    }
  }
  CHECK(fsmState()==RobotState::Search);
  Serial1.inject(packet(1)); loop(); Serial1.inject(packet(2)); loop();
  CHECK(fsmState()==(config::control::ALIGN_ONLY_FSM && !config::control::CHARGE_ENABLED
                         ? RobotState::TrackAlign : RobotState::Approach));
  fake::advance(110); CHECK(!MotorController::status().outputsActive);
  loop(); CHECK(SystemManager::motorAllowed());  // No restart policy for a brief command expiry.
}

void testWanderAndTilt() {
  fixture(); RobotFsm::begin(); SensorSnapshot sensors; VisionSnapshot missing;
  // Pi alive but no tower: the normal spin search.
  VisionSnapshot empty=target(1,1000); empty.targetValid=false;
  RobotFsm::update(1000,sensors,empty,running()); CHECK(RobotFsm::state()==RobotState::Search);
  // 11/09: Wander is off (WANDER_WHEN_VISION_LOST): a silent Pi keeps the same spin.
  static_assert(!config::control::WANDER_WHEN_VISION_LOST, "update this test when Wander is back on");
  const auto firstSpin=RobotFsm::update(1000+config::wander::VISION_LOST_MS,sensors,missing,running()).type;
  for(uint32_t ms=1000+config::wander::VISION_LOST_MS;ms<12000;ms+=50) {
    CHECK(RobotFsm::update(ms,sensors,missing,running()).type==firstSpin);
    CHECK(RobotFsm::state()==RobotState::Search);
  }
  // The Wander module itself (still built, used again if the switch is turned on):
  // over 20 s it alternates turn / forward with changing lengths and sides.
  uint32_t t=2000; DriveIntentType last=DriveIntentType::Brake; uint32_t segStart=t;
  int forwards=0,lefts=0,rights=0; uint32_t minSeg=UINT32_MAX,maxSeg=0; bool consecutiveSame=false;
  Wander::reset(t);
  for(;t<22000;t+=10) {
    const auto d=Wander::intent(t);
    CHECK(d.type==DriveIntentType::Approach||d.type==DriveIntentType::SearchLeft||d.type==DriveIntentType::SearchRight);
    if(d.type!=last) {
      const bool lastTurn=last!=DriveIntentType::Approach, nowTurn=d.type!=DriveIntentType::Approach;
      if(last!=DriveIntentType::Brake && lastTurn==nowTurn) consecutiveSame=true;
      if(last!=DriveIntentType::Brake) { minSeg=std::min(minSeg,t-segStart); maxSeg=std::max(maxSeg,t-segStart); }
      if(d.type==DriveIntentType::Approach) { ++forwards; CHECK(d.requestedDuty>=config::wander::FORWARD_DUTY_MIN&&d.requestedDuty<=config::wander::FORWARD_DUTY_MAX); }
      if(d.type==DriveIntentType::SearchLeft) ++lefts;
      if(d.type==DriveIntentType::SearchRight) ++rights;
      last=d.type; segStart=t;
    }
  }
  CHECK(!consecutiveSame && forwards>5 && lefts>0 && rights>0 && maxSeg>minSeg+150);
  // A floor hazard still wins over the search.
  sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
  CHECK(RobotFsm::update(t,sensors,missing,running()).type==DriveIntentType::Reverse);
  CHECK(RobotFsm::state()==RobotState::ZoneEscape);

  // TiltGuard: learn level while stopped, trip only after HOLD_MS past the limit.
  TiltGuard::begin(); ImuStatus imu; imu.dataFresh=true; ImuSample s; uint32_t ms=0;
  auto feed=[&](int16_t x,int16_t y,int16_t z,bool run){ ms+=10; s.accel[0]=x; s.accel[1]=y; s.accel[2]=z; s.timestampMs=ms;
    return TiltGuard::update(ms,run,imu,s); };
  for(int i=0;i<20;++i) CHECK(!feed(0,0,4096,false));     // level, z up
  CHECK(!feed(0,2800,2900,true)); CHECK(TiltGuard::lastAngleDeg()<50);
  bool tripped=false; uint32_t start=ms;
  while(ms-start<config::tilt::HOLD_MS-20) tripped|=feed(0,0,-4096,true);   // upside down
  CHECK(!tripped && TiltGuard::lastAngleDeg()>170);
  for(int i=0;i<10;++i) tripped|=feed(0,0,-4096,true);   // filter: ~50 ms to cross the limit
  CHECK(tripped);
  // 11/10: rocking or vibration (short dips under the limit) no longer restarts the hold.
  TiltGuard::begin(); for(int i=0;i<20;++i) feed(0,0,4096,false);
  start=ms; tripped=false;
  while(ms-start<config::tilt::HOLD_MS+200) {
    const bool bump=((ms-start)/10)%10==0;                 // one raw level sample in ten
    tripped|=bump ? feed(0,0,4096,true) : feed(0,0,-4096,true);
  }
  CHECK(tripped);
  // Back under the limit for longer than RELEASE_MS restarts the hold; stale IMU never trips.
  TiltGuard::begin(); for(int i=0;i<20;++i) feed(0,0,4096,false);
  start=ms; while(ms-start<config::tilt::HOLD_MS-200) CHECK(!feed(4096,0,200,true));   // ~87 deg
  start=ms; while(ms-start<config::tilt::RELEASE_MS+200) CHECK(!feed(0,0,4096,true));
  start=ms; while(ms-start<config::tilt::HOLD_MS-200) CHECK(!feed(4096,0,200,true));
  imu.dataFresh=false; start=ms; while(ms-start<5000) CHECK(!feed(0,0,-4096,true));
  // Mounting does not matter: level learned with x up, then flipped.
  imu.dataFresh=true; TiltGuard::begin(); for(int i=0;i<20;++i) feed(4096,0,0,false);
  start=ms; tripped=false; while(ms-start<config::tilt::HOLD_MS+150) tripped|=feed(-4096,0,0,true);
  CHECK(tripped);
}

void demo() {
  fixture(); RobotFsm::begin(); SensorSnapshot sensors;
  const char* states[]={"Boot","WaitStart","Search","TargetConfirm","TrackAlign",
    "Approach","ContactCheck","AttackCharge","DiveBrake","SpokeInsert","UndercutDrive",
    "Push","ScoreHold","Backoff","LostTarget","WallEscape","ZoneEscape","StuckRecovery",
    "EmergencyStop","Stop","Wander","Opening","CircleBypass"};
  std::cout<<"time_ms,state,left,right (permille)\n";
  for(uint32_t time=0;time<=1600;time+=20) {
    VisionSnapshot vision;
    if(time>=100 && time<900) vision=target(static_cast<uint8_t>(time/20),time,960,time<300?300:600);
    const auto drive=RobotFsm::update(time,sensors,vision,running());
    const auto motors=DriveLogic::makeCommand(time,drive);
    if(time%100==0 || RobotFsm::state()==RobotState::DiveBrake)
      std::cout<<time<<","<<states[static_cast<uint8_t>(RobotFsm::state())]<<","<<motors.leftPermille<<","<<motors.rightPermille<<"\n";
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc>1 && std::string(argv[1])=="--demo") { demo(); return 0; }
    struct Test { const char* name; void (*run)(); };
    const Test tests[]={{"motor limit/watchdog/rollover",testMotor},{"motor straight trim/NVS",testMotorTrim},
      {"buttons/ESTOP/race",testButtonsAndEstop},{"two-button soft reset (11/11)",testSoftReset},{"boot mode: bench or match (11/11)",testBootMode},{"system start/fault",testSystem},
      {"UART loss/duplicates/reboot/fuzz",testProtocol},{"FSM attack/hazards/loss",testFsm},{"wander/tilt guard",testWanderAndTilt},{"align-only FSM",testAlignFsm},
      {"color sampling/errors/bounded retry",testColorsAndErrors},{"three independent samples",testThreeSamples},
      {"continuous single-exposure floor mode",testContinuousFloorMode},
      {"12-step calibration/NVS",testCalibration},{"nonblocking logs/LED priority",testLogsAndLeds},
      {"power-on battery check",testBattery},
      {"run log to the Pi",testRunLog},{"reset report to Pi (11/13)",testResetReport},{"HuskyLens link (old V1 robot)",testHuskyLens},{"log-only pose estimate",testPoseEstimate},{"real main loop integration",testMainIntegration},
      {"I2C time/optional sensors/boot ESTOP",testRealisticTimingAndBoot}};
    for(const auto& test:tests) { test.run(); ++groups; std::cout<<"PASS "<<test.name<<"\n"; }
    std::cout<<"MODEL "<<COLOR_SENSOR_MODEL<<": "<<groups<<" groups, "<<assertions<<" checks passed\n";
  } catch(const std::exception& error) { std::cerr<<"FAIL "<<error.what()<<"\n"; return 1; }
}
