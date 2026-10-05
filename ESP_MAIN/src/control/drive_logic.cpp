#include "control/drive_logic.hpp"

#include "control/control_config.hpp"
#include "motor/motor_config.hpp"

namespace {

int16_t clampMotor(int32_t value) {
  return static_cast<int16_t>(constrain(
      value, -config::motor::COMMAND_MAX, config::motor::COMMAND_MAX));
}

void arcade(MotorCommand& command, int16_t throttle, int16_t steering) {
  command.leftPermille = clampMotor(static_cast<int32_t>(throttle) + steering);
  command.rightPermille =
      clampMotor(static_cast<int32_t>(throttle) - steering);
}

}  // namespace

MotorCommand DriveLogic::makeCommand(uint32_t nowMs,
                                     const DriveIntent& intent) {
  MotorCommand command;
  const int16_t duty = static_cast<int16_t>(min(
      intent.requestedDuty,
      static_cast<uint16_t>(config::motor::COMMAND_MAX)));
  command.stopMode = StopMode::Brake;
  command.validUntilMs = nowMs + config::control::COMMAND_VALID_MS;
  command.deadlineSet = true;

  switch (intent.type) {
    case DriveIntentType::Coast:
      command.stopMode = StopMode::Coast;
      break;
    case DriveIntentType::SearchLeft:
    case DriveIntentType::AlignLeft:
    case DriveIntentType::EscapeLeft:
      command.leftPermille = -duty;
      command.rightPermille = duty;
      break;
    case DriveIntentType::SearchRight:
    case DriveIntentType::AlignRight:
    case DriveIntentType::EscapeRight:
      command.leftPermille = duty;
      command.rightPermille = -duty;
      break;
    case DriveIntentType::Approach:
    case DriveIntentType::AttackCharge:
    case DriveIntentType::SpokeInsert:
    case DriveIntentType::Push:
      arcade(command, duty, intent.steering);
      break;
    case DriveIntentType::Reverse:
      arcade(command, -duty, intent.steering);
      break;
    case DriveIntentType::DiveBrake:
    case DriveIntentType::Brake:
    default:
      command.leftPermille = 0;
      command.rightPermille = 0;
      break;
  }
  return command;
}
