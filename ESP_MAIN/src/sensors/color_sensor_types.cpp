#include "sensors/color_sensor_types.hpp"

const char* sensorCornerName(uint8_t sensorIndex) {
  switch (static_cast<SensorCorner>(sensorIndex)) {
    case SensorCorner::FrontLeft: return "FRONT_LEFT";
    case SensorCorner::FrontRight: return "FRONT_RIGHT";
    case SensorCorner::RearLeft: return "REAR_LEFT";
    case SensorCorner::RearRight: return "REAR_RIGHT";
    default: return "UNKNOWN_CORNER";
  }
}

const char* floorColorName(FloorColor color) {
  switch (color) {
    case FloorColor::Red: return "RED";
    case FloorColor::Yellow: return "YELLOW";
    case FloorColor::Blue: return "BLUE";
    case FloorColor::Black: return "BLACK";
    case FloorColor::Unknown:
    default: return "UNKNOWN";
  }
}

const char* calibrationPoseName(CalibrationPose pose) {
  switch (pose) {
    case CalibrationPose::Level: return "LEVEL";
    case CalibrationPose::Lifted: return "LIFTED";
    case CalibrationPose::Pressed: return "PRESSED";
    default: return "UNKNOWN_POSE";
  }
}
