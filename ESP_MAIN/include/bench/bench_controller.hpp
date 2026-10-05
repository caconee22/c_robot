#pragma once
#include "robot_types.hpp"

enum class BenchMode : uint8_t { Stop, Motor, Manual, Data, Align, Follow, Avoid };

class BenchController {
 public:
  static void begin();
  static void select(BenchMode mode);
  static BenchMode mode();
  static const char* modeName(BenchMode mode);
  static bool setManual(int left, int right, uint32_t nowMs, bool allowed);
  static MotorCommand update(uint32_t nowMs, bool allowed,
                             const SensorSnapshot& sensors,
                             const VisionSnapshot& vision);
  static bool finished();
  static uint8_t hazardMask(const SensorSnapshot& sensors);
};
