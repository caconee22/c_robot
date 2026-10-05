#pragma once

#include <Arduino.h>

namespace config {
namespace sensors {

// Change only this line when building for the other color sensor board.
#define COLOR_SENSOR_AS7341 1
#define COLOR_SENSOR_TCS34725 2
#ifndef COLOR_SENSOR_MODEL
#define COLOR_SENSOR_MODEL COLOR_SENSOR_AS7341
#endif

// These are safe starting values. Final timing must be measured on the robot.
constexpr uint32_t COLOR_SENSOR_SLOT_PERIOD_MS = 1;
constexpr uint8_t COLOR_SENSOR_VISITS_PER_UPDATE =
    COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341 ? 4 : 1;

constexpr uint8_t TCA9548A_ADDRESS = 0x70;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 5;
constexpr uint32_t CONVERSION_TIMEOUT_MARGIN_MS = 100;
constexpr uint32_t SMUX_TIMEOUT_MS = 30;
constexpr uint8_t COLOR_SENSOR_COUNT = 4;

// Robot front is up: 1=front-left, 2=front-right,
// 3=rear-left, 4=rear-right.
constexpr uint8_t COLOR_SENSOR_TCA_CHANNELS[COLOR_SENSOR_COUNT] = {0, 1, 2, 3};

// ---------------------------------------------------------------------------
// Color sensor exposure settings. Change these values in this file only.
// ---------------------------------------------------------------------------
enum class As7341Gain : uint8_t {
  X0_5,
  X1,
  X2,
  X4,
  X8,
  X16,
  X32,
  X64,
  X128,
  X256,
  X512,
};

enum class Tcs34725Gain : uint8_t {
  X1,
  X4,
  X16,
  X60,
};

enum class Tcs34725Integration : uint8_t {
  Ms2_4 = 0xFF,
  Ms24 = 0xF6,
  Ms50 = 0xEB,
  Ms101 = 0xD6,
  Ms154 = 0xC0,
  Ms614 = 0x00,
};

// AS7341 integration time is approximately
// (ATIME + 1) * (ASTEP + 1) * 2.78 us.
constexpr uint32_t AS7341_EXPOSURE_US = 5000;  // Edit this, not ATIME/ASTEP.
constexpr uint8_t AS7341_INTEGRATION_ATIME = 0;
constexpr uint16_t AS7341_INTEGRATION_ASTEP =
    (AS7341_EXPOSURE_US * 100UL) / 278UL - 1UL;
constexpr As7341Gain AS7341_GAIN = As7341Gain::X16;
// Applies only to modules with white LED wired to AS7341 LDR.
constexpr bool AS7341_LED_ENABLED = true;
constexpr uint16_t AS7341_LED_CURRENT_MA[COLOR_SENSOR_COUNT] = {20, 20, 20, 20};
constexpr uint8_t AS7341_AUTOZERO_INTERVAL = 255;  // Before first sample only.
constexpr uint16_t MINIMUM_CLEAR_COUNTS = 1;
static_assert(AS7341_EXPOSURE_US >= 6 && AS7341_EXPOSURE_US <= 182000,
              "Invalid AS7341 exposure");
static_assert(COLOR_SENSOR_VISITS_PER_UPDATE >= 1 &&
              COLOR_SENSOR_VISITS_PER_UPDATE <= COLOR_SENSOR_COUNT,
              "Invalid scan count");

constexpr Tcs34725Integration TCS34725_INTEGRATION =
    Tcs34725Integration::Ms2_4;
constexpr Tcs34725Gain TCS34725_GAIN = Tcs34725Gain::X16;

// A color becomes reliable only after this many consecutive identical reads.
constexpr uint8_t COLOR_CONFIRM_COUNT = 3;
static_assert(COLOR_CONFIRM_COUNT >= 3, "At least three color samples required");

constexpr uint32_t AS7341_INTEGRATION_MS =
    ((AS7341_INTEGRATION_ATIME + 1ULL) * (AS7341_INTEGRATION_ASTEP + 1ULL) *
         278ULL + 99999ULL) / 100000ULL;
constexpr uint32_t TCS34725_INTEGRATION_MS =
    ((256UL - static_cast<uint8_t>(TCS34725_INTEGRATION)) * 24UL + 9UL) / 10UL;
constexpr uint32_t SAMPLE_STALE_MS =
    (COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
         ? AS7341_INTEGRATION_MS
         : TCS34725_INTEGRATION_MS) + 250;
static_assert(AS7341_INTEGRATION_ASTEP < 65535, "ASTEP 65535 is reserved");
static_assert(AS7341_INTEGRATION_ATIME != 0 || AS7341_INTEGRATION_ASTEP != 0,
              "AS7341 ATIME and ASTEP cannot both be zero");
static_assert(TCA9548A_ADDRESS >= 0x70 && TCA9548A_ADDRESS <= 0x77,
              "Invalid TCA9548A address");

// Fallback thresholds used when no NVS calibration exists. These are only
// starting values and must be adjusted with real floor measurements.
constexpr uint16_t FALLBACK_AS7341_BLACK_CLEAR_BY_SENSOR[COLOR_SENSOR_COUNT] =
    {180, 180, 180, 180};
constexpr uint16_t FALLBACK_TCS34725_BLACK_CLEAR_BY_SENSOR[COLOR_SENSOR_COUNT] =
    {120, 120, 120, 120};
constexpr float FALLBACK_RED_DOMINANCE = 1.20f;
constexpr float FALLBACK_BLUE_DOMINANCE = 1.18f;
constexpr float FALLBACK_YELLOW_OVER_BLUE = 1.30f;
constexpr float FALLBACK_YELLOW_RG_MIN = 0.55f;
constexpr float FALLBACK_YELLOW_RG_MAX = 1.80f;
constexpr float MATCH_MINIMUM_SCALE = 0.005f;
constexpr float MATCH_MAXIMUM_SCORE = 1.0f;
constexpr float MATCH_MINIMUM_WINNER_GAP = 0.08f;
constexpr float MATCH_COLOR_BRIGHTNESS_WEIGHT = 0.20f;
constexpr float MATCH_BLACK_BRIGHTNESS_WEIGHT = 0.55f;
static_assert(MATCH_COLOR_BRIGHTNESS_WEIGHT >= 0 && MATCH_COLOR_BRIGHTNESS_WEIGHT <= 1 &&
              MATCH_BLACK_BRIGHTNESS_WEIGHT >= 0 && MATCH_BLACK_BRIGHTNESS_WEIGHT <= 1,
              "Invalid brightness weights");

// Calibration console and measurement timing.
// LEVEL / LIFTED / PRESSED: guidance only, no distance sensor in V1.
constexpr uint8_t CALIBRATION_HEIGHT_MM[3] = {10, 20, 3};
constexpr uint32_t CALIBRATION_PREVIEW_PERIOD_MS = 200;
constexpr uint32_t CALIBRATION_COUNTDOWN_MS = 3000;
constexpr uint32_t CALIBRATION_MEASURE_MS = 3000;
constexpr uint16_t CALIBRATION_MIN_SAMPLES = 5;
constexpr uint16_t DEFAULT_RATIO_MARGIN_PERMILLE = 150;
constexpr uint16_t DEFAULT_BRIGHTNESS_MARGIN_PERMILLE = 300;

}  // namespace sensors
}  // namespace config
