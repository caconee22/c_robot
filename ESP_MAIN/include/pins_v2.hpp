#pragma once

// V2 board pin map. Included only through pins.hpp when HW_VERSION == 2.
// Source: PCBV2/C_ROBOT.kicad_sch netlist, confirmed by the user 2026-10-14.
// Not yet verified on an assembled V2 board.
//
// Banned on this board (AGENTS.md): Wi-Fi, PSRAM/external flash, native USB.
// GPIO19/20 are the USB pins and are used here as motor current inputs.

#include <Arduino.h>

namespace pins {

// Switches use INPUT_PULLUP and are active LOW.
constexpr gpio_num_t SAFETY_SWITCH = GPIO_NUM_15;  // SW1
constexpr gpio_num_t START_SWITCH = GPIO_NUM_16;   // SW2

// Raspberry Pi communication.
constexpr gpio_num_t UART1_TX = GPIO_NUM_17;
constexpr gpio_num_t UART1_RX = GPIO_NUM_18;

// Main I2C bus: TCA9548A (spectral sensors on channels 0..3), MPU6050 IMU
// directly on the bus, and the auxiliary I2C connector J21.
constexpr gpio_num_t I2C_SDA = GPIO_NUM_11;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_12;

// Status LED. V2 has only the on-board WS2812; GPIO13 is IR6 now.
constexpr gpio_num_t RGB_LED_INTERNAL = GPIO_NUM_48;
#define PINS_HAS_EXTERNAL_RGB_LED 0

// UART0 is shared by COM, SiK telemetry, and debug output.
constexpr gpio_num_t UART0_TX = GPIO_NUM_43;
constexpr gpio_num_t UART0_RX = GPIO_NUM_44;

// BTS7960 motor driver 1: left motor (J9). L_EN and R_EN share one pin.
constexpr gpio_num_t MOTOR1_LPWM = GPIO_NUM_42;
constexpr gpio_num_t MOTOR1_RPWM = GPIO_NUM_41;
constexpr gpio_num_t MOTOR1_EN = GPIO_NUM_40;

// BTS7960 motor driver 2: right motor (J10). L_EN and R_EN share one pin.
constexpr gpio_num_t MOTOR2_LPWM = GPIO_NUM_47;
constexpr gpio_num_t MOTOR2_RPWM = GPIO_NUM_21;
// GPIO35 is wired to the octal PSRAM inside N16R8/N8R8 modules. If the V2
// bench test shows EN_2 glitching at power-up or not following the command,
// jumper EN_2 to GPIO38 (or GPIO39) and change only this line:
//   constexpr gpio_num_t MOTOR2_EN = GPIO_NUM_38;
// The SW1 ISR EN mask is computed from this constant, so nothing else changes.
constexpr gpio_num_t MOTOR2_EN = GPIO_NUM_35;

// Existing code drives L_EN and R_EN separately; on V2 they are one wire.
constexpr gpio_num_t MOTOR1_LEN = MOTOR1_EN;
constexpr gpio_num_t MOTOR1_REN = MOTOR1_EN;
constexpr gpio_num_t MOTOR2_LEN = MOTOR2_EN;
constexpr gpio_num_t MOTOR2_REN = MOTOR2_EN;

// BTS7960 current sense (IS) through 1 kOhm, 470 Ohm to GND. Analog input.
// Use: wall push and stall detection only. GPIO19/20 are ADC2.
constexpr gpio_num_t MOTOR1_L_IS = GPIO_NUM_1;
constexpr gpio_num_t MOTOR1_R_IS = GPIO_NUM_2;
constexpr gpio_num_t MOTOR2_L_IS = GPIO_NUM_20;
constexpr gpio_num_t MOTOR2_R_IS = GPIO_NUM_19;

// Analog IR inputs IR1..IR6 (J1..J4, J7, J8). Purpose still to be confirmed.
// GPIO13 is ADC2.
constexpr gpio_num_t IR_INPUTS[6] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6,
                                     GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_13};

// Battery voltage through 100 kOhm / 10 kOhm (x11). GPIO14 is ADC2.
constexpr gpio_num_t BATTERY_ADC = GPIO_NUM_14;

// Encoders (J13 left, J16 right). Wired but NOT used: never configure these
// pins. GPIO36/37 are also octal PSRAM pins on R8 modules.
constexpr gpio_num_t ENCODER1_A = GPIO_NUM_39;
constexpr gpio_num_t ENCODER1_B = GPIO_NUM_38;
constexpr gpio_num_t ENCODER1_C = GPIO_NUM_9;   // Through 6.8k/4.7k divider.
constexpr gpio_num_t ENCODER2_A = GPIO_NUM_37;
constexpr gpio_num_t ENCODER2_B = GPIO_NUM_36;
constexpr gpio_num_t ENCODER2_C = GPIO_NUM_10;  // Through 6.8k/4.7k divider.

static_assert(MOTOR2_EN != GPIO_NUM_36 && MOTOR2_EN != GPIO_NUM_37,
              "GPIO36/37 are octal PSRAM pins; use GPIO38 or GPIO39");
static_assert(MOTOR1_EN != GPIO_NUM_19 && MOTOR1_EN != GPIO_NUM_20 &&
                  MOTOR2_EN != GPIO_NUM_19 && MOTOR2_EN != GPIO_NUM_20,
              "GPIO19/20 are current-sense inputs");
static_assert(MOTOR1_EN != MOTOR2_EN, "Each motor needs its own EN pin");

}  // namespace pins
