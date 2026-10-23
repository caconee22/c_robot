#pragma once

#include <Arduino.h>

// Hardware version is chosen by the PlatformIO environment (-D HW_VERSION=2).
// Without the flag this is the V1 board. Never change V1 pins for V2 work.
#ifndef HW_VERSION
#define HW_VERSION 1
#endif

// Banned on V1 and V2 (AGENTS.md "하드웨어 기능 사용 금지"). These checks
// stop a build that turns the features on by configuration.
#if defined(BOARD_HAS_PSRAM)
#error "PSRAM is banned. Remove BOARD_HAS_PSRAM (V2 uses GPIO35 for motor EN)."
#endif
#if defined(CONFIG_SPIRAM_MODE_OCT) && CONFIG_SPIRAM_MODE_OCT
#error "Octal PSRAM memory_type is banned. Use board_build.arduino.memory_type = qio_qspi."
#endif
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "Native USB is banned. Keep ARDUINO_USB_CDC_ON_BOOT=0 and use UART0."
#endif

#if HW_VERSION == 2
#include "pins_v2.hpp"
#elif HW_VERSION == 1

namespace pins {

// Switches use INPUT_PULLUP and are active LOW.
constexpr gpio_num_t SAFETY_SWITCH = GPIO_NUM_15;  // SW1
constexpr gpio_num_t START_SWITCH = GPIO_NUM_16;   // SW2

// Raspberry Pi communication.
constexpr gpio_num_t UART1_TX = GPIO_NUM_17;
constexpr gpio_num_t UART1_RX = GPIO_NUM_18;

// AS7341 sensors through TCA9548A.
constexpr gpio_num_t I2C_SDA = GPIO_NUM_11;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_12;

// WS2812 status LEDs.
constexpr gpio_num_t RGB_LED_INTERNAL = GPIO_NUM_48;
constexpr gpio_num_t RGB_LED_EXTERNAL = GPIO_NUM_13;
#define PINS_HAS_EXTERNAL_RGB_LED 1
// UART0 is shared by COM, SiK telemetry, and debug output.
constexpr gpio_num_t UART0_TX = GPIO_NUM_43;
constexpr gpio_num_t UART0_RX = GPIO_NUM_44;

// BTS7960 motor driver 1: left motor.
constexpr gpio_num_t MOTOR1_LPWM = GPIO_NUM_42;
constexpr gpio_num_t MOTOR1_RPWM = GPIO_NUM_41;
constexpr gpio_num_t MOTOR1_LEN = GPIO_NUM_40;
constexpr gpio_num_t MOTOR1_REN = GPIO_NUM_39;

// BTS7960 motor driver 2: right motor.
constexpr gpio_num_t MOTOR2_LPWM = GPIO_NUM_38;
constexpr gpio_num_t MOTOR2_RPWM = GPIO_NUM_47;
constexpr gpio_num_t MOTOR2_LEN = GPIO_NUM_21;
constexpr gpio_num_t MOTOR2_REN = GPIO_NUM_14;

}  // namespace pins

#else
#error "Unsupported HW_VERSION (use 1 or 2)"
#endif
