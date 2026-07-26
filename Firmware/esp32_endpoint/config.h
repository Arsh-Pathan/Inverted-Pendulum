#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ─── Serial Communication Settings ───
#define SERIAL_BAUD_RATE      115200
#define DEFAULT_TELEMETRY_HZ  100
#define MAX_TELEMETRY_HZ      1000

// ─── AS5600 Magnetic Encoder (I2C) Pin Mapping ───
#define PIN_I2C_SDA           22
#define PIN_I2C_SCL           23
#define AS5600_I2C_ADDR       0x36
#define AS5600_REG_ANGLE_H    0x0E
#define AS5600_REG_ANGLE_L    0x0F

// ─── TB6612FNG Motor Driver Pin Mapping ───
#define PIN_MOTOR_AIN1        25
#define PIN_MOTOR_AIN2        26
#define PIN_MOTOR_PWMA        27
#define PIN_MOTOR_STBY        33

// ─── ESP32 LEDC PWM Settings ───
#define LEDC_TIMER_BIT_RES    8        // 8-bit resolution (0-255 PWM duty cycle)
#define LEDC_BASE_FREQ_HZ     1000     // 1 kHz PWM frequency for smooth motor actuation
#define LEDC_PWM_CHANNEL      0        // ESP32 LEDC Channel 0

// ─── Safety and Deadband Limits ───
#define MIN_PWM_POWER         45       // Minimum duty cycle to overcome static motor friction
#define MAX_PWM_POWER         255      // Maximum duty cycle

#endif // CONFIG_H
