/**
 * Inverted Pendulum ESP32 Hardware-in-the-Loop (HIL) Endpoint v2.0
 * 
 * Modular C++ firmware for the ESP32 microcontroller. Acts as an ultra-fast,
 * low-latency I/O server that streams calibrated AS5600 12-bit angle telemetry
 * over USB Serial (115200 baud) and actuates the TB6612FNG H-bridge motor driver.
 * 
 * All closed-loop balancing algorithms (PID, LQR, Energy Swing-Up) reside on the
 * Python PC host for zero-recompile experimentation and Reinforcement Learning.
 */

#include <Arduino.h>
#include "config.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "serial_parser.h"

EncoderDriver encoder;
MotorDriver motor;
uint16_t telemetryRateHz = DEFAULT_TELEMETRY_HZ;
SerialParser parser(encoder, motor, telemetryRateHz);

unsigned long lastTelemetryUs = 0;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 2000); // Allow USB CDC serial settling

    // Initialize motor GPIO and LEDC PWM channels
    motor.begin();

    // Initialize I2C bus and AS5600 magnetic encoder
    bool encoderOk = encoder.begin();
    if (!encoderOk) {
        Serial.println("[WARNING] AS5600 sensor not detected on I2C bus! Check SDA (GPIO21) & SCL (GPIO22).");
    } else {
        Serial.println("[READY] Inverted Pendulum ESP32 HIL Endpoint v2.0 initialized successfully.");
    }
}

void loop() {
    // 1) Non-blockingly poll and execute incoming USB serial commands (M, F, R, B, C, T, Q, Z)
    parser.poll();

    // 2) Stream high-frequency ASCII angle telemetry at the configured rate
    if (telemetryRateHz > 0) {
        unsigned long intervalUs = 1000000UL / telemetryRateHz;
        unsigned long nowUs = micros();
        
        if (nowUs - lastTelemetryUs >= intervalUs) {
            lastTelemetryUs = nowUs;
            float angle = encoder.getCalibratedAngle();
            Serial.println(angle, 2);
        }
    }
}
