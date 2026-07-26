#include "motor_driver.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

MotorDriver::MotorDriver() : _currentPower(0) {}

// Helper function to handle PWM write differences between ESP32 Core v2.x and v3.x
inline void writeMotorPWM(int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // In ESP32 Arduino Core v3.0+, ledcWrite takes the GPIO pin number
    ledcWrite(PIN_MOTOR_PWMA, duty);
#else
    // In legacy ESP32 Arduino Core v2.x, ledcWrite takes the LEDC channel number
    ledcWrite(LEDC_PWM_CHANNEL, duty);
#endif
}

void MotorDriver::begin() {
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Enable TB6612FNG H-bridge standby pin
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Configure ESP32 LEDC PWM timer (compatible with both Core v2.x and Core v3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino Core v3.0+ API (ESP-IDF v5.x)
    ledcAttach(PIN_MOTOR_PWMA, LEDC_BASE_FREQ_HZ, LEDC_TIMER_BIT_RES);
#else
    // Legacy ESP32 Arduino Core v2.x API (ESP-IDF v4.x)
    ledcSetup(LEDC_PWM_CHANNEL, LEDC_BASE_FREQ_HZ, LEDC_TIMER_BIT_RES);
    ledcAttachPin(PIN_MOTOR_PWMA, LEDC_PWM_CHANNEL);
#endif

    coast();
}

void MotorDriver::setPower(int power) {
    power = constrain(power, -MAX_PWM_POWER, MAX_PWM_POWER);
    _currentPower = power;

    if (power == 0) {
        coast();
    } else if (power > 0) {
        forward(power);
    } else {
        reverse(-power);
    }
}

void MotorDriver::forward(int speed) {
    speed = constrain(speed, 0, MAX_PWM_POWER);
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    writeMotorPWM(speed);
}

void MotorDriver::reverse(int speed) {
    speed = constrain(speed, 0, MAX_PWM_POWER);
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    writeMotorPWM(speed);
}

void MotorDriver::brake() {
    _currentPower = 0;
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    writeMotorPWM(MAX_PWM_POWER);
}

void MotorDriver::coast() {
    _currentPower = 0;
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    writeMotorPWM(0);
}
