#include "motor_driver.h"

MotorDriver::MotorDriver() : _currentPower(0) {}

void MotorDriver::begin() {
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Enable TB6612FNG H-bridge standby pin
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Configure ESP32 LEDC PWM timer
    ledcSetup(LEDC_PWM_CHANNEL, LEDC_BASE_FREQ_HZ, LEDC_TIMER_BIT_RES);
    ledcAttachPin(PIN_MOTOR_PWMA, LEDC_PWM_CHANNEL);

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
    ledcWrite(LEDC_PWM_CHANNEL, speed);
}

void MotorDriver::reverse(int speed) {
    speed = constrain(speed, 0, MAX_PWM_POWER);
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    ledcWrite(LEDC_PWM_CHANNEL, speed);
}

void MotorDriver::brake() {
    _currentPower = 0;
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    ledcWrite(LEDC_PWM_CHANNEL, MAX_PWM_POWER);
}

void MotorDriver::coast() {
    _currentPower = 0;
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    ledcWrite(LEDC_PWM_CHANNEL, 0);
}
