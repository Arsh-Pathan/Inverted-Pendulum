#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
    MotorDriver();
    
    /**
     * Configures GPIO pins and initializes ESP32 LEDC PWM generator.
     */
    void begin();
    
    /**
     * Applies signed motor power (-255 to +255).
     * Positive = Forward, Negative = Reverse, 0 = Coast.
     */
    void setPower(int power);
    
    /**
     * Spins motor forward at specified PWM duty cycle (0-255).
     */
    void forward(int speed);
    
    /**
     * Spins motor reverse at specified PWM duty cycle (0-255).
     */
    void reverse(int speed);
    
    /**
     * Applies hard short-circuit dynamic braking to the H-bridge.
     */
    void brake();
    
    /**
     * Opens H-bridge circuitry for free-wheel coasting.
     */
    void coast();

private:
    int _currentPower;
};

#endif // MOTOR_DRIVER_H
