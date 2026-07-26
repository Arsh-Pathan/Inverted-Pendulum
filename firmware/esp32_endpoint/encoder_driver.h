#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include "config.h"

class EncoderDriver {
public:
    EncoderDriver();
    
    /**
     * Initializes the I2C bus and verifies AS5600 connectivity.
     * Returns true if sensor device responds at 0x36.
     */
    bool begin();
    
    /**
     * Reads raw 12-bit angle from AS5600 registers (0-4095).
     */
    uint16_t readRawBits();
    
    /**
     * Reads raw uncalibrated angle in degrees [0.0, 360.0).
     */
    float readRawAngleDeg();
    
    /**
     * Returns calibrated angle in degrees relative to tare zero offset [0.0, 360.0).
     */
    float getCalibratedAngle();
    
    /**
     * Samples the current stationary position and sets it as the zero equilibrium offset.
     * Returns the calibrated zero offset in degrees.
     */
    float tare(int samples = 50);
    
    /**
     * Directly set the zero offset in degrees.
     */
    void setZeroOffset(float offset_deg);
    
    /**
     * Get current zero offset in degrees.
     */
    float getZeroOffset() const;

private:
    AS5600 _sensor;
    float _zeroOffsetDeg;
    bool _initialized;
};

#endif // ENCODER_DRIVER_H
