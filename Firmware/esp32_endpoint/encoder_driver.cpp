#include "encoder_driver.h"

EncoderDriver::EncoderDriver() : _zeroOffsetDeg(0.0f), _initialized(false) {}

bool EncoderDriver::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // 400 kHz Fast I2C Mode for minimum latency
    
    // Check if AS5600 responds on I2C bus
    Wire.beginTransmission(AS5600_I2C_ADDR);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        return false;
    }
    
    _sensor.begin(); // Initialize official AS5600 library
    _initialized = true;
    
    // Wait 2.5 seconds to let power rails stabilize and pendulum motionless settling
    Serial.println("[CALIBRATION] Waiting 2.5s for pendulum to settle before zeroing...");
    delay(2500);
    Serial.println("[CALIBRATION] Sampling hanging zero equilibrium (300 samples)...");
    tare(300);
    return true;
}

uint16_t EncoderDriver::readRawBits() {
    if (!_initialized) return 0;
    // Use official AS5600 library method for reliable register timing and hysteresis handling
    return _sensor.readAngle();
}

float EncoderDriver::readRawAngleDeg() {
    uint16_t bits = readRawBits();
    return (float)bits * (360.0f / 4096.0f);
}

float EncoderDriver::getCalibratedAngle() {
    float rawDeg = readRawAngleDeg();
    float angle = rawDeg - _zeroOffsetDeg;
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

float EncoderDriver::tare(int samples) {
    if (!_initialized || samples <= 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
        sum += readRawAngleDeg();
        delay(10); // 10ms delay per sample for deep noise attenuation and vibration averaging
    }
    _zeroOffsetDeg = sum / (float)samples;
    return _zeroOffsetDeg;
}

void EncoderDriver::setZeroOffset(float offset_deg) {
    _zeroOffsetDeg = offset_deg;
}

float EncoderDriver::getZeroOffset() const {
    return _zeroOffsetDeg;
}
