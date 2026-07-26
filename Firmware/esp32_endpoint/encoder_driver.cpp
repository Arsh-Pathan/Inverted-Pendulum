#include "encoder_driver.h"

EncoderDriver::EncoderDriver() : _zeroOffsetDeg(0.0f), _initialized(false) {}

bool EncoderDriver::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // 400 kHz Fast I2C Mode for minimum latency
    
    // Check if AS5600 responds on I2C bus
    Wire.beginTransmission(AS5600_I2C_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        _initialized = true;
        // Perform initial tare at boot up
        tare(20);
        return true;
    }
    return false;
}

uint16_t EncoderDriver::readRawBits() {
    if (!_initialized) return 0;
    
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(AS5600_REG_ANGLE_H);
    Wire.endTransmission(false);
    
    Wire.requestFrom(AS5600_I2C_ADDR, 2);
    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte = Wire.read();
        return ((uint16_t)highByte << 8 | lowByte) & 0x0FFF;
    }
    return 0;
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
        delay(2);
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
