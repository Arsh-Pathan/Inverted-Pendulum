#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>
#include "encoder_driver.h"
#include "motor_driver.h"
#include "config.h"

class SerialParser {
public:
    SerialParser(EncoderDriver& encoder, MotorDriver& motor, uint16_t& telemetryRateHz);
    
    /**
     * Non-blocking poll method to process incoming characters on USB Serial.
     * Must be called frequently in the main loop().
     */
    void poll();

private:
    EncoderDriver& _encoder;
    MotorDriver& _motor;
    uint16_t& _telemetryRateHz;
    String _buffer;

    void _processCommand(const String& cmd);
};

#endif // SERIAL_PARSER_H
