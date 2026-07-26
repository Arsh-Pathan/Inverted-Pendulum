#include "serial_parser.h"

SerialParser::SerialParser(EncoderDriver& encoder, MotorDriver& motor, uint16_t& telemetryRateHz)
    : _encoder(encoder), _motor(motor), _telemetryRateHz(telemetryRateHz) {
    _buffer.reserve(32);
}

void SerialParser::poll() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_buffer.length() > 0) {
                _processCommand(_buffer);
                _buffer = "";
            }
        } else {
            if (_buffer.length() < 30) {
                _buffer += c;
            }
        }
    }
}

void SerialParser::_processCommand(const String& cmd_raw) {
    String cmd = cmd_raw;
    cmd.trim();
    if (cmd.length() == 0) return;

    char type = cmd.charAt(0);

    if (type == 'M') {
        int idx = cmd.indexOf(',');
        if (idx != -1) {
            int power = cmd.substring(idx + 1).toInt();
            _motor.setPower(power);
        }
    } else if (type == 'F') {
        int idx = cmd.indexOf(',');
        if (idx != -1) {
            int speed = cmd.substring(idx + 1).toInt();
            _motor.forward(speed);
        }
    } else if (type == 'R') {
        int idx = cmd.indexOf(',');
        if (idx != -1) {
            int speed = cmd.substring(idx + 1).toInt();
            _motor.reverse(speed);
        }
    } else if (type == 'B' || type == 'S') {
        _motor.brake();
    } else if (type == 'C') {
        _motor.coast();
    } else if (type == 'T') {
        int idx = cmd.indexOf(',');
        if (idx != -1) {
            int rate = cmd.substring(idx + 1).toInt();
            _telemetryRateHz = constrain(rate, 0, MAX_TELEMETRY_HZ);
            Serial.printf("[TELEMETRY] Rate set to %u Hz\n", _telemetryRateHz);
        }
    } else if (type == 'Q') {
        float angle = _encoder.getCalibratedAngle();
        Serial.println(angle, 2);
    } else if (type == 'Z') {
        _motor.coast();
        delay(50);
        float offset = _encoder.tare(40);
        Serial.printf("[CALIBRATED] Tare complete. Offset: %.2f deg\n", offset);
    } else {
        Serial.printf("[ERROR] Unknown command: %s\n", cmd.c_str());
    }
}
