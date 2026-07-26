# ESP32 Modular Hardware-in-the-Loop (HIL) Endpoint Firmware

This directory contains the industrial-grade, modular C++ firmware for the ESP32 microcontroller powering the **Inverted Pendulum HIL Platform**.

---

## 🏛️ Modular C++ Architecture

To maintain maximum code readability and clean separation of concerns, the firmware is structured into modular C++ classes:

*   **`config.h`**: Centralized definitions for GPIO pin mappings, I2C addresses, LEDC PWM frequencies/resolutions, and default baud rates.
*   **`encoder_driver.h` / `.cpp`**: Encapsulates 400 kHz Fast I2C communication with the AS5600 12-bit magnetic encoder. Handles raw bit extraction, angle conversion, and zero-equilibrium tare calibration.
*   **`motor_driver.h` / `.cpp`**: Manages the TB6612FNG dual H-bridge driver. Configures ESP32 LEDC hardware PWM timers (1 kHz, 8-bit resolution) and implements directional control, coasting, and hard dynamic braking.
*   **`serial_parser.h` / `.cpp`**: High-speed, non-blocking USB serial command parser. Processes ASCII commands in microseconds without stalling sensor streaming.
*   **`esp32_endpoint.ino`**: The clean 50-line main sketch connecting the drivers and scheduling high-frequency microsecond telemetry streaming.

---

## ⚡ Dual ESP32 Arduino Core v2.x / v3.0+ API Support

In major release v3.0.0 (based on ESP-IDF v5.x), Espressif introduced breaking changes to the LEDC hardware PWM API by removing `ledcSetup` and `ledcAttachPin`, and changing `ledcWrite` to accept a GPIO pin number instead of a channel ID.

The `MotorDriver` class in this firmware automatically detects your toolchain's ESP32 Core version via `<esp_arduino_version.h>` preprocessor macros and adapts compilation on the fly:
*   **On ESP32 Core v3.0+ (ESP-IDF v5.x):** Automatically utilizes `ledcAttach(pin, freq, resolution)` and pin-targeted `ledcWrite(pin, duty)`.
*   **On Legacy Core v2.x (ESP-IDF v4.x):** Seamlessly falls back to `ledcSetup(channel, freq, resolution)` and `ledcAttachPin(pin, channel)`.

This guarantees 100% compilation compatibility across modern Arduino IDE 2.x and legacy PlatformIO environments without manual code edits.

---

## 📌 Hardware Pin Mapping

| Component | Signal | ESP32 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **AS5600 Sensor** | I2C SDA | **GPIO 21** | 400 kHz Fast I2C Data |
| **AS5600 Sensor** | I2C SCL | **GPIO 22** | 400 kHz Fast I2C Clock |
| **TB6612FNG Driver**| AIN1 | **GPIO 25** | Direction control line 1 |
| **TB6612FNG Driver**| AIN2 | **GPIO 26** | Direction control line 2 |
| **TB6612FNG Driver**| PWMA | **GPIO 27** | LEDC Channel 0 PWM (0-255) |
| **TB6612FNG Driver**| STBY | **GPIO 33** | Standby enable (Active HIGH) |

---

## 🔌 Serial Command API (115200 Baud)

When connected via USB Serial, the firmware responds to the following newline-terminated commands:

*   `M,<power>`: Apply signed motor voltage in range `[-255, +255]` (e.g. `M,150` or `M,-200`).
*   `F,<speed>`: Spin motor forward at PWM duty cycle `[0, 255]`.
*   `R,<speed>`: Spin motor reverse at PWM duty cycle `[0, 255]`.
*   `B` or `S`: Hard dynamic short-circuit brake on the motor H-bridge.
*   `C`: Open-circuit free-wheel coasting.
*   `T,<rate_hz>`: Change telemetry streaming rate dynamically (`0` to `1000` Hz).
*   `Q`: Query an immediate single angle float reading.
*   `Z`: Execute on-demand tare calibration (measures 100 samples over 1 second and sets hanging zero offset).

---

## 🚀 Building and Uploading

1. Open the Arduino IDE or PlatformIO.
2. Select **ESP32 Dev Module** (or equivalent ESP32 board).
3. Ensure baud rate is set to **115200**.
4. Compile and upload `esp32_endpoint.ino`.
5. Keep the pendulum hanging completely motionless during the ~5.5 second bootup window (2.5s settling + 3s averaging) after reset to establish high-stability hanging zero calibration!
