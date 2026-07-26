# ESP32 Hardware Endpoint Serial Protocol

The ESP32 firmware in this repository operates as a **deterministic hardware I/O endpoint**. It exposes serial commands to control motor actuation and streams calibrated AS5600 magnetic encoder telemetry to the PC host.

---

## 📡 Communication Specs

*   **Baud Rate:** `115200` (configurable up to `921600`)
*   **Data Bits:** `8`, **Parity:** `None`, **Stop Bits:** `1` (`8N1`)
*   **Line Termination:** ASCII Newline (`\n`) or Carriage Return + Newline (`\r\n`)
*   **Telemetry Streaming Rate:** Default `100 Hz` (10 ms interval), configurable from `1 Hz` to `1000 Hz`.

---

## 📥 Host-to-Target Commands (PC $\rightarrow$ ESP32)

Send these commands over USB serial to control the physical hardware. Commands are evaluated immediately in the ESP32 main loop.

| Command Syntax | Parameters | Description | Example |
| :--- | :--- | :--- | :--- |
| `M,<power>` | `<power>`: Integer in range `[-255, +255]` | Directly set motor power and direction. Positive values spin forward, negative values spin reverse, and `0` coasts. | `M,150`<br>`M,-120`<br>`M,0` |
| `F,<speed>` | `<speed>`: Integer in range `[0, 255]` | Spin motor forward at specified PWM speed. | `F,200` |
| `R,<speed>` | `<speed>`: Integer in range `[0, 255]` | Spin motor reverse at specified PWM speed. | `R,200` |
| `B` or `S` | *None* | **Hard Brake:** Short-circuits the H-bridge motor terminals to stop cart motion immediately. | `B` |
| `C` | *None* | **Coast:** Open-circuits the H-bridge, allowing the motor and cart to free-wheel smoothly. | `C` |
| `T,<rate_hz>` | `<rate_hz>`: Integer in range `[0, 1000]` | Set the streaming frequency for sensor telemetry. Setting `0` pauses streaming. | `T,200`<br>`T,0` |
| `Q` | *None* | **Query Single Reading:** Requests an immediate single angle telemetry float from the encoder. | `Q` |
| `Z` | *None* | **Tare / Zero Calibrate:** Coasts motor and samples 500 readings over 2 seconds to set the hanging vertical zero reference offset. | `Z` |

---

## 📤 Target-to-Host Telemetry (ESP32 $\rightarrow$ PC)

When streaming is enabled (default `100 Hz`), the ESP32 outputs ASCII-formatted floating point numbers representing the current angular deviation in degrees:

```text
0.02
-0.15
1.42
12.85
```

### Telemetry Angle Conventions:
*   `0.00°`: Bottom hanging equilibrium (calibrated during bootup or via `Z` command).
*   `±180.00°`: Vertical inverted upright position (target balancing point).
*   **Resolution:** 12-bit magnetic encoder (`360° / 4096 counts ≈ 0.0879°` per LSB).

---

## ⚡ Latency & Timing Budget

In Hardware-in-the-Loop (HIL) operation over USB serial at `115200` baud:
*   **Telemetry Frame Size:** ~8 bytes (`"-179.50\r\n"`) $\rightarrow$ Transmission duration $\approx 0.7\text{ ms}$.
*   **Command Frame Size:** ~6 bytes (`"M,-150\n"`) $\rightarrow$ Transmission duration $\approx 0.5\text{ ms}$.
*   **Total Serial Round-Trip Time:** $\approx 1.2\text{ ms}$.
*   **Host Python Processing:** $\approx 0.1\text{ ms}$ (using optimized numpy / direct Qt signal callbacks).

This ensures total closed-loop control latency remains **under 2.0 ms**, easily satisfying Nyquist stability criteria for an inverted pendulum with a natural frequency of ~2–3 Hz.
