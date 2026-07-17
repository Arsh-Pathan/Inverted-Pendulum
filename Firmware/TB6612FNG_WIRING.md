# TB6612FNG Wiring Guide – Inverted Pendulum

## Pinout Summary

| TB6612FNG Pin | Connect To        | Notes                        |
|---------------|-------------------|------------------------------|
| **VM**        | Motor power (6–12V) | Matches your DC motor voltage |
| **VCC**       | ESP32 **3.3V**    | Logic supply                 |
| **GND**       | ESP32 **GND** + Motor GND | Common ground (all share) |
| **STBY**      | ESP32 **GPIO 33** | HIGH = driver active         |
| **AIN1**      | ESP32 **GPIO 25** | Direction pin 1              |
| **AIN2**      | ESP32 **GPIO 26** | Direction pin 2              |
| **PWMA**      | ESP32 **GPIO 27** | Speed (PWM)                  |
| **AO1**       | Motor terminal 1  | Motor wire A                 |
| **AO2**       | Motor terminal 2  | Motor wire B                 |

> [!IMPORTANT]
> - **VM** is the motor power supply (6V–12V depending on your motor). Do NOT connect this to 3.3V.
> - **VCC** is the logic supply — connect to ESP32's 3.3V pin.
> - All GNDs must be tied together (ESP32, TB6612FNG, motor power supply).

## Wiring Diagram

```
    ESP32                    TB6612FNG                  MOTOR
    ─────                    ─────────                  ─────
    3.3V  ──────────────────► VCC
    GND   ──────────────────► GND ◄─── Battery GND
                              VM  ◄─── Battery + (6-12V)

    GPIO 25 ────────────────► AIN1
    GPIO 26 ────────────────► AIN2
    GPIO 27 ────────────────► PWMA
    GPIO 33 ────────────────► STBY

                              AO1 ────────────────────► Motor +
                              AO2 ────────────────────► Motor -


    (Existing AS5600 I2C)
    GPIO 21 (SDA) ──────────► AS5600 SDA
    GPIO 22 (SCL) ──────────► AS5600 SCL
```

## Motor Control Truth Table

| AIN1 | AIN2 | PWMA    | Action     |
|------|------|---------|------------|
| HIGH | LOW  | PWM     | **Forward** (cart moves left)  |
| LOW  | HIGH | PWM     | **Reverse** (cart moves right) |
| HIGH | HIGH | 0       | **Brake** (motor locked)       |
| LOW  | LOW  | 0       | **Coast** (motor free-wheels)  |

## Serial Commands (from Python)

Send these over serial to control the motor:

| Command   | Action                          |
|-----------|---------------------------------|
| `L\n`     | Move cart LEFT at current speed |
| `R\n`     | Move cart RIGHT at current speed|
| `S\n`     | Brake (hard stop)               |
| `C\n`     | Coast (free-wheel)              |
| `P,150\n` | Set PWM speed to 150 (0–255)   |

## Quick Test (Arduino Serial Monitor)

1. Upload the firmware
2. Open Serial Monitor at **115200 baud**
3. You should see angle values streaming
4. Type `L` and hit Enter → motor spins one direction
5. Type `R` and hit Enter → motor spins other direction
6. Type `S` and hit Enter → motor brakes
7. Type `P,80` → lower speed, then `L` again to test
