# TB6612FNG & AS5600 Wiring Guide

This document outlines the physical wiring and connections between the **ESP32 Microcontroller**, the **AS5600 Magnetic Angle Encoder**, the **TB6612FNG Dual H-Bridge Motor Driver**, and the DC Gearmotor.

---

## 📌 Pinout Summary Table

| Device / Pin | ESP32 GPIO Pin | Description / Notes |
| :--- | :--- | :--- |
| **AS5600 SDA** | `GPIO 22` | I2C Data Line (with internal/external pull-up) |
| **AS5600 SCL** | `GPIO 23` | I2C Clock Line (with internal/external pull-up) |
| **TB6612FNG AIN1**| `GPIO 25` | Motor Direction Pin 1 |
| **TB6612FNG AIN2**| `GPIO 26` | Motor Direction Pin 2 |
| **TB6612FNG PWMA**| `GPIO 27` | Motor Speed (PWM Output @ 1 kHz) |
| **TB6612FNG STBY**| `GPIO 33` | Driver Standby Control (`HIGH` = Active, `LOW` = Standby/Sleep) |
| **TB6612FNG VCC** | `3.3V` | Logic Voltage Supply (MUST match ESP32 logic voltage) |
| **TB6612FNG GND** | `GND` | Common Ground (Tie ESP32 GND, TB6612 GND, and Motor Battery GND together) |
| **TB6612FNG VM**  | *External 6V–12V* | Motor Power Supply (Do **NOT** connect to ESP32 3.3V or 5V!) |
| **TB6612FNG AO1** | *Motor Term A* | H-Bridge Output 1 to DC Gearmotor |
| **TB6612FNG AO2** | *Motor Term B* | H-Bridge Output 2 to DC Gearmotor |

---

## ⚡ System Wiring Diagram

```
         ESP32 MICROCONTROLLER                   TB6612FNG DRIVER                 DC GEARMOTOR
       ┌───────────────────────┐             ┌──────────────────────┐           ┌──────────────┐
       │                       │             │                      │           │              │
       │                  3.3V ├────────────►│ VCC (Logic Power)    │           │              │
       │                       │             │                      │           │              │
       │                   GND ├────────────►│ GND (Logic & Motor)  │◄──────────┼─ Battery GND │
       │                       │             │                      │           │              │
       │               GPIO 25 ├────────────►│ AIN1 (Dir 1)         │           │              │
       │               GPIO 26 ├────────────►│ AIN2 (Dir 2)         │           │              │
       │               GPIO 27 ├────────────►│ PWMA (PWM Speed)     │           │              │
       │               GPIO 33 ├────────────►│ STBY (Standby HIGH)  │           │              │
       │                       │             │                      │           │              │
       │                       │             │                   VM │◄──────────┼─ Battery + (6-12V)
       │                       │             │                      │           │              │
       │                       │             │                  AO1 ├──────────►│ Terminal +   │
       │                       │             │                  AO2 ├──────────►│ Terminal -   │
       └───────────┬───────────┘             └──────────────────────┘           └──────────────┘
                   │
                   │ (I2C Bus @ 400 kHz)
                   │
       ┌───────────▼───────────┐
       │                       │
       │        AS5600         │
       │   MAGNETIC ENCODER    │
       │                       │
       │  SDA ◄────► GPIO 22   │
       │  SCL ◄────► GPIO 23   │
       │  VCC ◄────► 3.3V      │
       │  GND ◄────► GND       │
       └───────────────────────┘
```

> [!CAUTION]
> **NEVER** connect the motor battery voltage (**VM**, typically 6V to 12V) to the ESP32 `3.3V`, `5V`, or any GPIO pin. Doing so will permanently destroy the microcontroller. Ensure all ground lines (`GND`) from the ESP32, TB6612FNG, and external power supply are tied together into a single common ground reference.

---

## 🔄 Motor H-Bridge Truth Table

| AIN1 | AIN2 | PWMA Duty Cycle | H-Bridge State | Physical Cart Motion |
| :---: | :---: | :---: | :--- | :--- |
| `HIGH` | `LOW` | `1` .. `255` | **Forward** | Cart translates Left (or positive X direction) |
| `LOW` | `HIGH` | `1` .. `255` | **Reverse** | Cart translates Right (or negative X direction) |
| `HIGH` | `HIGH` | `0` | **Hard Brake** | Motor short-circuits to lock position immediately |
| `LOW` | `LOW` | `0` | **Coast** | Motor open-circuits and free-wheels with minimum resistance |

---

## 🛠️ Verification & Testing

To test the physical wiring before running the full control host:
1. Flash the `esp32_endpoint.ino` sketch onto the ESP32.
2. Open terminal or Arduino Serial Monitor at **115200 baud**.
3. Verify that calibrated angle telemetry floats are streaming at 100 Hz.
4. Type `M,100` and hit Enter $\rightarrow$ The motor should gently spin forward.
5. Type `M,-100` and hit Enter $\rightarrow$ The motor should spin reverse.
6. Type `B` and hit Enter $\rightarrow$ The motor should brake instantly.
