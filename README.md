# Inverted Pendulum: Real-Time Python Hardware-in-the-Loop (HIL) & Reinforcement Learning Platform

An advanced, high-precision **Hardware-in-the-Loop (HIL)** inverted pendulum research platform. This project integrates custom 3D-printed mechanical hardware, an ultra-low latency ESP32 hardware endpoint firmware, and a modular Python control station featuring classical controllers, modern state-feedback LQR, Reinforcement Learning (Gymnasium) environments, and a real-time PyQt6 / PyQtGraph engineering dashboard.

---

## 🌟 Major Architecture Overhaul (HIL Control in Python)

Unlike traditional embedded robotics projects where closed-loop control algorithms (PID) are hardcoded into microcontroller firmware in C++, this platform has been designed from the ground up as a **true Python-Hosted Hardware-in-the-Loop (HIL)** system:

1. **Zero-Latency Python Control Loop:** The ESP32 firmware acts purely as a high-speed sensor streaming and actuator driver endpoint. When calibrated 12-bit angle telemetry arrives over USB serial, the Python host executes the stabilization math and pushes the motor control voltage back to the hardware in **under 2 ms total round-trip time**.
2. **Multi-Algorithm Control Suite:** Easily switch between 5 interchangeable control engines:
   * **`PIDBalancer`**: Classical Proportional-Integral-Derivative with EMA noise filtering and linear stiction deadband mapping.
   * **`LQRBalancer`**: Modern Linear Quadratic Regulator state-feedback control ($u = -Kx$) linearized around vertical upright equilibrium ($180^\circ$).
   * **`SwingUpController`**: Energy-pumping controller based on the Åström-Furuta Lyapunov energy law for swinging a hanging pendulum up to vertical.
   * **`HybridBalancer`**: Autonomous controller that monitors system state and dynamically switches between `SwingUpController` (when hanging/falling) and precision stabilization (`PID`/`LQR`) when entering the $\pm 20^\circ$ upright capture basin.
   * **`OscillationController`**: Track rail and clearance diagnostics tester.
3. **Reinforcement Learning (Gymnasium) Native:** Includes `InvertedPendulumEnv(gym.Env)`, enabling continuous action/observation RL training via OpenAI Gym / Gymnasium. Supports both real USB hardware execution AND non-linear Euler numerical physics simulation for offline training without hardware!

---

## 📂 Repository Structure

```text
Inverted-Pendulum/
├── config/
│   └── default_config.json          # Consolidated JSON config for serial, encoder, PID gains, & limits
├── docs/
│   ├── architecture.md              # Deep-dive into HIL architecture, math formulation, & RL MDPs
│   ├── serial_protocol.md           # Full specification of ESP32 endpoint commands & ASCII telemetry
│   └── wiring_guide.md              # Pinout tables and wiring schematics for TB6612FNG & AS5600
├── firmware/
│   └── esp32_endpoint/
│       └── esp32_endpoint.ino       # Deterministic ESP32 hardware endpoint firmware (no balancing math)
├── models/                          # 3D CAD and slicing files for 3D printing
│   ├── cart/                        # Cart body STL, 3MF, and 0.2mm PLA G-code
│   ├── pendulum/                    # Pendulum rod/bob STL, 3MF, and G-code
│   └── assembly_model.FCStd         # Master FreeCAD assembly source file
├── python/                          # Modular Python Control Station Package
│   ├── main.py                      # Application launcher for the desktop PyQt6 dashboard
│   ├── core/                        # Type-safe PendulumState dataclass & system event definitions
│   ├── comms/                       # Thread-safe async serial reader & command protocol formatters
│   ├── controllers/                 # Swappable control engines (PID, LQR, SwingUp, Hybrid, Base)
│   ├── envs/                        # OpenAI Gym / Gymnasium RL environment wrapper (HIL & Sim modes)
│   ├── gui/                         # Rich CAD aesthetic UI cards, live telemetry viewport, & control panel
│   └── utils/                       # Configuration loader and CSV TelemetryLogger utilities
├── scripts/                         # Standalone CLI Utility & Research Scripts
│   ├── benchmark_controllers.py     # Offline non-linear physics simulation benchmark comparing controllers
│   ├── calibrate_sensor.py          # Interactive CLI tool to tare and measure AS5600 stability
│   ├── record_telemetry.py          # CLI recorder to capture live USB telemetry to timestamped CSV files
│   ├── run_cli_balancer.py          # Headless terminal HIL balancer (no GUI required)
│   └── test_serial_endpoints.py     # CLI verification script for hardware serial commands
├── tests/                           # Comprehensive automated unit & integration test suite
│   ├── test_protocol.py             # Verifies serial command string formatting and boundary clamping
│   ├── test_state.py                # Verifies shortest-path angle wrapping and hemisphere math
│   ├── test_controllers.py          # Verifies mathematical control laws, deadzones, and hybrid switching
│   ├── test_env.py                  # Verifies Gymnasium RL simulation step, reward, and truncation
│   └── test_logger.py               # Verifies CSV telemetry recording lifecycle and file formatting
├── requirements.txt                 # Python dependencies (PyQt6, pyqtgraph, pyserial, numpy, gymnasium)
├── .gitignore                       # Clean ignore rules for Python, Arduino/ESP32, logs, & IDEs
└── README.md                        # Project overview and quickstart guide
```

---

## 🚀 Quickstart Guide

### 1. Environment Setup
Install the required Python 3.10+ dependencies:
```bash
pip install -r requirements.txt
```

### 2. Flashing the Hardware Endpoint
1. Open Arduino IDE or PlatformIO and install the `AS5600` library via Library Manager.
2. Open `firmware/esp32_endpoint/esp32_endpoint.ino`, select your ESP32 board and COM port, and upload.
   * *Note: Keep the pendulum hanging motionless during the first 2 seconds of bootup to establish hanging zero equilibrium.*
   * *Compatibility: The firmware automatically detects and adapts to both legacy ESP32 Arduino Core v2.x and modern Core v3.0+ (ESP-IDF v5.x) LEDC PWM APIs on the fly.*

### 3. Launching the GUI Station
To start the full graphical HIL dashboard with real-time CAD animation and PyQtGraph plots:
```bash
python python/main.py
```
* **START OSCILLATION:** Runs back-and-forth track testing.
* **START AUTO-BALANCE (HIL):** Engages the closed-loop Python PID balancing engine. Manually raise the pendulum upright ($180^\circ$) to let the controller take over!
* **LIVE TUNING:** Adjust $K_P$, $K_I$, $K_D$, and EMA $\alpha$ spinboxes on the fly—settings automatically persist to `config/default_config.json`.
* **KEYBOARD OVERRIDE:** Press `A` (left) and `D` (right) for manual cart override when idle.

### 4. Running Standalone CLI Tools
For terminal-only operation without loading the GUI:
```bash
# Compare settling time and energy effort across PID, LQR, and Hybrid controllers in simulated physics
python scripts/benchmark_controllers.py

# Record 10 seconds of live hardware telemetry to a CSV file for FFT vibration or RL analysis
python scripts/record_telemetry.py --duration 10.0

# Test serial endpoints and actuator response
python scripts/test_serial_endpoints.py

# Re-tare sensor equilibrium offset
python scripts/calibrate_sensor.py

# Run headless closed-loop balancing directly in terminal
python scripts/run_cli_balancer.py
```

### 5. Running the Automated Test Suite
Verify that all mathematical control laws, protocol formatters, dataclasses, and RL environments are functioning properly:
```bash
python -m unittest discover tests -v
```

---

## 📚 Documentation Reference

For deeper technical dive into the algorithms, wiring, and serial API, consult the `docs/` directory:
* [Technical Research Paper Formulation & Lagrangian Mathematics](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/docs/research_paper_formulation.md)
* [Hardware-in-the-Loop Architecture & Algorithms](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/docs/architecture.md)
* [ESP32 Serial Endpoint Protocol Specification](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/docs/serial_protocol.md)
* [TB6612FNG & AS5600 Wiring Schematics](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/docs/wiring_guide.md)
* [AGENT.md — AI Contributor System Manifest](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/AGENT.md)
