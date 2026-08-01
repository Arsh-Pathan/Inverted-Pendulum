# Inverted Pendulum Hardware-in-the-Loop (HIL) & Control System

A high-performance **Hardware-in-the-Loop (HIL) Control & Simulation Platform** for single-axis inverted pendulums. Features physical modeling, classical control strategies (PID, LQR, Hybrid Energy Swing-Up), Reinforcement Learning environment (Gymnasium / PPO / SAC), real-time serial telemetry, and a PyQt6 control panel.

---

## 🏛️ Codebase Structure

The project has been cleaned and consolidated into modular packages:

```
Inverted-Pendulum/
├── algorithm/               # Core Control, Environment & GUI Package
│   ├── comms/              # Serial communication protocol & hardware interface
│   ├── gui/                # PyQt6 Desktop GUI & Real-time Telemetry Canvas
│   ├── math/               # Mathematical algorithms & control dynamics
│   │   ├── controllers/    # PID, LQR, Energy Swing-up & Hybrid controllers
│   │   ├── core/           # Telemetry state representation & parsing
│   │   └── envs/           # Gymnasium-compatible simulation environment
│   ├── utils/              # Data logging, CSV storage & configuration utilities
│   └── main.py             # Main entry point for PyQt6 Control Station
├── config/                 # Platform default settings & serial configuration
│   └── default_config.json
├── docs/                   # Mathematical derivations & research documentation
│   ├── math/               # Equations of motion, LQR, energy & RL formulation
│   └── research_paper_formulation.md
├── firmware/               # Microcontroller (ESP32 / Arduino) source code
│   └── esp32_firmware.ino
├── models/                 # Pre-trained RL policies (PPO / SAC weights)
├── rl/                     # Reinforcement Learning training & evaluation scripts
│   ├── evaluate_policy.py  # Run trained policy in simulation
│   ├── train_ppo.py        # PPO training pipeline
│   └── train_sac.py        # SAC training pipeline
├── scripts/                # Diagnostic tools & CLI utilities
│   ├── benchmark_controllers.py
│   ├── calibrate_sensor.py
│   ├── record_telemetry.py
│   └── run_cli_balancer.py
└── tests/                  # Pytest automated test suite
    ├── test_controllers.py
    ├── test_env.py
    ├── test_logger.py
    ├── test_protocol.py
    ├── test_rl.py
    └── test_state.py
```

---

## 🧩 Architectural Component Breakdown

### 1. `algorithm/comms/` (Serial Communication)
- [`protocol.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/comms/protocol.py): Formats outgoing binary/string commands (`M,<pwm>`, `B`, `C`) sent over serial to the ESP32.
- [`serial_client.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/comms/serial_client.py): Handles background thread serial I/O, port auto-detection, and fast non-blocking frame ingestion.

### 2. `algorithm/math/controllers/` (Control Laws)
- [`pid_balancer.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/math/controllers/pid_balancer.py): Implements discrete-time PID control with exponential moving average (EMA) derivative filtering, deadband compensation, and dead-reckoning cart recentering.
- [`lqr_balancer.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/math/controllers/lqr_balancer.py): State-feedback regulator ($u = -Kx$) designed using LQR synthesis on linearized cart-pole dynamics.
- [`swing_up.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/math/controllers/swing_up.py): Energy-pumping controller that injects energy into the system based on total mechanical energy deviation from upright potential energy ($E_{\text{target}} = mgl$).
- [`hybrid_balancer.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/math/controllers/hybrid_balancer.py): Finite-state machine that executes energy swing-up when hanging down and seamlessly hands over to PID/LQR when entering the upright capture basin ($\pm 20^\circ$).

### 3. `algorithm/math/envs/` (Gymnasium Simulation)
- [`inverted_pendulum_env.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/math/envs/inverted_pendulum_env.py): Non-linear equations of motion solver (RK4/Euler integration) equipped with realistic AS5600 encoder quantization, DC motor back-EMF dynamics, and rail limit constraints. Also supports dual-mode Hardware-in-the-Loop execution.

### 4. `algorithm/gui/` & `algorithm/main.py` (Desktop Control Station)
- PyQt6-based control panel ([`main_window.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/gui/main_window.py)) providing live angle/velocity plotting ([`telemetry_canvas.py`](file:///C:/Users/ArshPathan/Projects/Hardware/Inverted-Pendulum/algorithm/gui/telemetry_canvas.py)), parameter tuning controls, and interactive hardware testing.

---

## ⚡ Quick Start

### Running the Control Station GUI
```bash
python algorithm/main.py
```

### Running Automated Unit Tests
```bash
pytest -m "not hardware"
```

### Running CLI Hardware Balancer
```bash
python scripts/run_cli_balancer.py --controller pid
```

### Training Reinforcement Learning Policy (PPO)
```bash
python rl/train_ppo.py --timesteps 100000
```
