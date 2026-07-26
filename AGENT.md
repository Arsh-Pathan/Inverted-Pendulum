# AGENT.md — Engineering & Architecture Roadmap for Autonomous AI Assistants

This document serves as the canonical system manifest, architectural specification, and operational rulebook for AI agents (e.g., Google Antigravity, Copilot, Cursor) contributing to the **Inverted Pendulum HIL Platform**.

---

## 🏗️ Core Architectural Invariants (DO NOT VIOLATE)

### 1. Python-Hosted Hardware-in-the-Loop (HIL) Separation
*   **ESP32 Firmware (`firmware/esp32_endpoint/`) is Stateless:** The embedded firmware MUST NOT contain any closed-loop control mathematics (no PID, LQR, or neural networks). It functions strictly as a low-latency I/O server streaming AS5600 12-bit magnetic encoder telemetry over USB CDC serial (115200 baud) and applying TB6612FNG H-bridge PWM commands (`[-255, +255]`).
*   **Python Control Station (`python/`) is Closed-Loop Host:** All estimation, noise filtering (EMA), geometry math, and control laws run on the Python PC host inside a thread-safe asynchronous serial loop (`QThread`). Target round-trip control loop latency is $< 2.0\text{ ms}$.

### 2. Geometry & Coordinate Conventions
*   **Upright Equilibrium:** Vertical upright equilibrium is defined as $\theta = 180.0^\circ$ ($\pi$ rad).
*   **Shortest-Path Angle Wrapping:** Angular error $e_\theta = 180.0^\circ - \theta_{raw}$ MUST always be wrapped into the shortest path range $[-180.0^\circ, +180.0^\circ]$ using `while e > 180.0: e -= 360.0; while e < -180.0: e += 360.0`.
*   **Lower Hemisphere Inversion:** When the pendulum drops below horizontal ($\theta \in [0^\circ, 90^\circ) \cup (270^\circ, 360^\circ]$), control sign MUST be inverted relative to upright stabilization to prevent positive feedback acceleration into the ground.

### 3. Actuator Deadband & Motor Safety
*   **Stiction Deadband:** Static friction prevents movement below PWM duty $\approx 45$. All controllers MUST linearly map normalized output commands to $[45, 255]$ via `speed = min_power + int(abs_out * (max_power - min_power) / 255.0)`.
*   **Hard Clamping:** Never emit motor commands outside integer range `[-255, +255]`.

---

## 📂 Repository Tree & Module Responsibility

```text
Inverted-Pendulum/
├── config/default_config.json       # Persisted serial, encoder, PID gains, & safety limits
├── docs/                            # Deep-dive architecture, wiring schematics, & research papers
├── firmware/esp32_endpoint/         # Modular C++ drivers (AS5600 I2C, TB6612 PWM, Serial Parser)
├── models/                          # FreeCAD assembly and 3MF/STL/G-code 3D print assets
├── python/                          # Python HIL Control Station
│   ├── comms/                       # Thread-safe SerialClient & protocol formatters (M, F, R, B, C, T, Q, Z)
│   ├── controllers/                 # Swappable engines: PIDBalancer, LQRBalancer, SwingUp, Hybrid, Oscillation
│   ├── core/                        # Type-safe PendulumState dataclass & event definitions
│   ├── envs/                        # InvertedPendulumEnv(gym.Env) supporting both HIL USB & non-linear Sim
│   └── gui/                         # Premium CAD aesthetic PyQt6 / PyQtGraph dashboard & cards
├── rl/                              # Reinforcement Learning Suite
│   ├── rl_controller.py             # RLBalancer: deploys trained .zip policies into HIL serial loop
│   ├── train_ppo.py                 # Proximal Policy Optimization (PPO) training script
│   ├── train_sac.py                 # Soft Actor-Critic (SAC) continuous training script
│   └── evaluate_policy.py           # Multi-episode evaluation & trajectory benchmarking CLI
├── scripts/                         # Research CLI tools (benchmark_controllers, record_telemetry, calibrate)
└── tests/                           # 19 comprehensive automated unit & integration tests
```

---

## 🛠️ Operational Command Reference for AI Agents

When developing, debugging, or verifying changes, always utilize the following standard verification commands:

### 1. Run Automated Test Suite (MANDATORY BEFORE COMMITS)
Always ensure 100% test pass rate across protocol formatters, state wrapping, control laws, RL envs, and loggers:
```bash
python -m unittest discover tests -v
```

### 2. Verify Non-Linear Control Laws (Offline Physics Sim)
Benchmark settling time, peak error, and energy expenditure across PID, LQR, and Hybrid controllers:
```bash
python scripts/benchmark_controllers.py
```

### 3. Reinforcement Learning Workflows
```bash
# Train PPO in offline non-linear simulation for 50,000 steps
python rl/train_ppo.py --timesteps 50000 --save-path rl/models/ppo_pendulum.zip

# Train SAC in offline simulation
python rl/train_sac.py --timesteps 50000 --save-path rl/models/sac_pendulum.zip

# Evaluate a trained checkpoint (returns mean survival time and MSE)
python rl/evaluate_policy.py --model rl/models/ppo_pendulum.zip --algo PPO --episodes 10
```

### 4. Hardware-in-the-Loop (HIL) CLI Diagnostics
```bash
# Verify USB serial endpoints and command response
python scripts/test_serial_endpoints.py

# Record 10 seconds of high-speed sensor telemetry to CSV
python scripts/record_telemetry.py --duration 10.0

# Run headless closed-loop HIL balancer in terminal
python scripts/run_cli_balancer.py
```

---

## 📋 Git Workflow & Commit Convention

AI contributors MUST adhere to **Conventional Commits** formatting to maintain a clean git history:
*   `feat: ...` — New features, algorithms, or CLI utilities.
*   `fix: ...` — Bug fixes or sign inversion corrections.
*   `refactor: ...` — Code restructuring without behavior changes.
*   `docs: ...` — Documentation, README, or research paper updates.
*   `test: ...` — Adding or updating test suites.
*   `chore: ...` — Dependency updates or build config maintenance.

**Rule:** Every commit MUST be atomic, self-contained, and accompanied by a passing unit test run (`python -m unittest discover tests -v`).
