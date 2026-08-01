# Reinforcement Learning (RL) Autonomous Control Suite

This package provides a professional, end-to-end Reinforcement Learning framework for training, evaluating, and deploying deep neural network policies on the **Inverted Pendulum Platform**.

---

## 🏛️ Architecture & MDP Formulation

By moving the control loop from the ESP32 microcontroller into Python, the physical robotic hardware is directly modeled as a continuous Markov Decision Process (MDP) via our Gymnasium wrapper (`InvertedPendulumEnv`):

*   **State Space ($\mathcal{S} \subset \mathbb{R}^2$):** Continuous observation vector $s_t = [e_\theta, \dot{\theta}]^T$, where $e_\theta \in [-\pi, \pi]$ is the shortest-path angular error from upright equilibrium and $\dot{\theta}$ is angular velocity in rad/s.
*   **Action Space ($\mathcal{A} \subset \mathbb{R}$):** Continuous normalized motor voltage command $a_t \in [-1.0, +1.0]$, linearly mapped to H-bridge PWM duties `[-255, +255]` with stiction deadband compensation.
*   **Reward Function ($\mathcal{R}$):**
    $$R(s_t, a_t) = - \left( e_{\theta, t}^2 + 0.1 \cdot \dot{\theta}_t^2 + 0.001 \cdot a_t^2 \right)$$
    Penalizes angular deviation and velocity while encouraging smooth, low-energy motor actuation.

---

## 📂 Package Overview

*   **`rl_controller.py` (`RLBalancer`)**: Subclass of `BaseController`. Bridges trained neural networks directly into the real-time HIL QThread control loop.
*   **`train_ppo.py`**: Complete training suite for **Proximal Policy Optimization (PPO)** using `Stable-Baselines3`. Features automatic TensorBoard logging and periodic evaluation checkpoints.
*   **`train_sac.py`**: Complete training suite for **Soft Actor-Critic (SAC)**, an off-policy maximum entropy algorithm tailored for continuous robotics tasks.
*   **`evaluate_policy.py`**: Benchmarking tool that runs multi-episode evaluation runs (in simulation or hardware USB mode) and outputs survival times and MSE metrics.

---

## 🚀 Training & Deployment Quickstart

### 1. Install Dependencies
```bash
pip install stable-baselines3[extra] torch torchvision gymnasium
```

### 2. Train an RL Policy (Simulation Mode)
Train PPO for 50,000 steps without needing the hardware connected:
```bash
python rl/train_ppo.py --timesteps 50000 --save-path rl/models/ppo_pendulum.zip
```
Or train Soft Actor-Critic (SAC):
```bash
python rl/train_sac.py --timesteps 50000 --save-path rl/models/sac_pendulum.zip
```
Both scripts also write per-step transition data to CSV under `rl/training_data/`.
Override the destination with `--data-path`, for example:
```bash
python rl/train_ppo.py --timesteps 50000 --data-path rl/training_data/ppo_run_001.csv
```

### 3. Train Directly on Physical Hardware (HIL Mode)
Plug in the ESP32 via USB and train live on the physical cart-pole:
```bash
python rl/train_ppo.py --timesteps 20000 --hil-port COM3 --save-path rl/models/ppo_hardware.zip
```

### 4. Evaluate Trained Policy
Benchmark 10 evaluation episodes:
```bash
python rl/evaluate_policy.py --model rl/models/ppo_pendulum.zip --algo PPO --episodes 10
```
