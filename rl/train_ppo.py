#!/usr/bin/env python3
"""
Professional Reinforcement Learning Training Script: Proximal Policy Optimization (PPO).
Trains an autonomous balancing neural network policy in simulated non-linear physics or live HIL mode.
"""
import sys
import os
import argparse
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.envs.inverted_pendulum_env import InvertedPendulumEnv

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import EvalCallback
    from stable_baselines3.common.monitor import Monitor
    _SB3_AVAILABLE = True
except ImportError:
    _SB3_AVAILABLE = False


def plot_training_curve(monitor_dir: str = "rl/logs", save_path: str = "rl/logs/training_curve.png"):
    """Render an episode-reward learning curve from the Monitor CSV logs."""
    try:
        import matplotlib
        matplotlib.use("Agg")  # Headless-safe backend
        import matplotlib.pyplot as plt
        from stable_baselines3.common.monitor import load_results
        from stable_baselines3.common.results_plotter import ts2xy
    except ImportError:
        print("[WARN] matplotlib not available; skipping training curve. Install with: pip install matplotlib")
        return

    try:
        results = load_results(monitor_dir)
    except Exception as exc:
        print(f"[WARN] Could not load Monitor results for plotting: {exc}")
        return

    if len(results) == 0:
        print("[WARN] No episodes recorded; skipping training curve.")
        return

    x, y = ts2xy(results, "timesteps")

    # Rolling mean to smooth the noisy per-episode rewards
    window = min(50, max(1, len(y) // 10))
    if window > 1:
        import numpy as np
        smoothed = np.convolve(y, np.ones(window) / window, mode="valid")
        x_smoothed = x[window - 1:]
    else:
        smoothed, x_smoothed = y, x

    plt.figure(figsize=(10, 6))
    plt.plot(x, y, alpha=0.3, color="tab:blue", label="Episode reward")
    plt.plot(x_smoothed, smoothed, color="tab:red", linewidth=2, label=f"Rolling mean (window={window})")
    plt.xlabel("Timesteps")
    plt.ylabel("Episode Reward")
    plt.title("PPO Training Curve — Inverted Pendulum")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=120)
    plt.close()
    print(f"Training curve saved to: {os.path.abspath(save_path)}")

def train_ppo(timesteps: int = 50000, lr: float = 3e-4, hil_port: str = None, save_path: str = "rl/models/ppo_pendulum.zip"):
    if not _SB3_AVAILABLE:
        print("[ERROR] stable-baselines3 and PyTorch are required for RL training.")
        print("Install them by running: pip install stable-baselines3[extra] torch gymnasium")
        sys.exit(1)

    print("─── Proximal Policy Optimization (PPO) Training Suite ───")
    is_simulated = (hil_port is None)
    mode_str = "Numerical Simulation (Offline)" if is_simulated else f"Hardware-in-the-Loop (USB Port: {hil_port})"
    print(f"Training Mode: {mode_str}")
    print(f"Total Timesteps: {timesteps:,} | Learning Rate: {lr}")

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    os.makedirs("rl/logs", exist_ok=True)

    # Initialize Environment
    env = InvertedPendulumEnv(serial_port=hil_port, simulated=is_simulated, max_episode_steps=500)
    env = Monitor(env, os.path.join("rl/logs", "train"))

    # Setup Evaluation Environment and Callback
    eval_env = InvertedPendulumEnv(simulated=True, max_episode_steps=500)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="rl/models/best_model",
        log_path="rl/logs",
        eval_freq=2000,
        deterministic=True,
        render=False
    )

    # Initialize PPO Policy (Multi-Layer Perceptron Neural Network)
    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=lr,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,      # Small entropy coefficient to encourage exploration
        verbose=1,
        tensorboard_log="rl/tensorboard_logs/"
    )

    print("\nStarting neural network policy optimization...")
    start_time = time.time()
    try:
        model.learn(total_timesteps=timesteps, callback=eval_callback)
    except KeyboardInterrupt:
        print("\n[INFO] Training interrupted by user. Saving current checkpoint...")

    duration = time.time() - start_time
    model.save(save_path)
    print(f"\n─── Training Complete in {duration:.1f} seconds! ───")
    print(f"Final model checkpoint saved to: {os.path.abspath(save_path)}")

    # Render the learning curve from the recorded Monitor episode logs
    print("\nGenerating training curve from episode reward logs...")
    plot_training_curve()
    
    # Quick verification episode
    print("\nRunning 1 verification episode with trained policy...")
    obs, _ = eval_env.reset()
    ep_reward = 0.0
    for step in range(500):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, _ = eval_env.step(action)
        ep_reward += reward
        if terminated or truncated:
            break
    print(f"Verification Episode Total Reward: {ep_reward:.2f} (Survived {step+1} steps)")
    env.close()
    eval_env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train PPO Neural Network for Inverted Pendulum")
    parser.add_argument("-t", "--timesteps", type=int, default=50000, help="Number of training timesteps")
    parser.add_argument("--lr", type=float, default=3e-4, help="Learning rate")
    parser.add_argument("--hil-port", type=str, default=None, help="COM port for live hardware training (default: simulated)")
    parser.add_argument("-s", "--save-path", type=str, default="rl/models/ppo_pendulum.zip", help="Output model path")
    args = parser.parse_args()
    train_ppo(args.timesteps, args.lr, args.hil_port, args.save_path)
