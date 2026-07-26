#!/usr/bin/env python3
"""
Professional Reinforcement Learning Training Script: Soft Actor-Critic (SAC).
Trains an off-policy maximum entropy actor-critic neural network for continuous robotic control.
"""
import sys
import os
import argparse
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.envs.inverted_pendulum_env import InvertedPendulumEnv

try:
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import EvalCallback
    from stable_baselines3.common.monitor import Monitor
    _SB3_AVAILABLE = True
except ImportError:
    _SB3_AVAILABLE = False

def train_sac(timesteps: int = 50000, lr: float = 3e-4, hil_port: str = None, save_path: str = "rl/models/sac_pendulum.zip"):
    if not _SB3_AVAILABLE:
        print("[ERROR] stable-baselines3 and PyTorch are required for RL training.")
        print("Install them by running: pip install stable-baselines3[extra] torch gymnasium")
        sys.exit(1)

    print("─── Soft Actor-Critic (SAC) Continuous Control Training Suite ───")
    is_simulated = (hil_port is None)
    mode_str = "Numerical Simulation (Offline)" if is_simulated else f"Hardware-in-the-Loop (USB Port: {hil_port})"
    print(f"Training Mode: {mode_str}")
    print(f"Total Timesteps: {timesteps:,} | Learning Rate: {lr}")

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    os.makedirs("rl/logs", exist_ok=True)

    env = InvertedPendulumEnv(serial_port=hil_port, simulated=is_simulated, max_episode_steps=500)
    env = Monitor(env, "rl/logs")

    eval_env = InvertedPendulumEnv(simulated=True, max_episode_steps=500)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="rl/models/best_sac_model",
        log_path="rl/logs",
        eval_freq=2000,
        deterministic=True,
        render=False
    )

    model = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=lr,
        buffer_size=50000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        ent_coef="auto",
        verbose=1,
        tensorboard_log="rl/tensorboard_logs/"
    )

    print("\nStarting off-policy maximum entropy optimization...")
    start_time = time.time()
    try:
        model.learn(total_timesteps=timesteps, callback=eval_callback)
    except KeyboardInterrupt:
        print("\n[INFO] Training interrupted by user. Saving current checkpoint...")

    duration = time.time() - start_time
    model.save(save_path)
    print(f"\n─── Training Complete in {duration:.1f} seconds! ───")
    print(f"Final model checkpoint saved to: {os.path.abspath(save_path)}")
    env.close()
    eval_env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train SAC Neural Network for Inverted Pendulum")
    parser.add_argument("-t", "--timesteps", type=int, default=50000, help="Number of training timesteps")
    parser.add_argument("--lr", type=float, default=3e-4, help="Learning rate")
    parser.add_argument("--hil-port", type=str, default=None, help="COM port for live hardware training (default: simulated)")
    parser.add_argument("-s", "--save-path", type=str, default="rl/models/sac_pendulum.zip", help="Output model path")
    args = parser.parse_args()
    train_sac(args.timesteps, args.lr, args.hil_port, args.save_path)
