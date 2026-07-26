#!/usr/bin/env python3
"""
CLI Evaluation Script: Benchmark a trained RL Neural Network Policy.
Runs multi-episode evaluations in simulation or hardware-in-the-loop and reports performance metrics.
"""
import sys
import os
import argparse
import math
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.envs.inverted_pendulum_env import InvertedPendulumEnv

try:
    from stable_baselines3 import PPO, SAC, TD3, A2C
    _SB3_AVAILABLE = True
except ImportError:
    _SB3_AVAILABLE = False

def evaluate_policy(model_path: str, algo: str = "PPO", episodes: int = 10, hil_port: str = None):
    if not _SB3_AVAILABLE:
        print("[ERROR] stable-baselines3 is required. Run: pip install stable-baselines3 torch")
        sys.exit(1)

    if not os.path.exists(model_path):
        print(f"[ERROR] Model checkpoint not found: {model_path}")
        print("Train a policy first using: python rl/train_ppo.py")
        sys.exit(1)

    print(f"─── Evaluating RL Policy: {algo} ({model_path}) ───")
    is_simulated = (hil_port is None)
    print(f"Mode: {'Numerical Simulation' if is_simulated else f'Live Hardware ({hil_port})'}")
    print(f"Test Episodes: {episodes}\n")

    if algo.upper() == "PPO":
        model = PPO.load(model_path)
    elif algo.upper() == "SAC":
        model = SAC.load(model_path)
    else:
        model = PPO.load(model_path)

    env = InvertedPendulumEnv(serial_port=hil_port, simulated=is_simulated, max_episode_steps=500)

    total_rewards = []
    survival_steps = []
    peak_errors = []

    for ep in range(1, episodes + 1):
        obs, _ = env.reset()
        ep_reward = 0.0
        peak_err = 0.0
        step_count = 0

        for step in range(500):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            step_count += 1
            peak_err = max(peak_err, abs(info["error_deg"]))
            
            if terminated or truncated:
                break

        total_rewards.append(ep_reward)
        survival_steps.append(step_count)
        peak_errors.append(peak_err)
        print(f"Episode {ep:2d} | Reward: {ep_reward:7.1f} | Survived: {step_count:3d} steps | Peak Error: {peak_err:5.1f}°")

    env.close()

    print("\n─── Evaluation Benchmark Summary ───")
    print(f"Mean Reward:        {np.mean(total_rewards):.1f} ± {np.std(total_rewards):.1f}")
    print(f"Mean Survival Time: {np.mean(survival_steps) * env.dt:.2f} seconds ({np.mean(survival_steps):.1f} steps)")
    print(f"Mean Peak Error:    {np.mean(peak_errors):.2f}°")
    print("─────────────────────────────────────\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate Trained RL Model")
    parser.add_argument("-m", "--model", type=str, required=True, help="Path to trained .zip model")
    parser.add_argument("-a", "--algo", type=str, default="PPO", help="Algorithm name (PPO, SAC)")
    parser.add_argument("-e", "--episodes", type=int, default=10, help="Number of test episodes")
    parser.add_argument("--hil-port", type=str, default=None, help="COM port for live hardware test")
    args = parser.parse_args()
    evaluate_policy(args.model, args.algo, args.episodes, args.hil_port)
