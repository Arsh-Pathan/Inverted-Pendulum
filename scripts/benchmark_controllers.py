#!/usr/bin/env python3
"""
CLI Research Tool: Offline Physics Benchmark comparing PID, LQR, and Hybrid Controllers.
Simulates non-linear inverted pendulum physics and reports settling time, peak error, and energy usage.
"""
import sys
import os
import time
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.envs.inverted_pendulum_env import InvertedPendulumEnv
from python.controllers import PIDBalancer, LQRBalancer, HybridBalancer
from python.core.state import PendulumState

def run_benchmark(controller, name: str, init_error_deg: float = 15.0, steps: int = 500):
    env = InvertedPendulumEnv(simulated=True)
    # Reset with custom perturbation
    env.reset()
    env.state = np_array([math.radians(init_error_deg), 0.0])
    
    controller.enable()
    total_energy = 0.0
    peak_error = abs(init_error_deg)
    settling_step = -1
    
    for step in range(1, steps + 1):
        err_deg = math.degrees(env.state[0])
        vel_deg_s = math.degrees(env.state[1])
        
        # Convert to PendulumState (180 deg is upright in our controllers)
        state = PendulumState(angle_dev=180.0 - err_deg, velocity=-vel_deg_s)
        pwm_cmd = controller.compute_action_from_state(state, env.dt)
        
        # Step simulation (normalized action -1..1)
        norm_act = pwm_cmd / 255.0
        env.step([norm_act])
        
        abs_err = abs(math.degrees(env.state[0]))
        peak_error = max(peak_error, abs_err)
        total_energy += abs(norm_act)
        
        if abs_err < 1.0 and abs(vel_deg_s) < 5.0 and settling_step == -1:
            settling_step = step

    settling_time_s = (settling_step * env.dt) if settling_step != -1 else float('inf')
    return {
        "name": name,
        "peak_error": peak_error,
        "settling_time": settling_time_s,
        "total_energy": total_energy
    }

def np_array(val):
    import numpy as np
    return np.array(val, dtype=np.float32)

def main():
    print("─── Inverted Pendulum Offline Physics Benchmark ───")
    print("Simulating 5.0s disturbance recovery from 15.0° initial tilt...\n")
    
    controllers = [
        (PIDBalancer(kp=20.0, ki=0.5, kd=3.0), "PID Balancer"),
        (LQRBalancer(k_theta=25.0, k_omega=4.0), "LQR Balancer"),
        (HybridBalancer(capture_angle_deg=25.0), "Hybrid Balancer")
    ]
    
    results = []
    for ctrl, name in controllers:
        res = run_benchmark(ctrl, name, init_error_deg=15.0, steps=500)
        results.append(res)
        
    print(f"{'Controller Name':<18} | {'Peak Error':<12} | {'Settling Time':<14} | {'Control Effort':<14}")
    print("-" * 64)
    for r in results:
        st_str = f"{r['settling_time']:.2f} s" if r['settling_time'] != float('inf') else '> 5.00 s (Unsettled)'
        print(f"{r['name']:<18} | {r['peak_error']:6.2f}°     | {st_str:<14} | {r['total_energy']:8.1f} units")
    print("-" * 64)
    print("\nBenchmark completed successfully.")

if __name__ == "__main__":
    main()
