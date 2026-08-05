#!/usr/bin/env python3
"""
Simple Discrete Track-Constrained Inverted Pendulum Balancer.

Task:
- Tare/Zero initial hanging angle to 0.0°.
- Start pendulum at 0° (hanging down).
- Swing up and balance at 180° (upright) without high-frequency vibration/chatter.
- Constraint 1: Track limit (configurable, e.g. ±0.20 m small track or ±0.40 m longer track).
- Constraint 2: Only angle data is available.
- Constraint 3: Discrete actions ONLY (RIGHT, LEFT, STOP).

Usage:
  # Run in simulation mode (offline non-linear physics):
  python scripts/balance_discrete_track.py --sim --track-limit 0.40

  # Run on real hardware (ESP32 USB serial):
  python scripts/balance_discrete_track.py --port COM3 --track-limit 0.40 --stabilize-power 90
"""

import sys
import os
import time
import math
import argparse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from algorithm.math.controllers.discrete_balancer import DiscreteTrackBalancer, DiscreteAction
from algorithm.math.envs.inverted_pendulum_env import InvertedPendulumEnv
from algorithm.comms.protocol import cmd_motor, cmd_brake, cmd_coast, cmd_zero_tare

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def render_ascii_status(step: int, raw_angle: float, cal_angle: float, vel_deg_s: float, action: DiscreteAction, mode: str, cart_x: float = 0.0):
    """Prints a clean, real-time ASCII visualization of the pendulum and cart status."""
    act_str = {DiscreteAction.RIGHT: "➡️ RIGHT", DiscreteAction.LEFT: "⬅️ LEFT", DiscreteAction.STOP: "🛑 STOP"}.get(action, "STOP")
    
    theta_from_upright = (cal_angle - 180.0) % 360.0
    if theta_from_upright > 180.0:
        theta_from_upright -= 360.0

    bar_width = 21
    norm_idx = int((theta_from_upright + 180.0) / 360.0 * (bar_width - 1))
    norm_idx = max(0, min(bar_width - 1, norm_idx))
    bar = ["-"] * bar_width
    bar[norm_idx] = "O"
    vis_pendulum = "".join(bar)

    print(f"\rStep {step:4d} | Mode: {mode:<9} | Angle: {cal_angle:6.1f}° (Err: {theta_from_upright:+6.1f}°) | Vel: {vel_deg_s:+6.1f}°/s | Action: {act_str:<9} | Cart: {cart_x:+5.2f}m | [{vis_pendulum}]", end="", flush=True)

def run_hardware(balancer: DiscreteTrackBalancer, port: str, baud: int = 115200):
    """Runs the controller on physical hardware via USB serial."""
    if not SERIAL_AVAILABLE:
        print("[ERROR] pyserial package is not installed.")
        return

    print(f"─── Running Discrete Track Balancer (HARDWARE HIL MODE on {port}) ───")
    print("Connecting to serial port...")

    balancer.enable()
    sample_count = 0
    last_time = time.time()

    try:
        ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(1.0)
        ser.reset_input_buffer()

        # Send zero/tare hardware command
        print("Sending zero/tare command to hardware...")
        ser.write(cmd_zero_tare().encode("utf-8"))
        ser.flush()
        time.sleep(0.2)

        # Tare calibration: sample initial hanging rest position
        tare_samples = []
        start_tare = time.time()
        while time.time() - start_tare < 0.5:
            raw = ser.readline()
            if raw:
                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line and not line.startswith("[") and line != "READY":
                        tare_samples.append(float(line))
                except ValueError:
                    pass

        if tare_samples:
            initial_zero = sum(tare_samples) / len(tare_samples)
            balancer.tare(initial_zero)
            print(f"[TARE SUCCESS] Hanging zero angle calibrated to 0.0° (Hardware reading: {initial_zero:.2f}°)")
        else:
            balancer.tare(0.0)
            print("[TARE WARNING] No telemetry received during tare window. Defaulting zero to 0.0°.")

        print("Connected! Starting discrete anti-vibration control loop... (Press Ctrl+C to stop)\n")

        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line or line.startswith("[") or line == "READY":
                    continue
                raw_angle_deg = float(line)
            except ValueError:
                continue

            now = time.time()
            dt = now - last_time if last_time > 0 else 0.01
            last_time = now

            # Compute discrete action using ONLY raw_angle_deg
            act_enum, pwm_val = balancer.compute_action_enum(raw_angle_deg, dt)
            cal_angle_deg = (raw_angle_deg - balancer.zero_offset) % 360.0

            # Send motor command over serial
            cmd = cmd_motor(pwm_val)
            ser.write(cmd.encode("utf-8"))
            ser.flush()

            sample_count += 1
            if sample_count % 5 == 0:
                render_ascii_status(sample_count, raw_angle_deg, cal_angle_deg, balancer._filtered_vel, act_enum, balancer.active_mode, balancer.est_x)

    except KeyboardInterrupt:
        print("\n\n[STOP] User stopped control loop. Hard braking motor...")
        if 'ser' in locals() and ser.is_open:
            ser.write(cmd_brake().encode("utf-8"))
            ser.flush()
            ser.close()
    except Exception as e:
        print(f"\n\n[ERROR] Hardware loop exception: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.write(cmd_coast().encode("utf-8"))
            ser.close()


def main():
    parser = argparse.ArgumentParser(description="Discrete Track-Constrained Inverted Pendulum Balancer")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port for real hardware (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds")
    parser.add_argument("--power", type=int, default=255, help="Full PWM magnitude for swing-up / large recovery (0-255)")
    parser.add_argument("--stabilize-power", type=int, default=200, help="Reduced PWM magnitude for smooth upright balancing (0-255)")
    parser.add_argument("--track-limit", type=float, default=0.60, help="Track limit in meters from center (default: 0.40 m)")
    args = parser.parse_args()

    balancer = DiscreteTrackBalancer(
        pwm_power=args.power,
        stabilize_power=args.stabilize_power,
        track_limit=args.track_limit
    )

    if args.port:
        run_hardware(balancer, args.port, args.baud)
    else:
        print("Faild to access port.")


if __name__ == "__main__":
    main()
