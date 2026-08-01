#!/usr/bin/env python3
"""
Headless CLI HIL Balancer: Run Closed-Loop PID Balancing in Terminal without GUI.
"""
import sys
import os
import time
import serial

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.utils.config_loader import load_config
from algorithm.math.controllers.pid_balancer import PIDBalancer
from algorithm.comms.protocol import cmd_motor, cmd_brake, cmd_coast

def run_cli_balancer():
    config = load_config()
    port = config.get("serial", {}).get("preferred_port", "COM3")
    baud = config.get("serial", {}).get("baud_rate", 115200)
    ctrl_cfg = config.get("control", {})

    balancer = PIDBalancer(
        kp=ctrl_cfg.get("kp", 20.0),
        ki=ctrl_cfg.get("ki", 0.0),
        kd=ctrl_cfg.get("kd", 2.5),
        alpha=ctrl_cfg.get("alpha", 0.45),
        min_power=ctrl_cfg.get("min_motor_power", 35),
        max_power=ctrl_cfg.get("max_motor_power", 255),
        deadzone_deg=ctrl_cfg.get("equilibrium_deadzone_deg", 0.0),
        deadzone_vel=ctrl_cfg.get("equilibrium_deadzone_vel", 0.0),
        k_cart_v=ctrl_cfg.get("k_cart_v", 150.0),
        k_cart_x=ctrl_cfg.get("k_cart_x", 200.0),
        cart_accel_max=ctrl_cfg.get("cart_accel_max", 6.0),
        cart_damping=ctrl_cfg.get("cart_damping", 7.5),
        dither_power=ctrl_cfg.get("dither_power", 0)
    )
    balancer.enable()

    print(f"─── Launching Headless HIL PID Balancer on {port} ───")
    print(f"Gains: KP={balancer.kp}, KI={balancer.ki}, KD={balancer.kd}, ALPHA={balancer.alpha}")
    print("Press Ctrl+C to stop balancing and brake the motor.")

    try:
        ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(1.0)
        ser.reset_input_buffer()

        last_time = time.time()
        sample_count = 0

        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line or line.startswith("[") or line == "READY":
                    continue
                angle = float(line)
            except ValueError:
                continue

            now = time.time()
            dt = now - last_time if last_time > 0 else 0.01
            last_time = now

            power = balancer.compute_action(angle, dt)
            cmd = cmd_motor(power)
            ser.write(cmd.encode("utf-8"))
            ser.flush()

            sample_count += 1
            if sample_count % 20 == 0:
                print(f"[HIL LOOP] Angle: {angle:+.2f}° | Action Power: {power:+d} | dt: {dt*1000:.1f} ms")

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt detected. Hard braking motor...")
        if 'ser' in locals() and ser.is_open:
            ser.write(cmd_brake().encode("utf-8"))
            ser.flush()
            ser.close()
    except Exception as e:
        print(f"\n[ERROR] HIL loop exception: {e}")
        if 'ser' in locals() and ser.is_open:
            ser.write(cmd_coast().encode("utf-8"))
            ser.close()

if __name__ == "__main__":
    run_cli_balancer()
