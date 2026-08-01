#!/usr/bin/env python3
"""
CLI Research Tool: Connect to ESP32 Hardware Endpoint & Log Live Telemetry to CSV.
"""
import sys
import os
import time
import argparse
import serial

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.utils.config_loader import load_config
from algorithm.utils.data_logger import TelemetryLogger
from algorithm.math.core.state import PendulumState

def record_telemetry(duration_sec: float = 10.0, output_file: str = None):
    config = load_config()
    port = config.get("serial", {}).get("preferred_port", "COM3")
    baud = config.get("serial", {}).get("baud_rate", 115200)

    print(f"─── Live Hardware Telemetry Recorder ({port} @ {baud} baud) ───")
    print(f"Target recording duration: {duration_sec:.1f} seconds...")

    logger = TelemetryLogger(log_dir="logs")
    filepath = logger.start_recording(output_file)
    if not filepath:
        sys.exit(1)

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1.5) # Wait for hardware reset
        ser.reset_input_buffer()
        print("Connected! Recording stream...")

        start_time = time.time()
        last_time = start_time
        prev_angle = None

        while time.time() - start_time < duration_sec:
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

            vel = 0.0
            if prev_angle is not None and dt > 0:
                delta = (angle - prev_angle) % 360.0
                if delta > 180.0: delta -= 360.0
                elif delta < -180.0: delta += 360.0
                vel = delta / dt
            prev_angle = angle

            state = PendulumState(
                timestamp=now,
                raw_angle=angle,
                angle_dev=angle % 360.0,
                velocity=vel,
                control_output=0,
                sample_rate_hz=1.0 / max(0.001, dt)
            )
            logger.log_state(state)

        ser.close()
        logger.stop_recording()
        print(f"\n─── Recording Complete! Saved to: {filepath} ───")

    except Exception as e:
        print(f"\n[ERROR] Recording interrupted: {e}")
        logger.stop_recording()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Log live telemetry from ESP32 to CSV")
    parser.add_argument("-d", "--duration", type=float, default=10.0, help="Recording duration in seconds")
    parser.add_argument("-f", "--file", type=str, default=None, help="Output CSV filename")
    args = parser.parse_args()
    record_telemetry(args.duration, args.file)
