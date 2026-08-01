#!/usr/bin/env python3
"""
CLI Diagnostic Script: Calibrate AS5600 Encoder Zero-Offset & Save to Config.
"""
import sys
import os
import time
import serial

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.utils.config_loader import load_config

def calibrate_sensor():
    config = load_config()
    port = config.get("serial", {}).get("preferred_port", "COM3")
    baud = config.get("serial", {}).get("baud_rate", 115200)

    print(f"─── AS5600 Zero-Offset Calibration Tool ({port}) ───")
    print("Please ensure the pendulum is hanging COMPLETELY STILL in its natural equilibrium position.")
    for i in range(3, 0, -1):
        print(f"Starting calibration in {i} seconds...")
        time.sleep(1.0)

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.0)
        ser.reset_input_buffer()

        print("Sending tare endpoint command (Z)...")
        ser.write(b"Z\n")
        ser.flush()
        
        # Read response logs
        start_time = time.time()
        while time.time() - start_time < 4.0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"  [ESP32] {line}")
                if "[CALIBRATED]" in line:
                    break

        print("Measuring post-tare zero stability (100 samples)...")
        readings = []
        while len(readings) < 100 and time.time() - start_time < 8.0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line and not line.startswith("[") and line != "READY":
                try:
                    readings.append(float(line))
                except ValueError:
                    pass

        ser.close()

        if readings:
            avg_angle = sum(readings) / len(readings)
            max_dev = max(abs(x - avg_angle) for x in readings)
            print(f"\nCalibration Complete! Average reading: {avg_angle:.3f}°, Max jitter: {max_dev:.3f}°")
            if max_dev < 0.2:
                print("Sensor stability: EXCELLENT.")
            else:
                print("Sensor stability: MODERATE (ensure pendulum is completely motionless).")
        else:
            print("[WARNING] Did not receive angle telemetry after calibration.")

    except Exception as e:
        print(f"[ERROR] Calibration failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    calibrate_sensor()
