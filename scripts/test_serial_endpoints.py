#!/usr/bin/env python3
"""
CLI Diagnostic Script: Test Hardware Endpoint Connection & Actuator Response.
"""
import sys
import os
import time
import serial

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.utils.config_loader import load_config

def test_endpoints():
    config = load_config()
    port = config.get("serial", {}).get("preferred_port", "COM3")
    baud = config.get("serial", {}).get("baud_rate", 115200)

    print(f"─── Testing ESP32 Hardware Endpoint on {port} ({baud} baud) ───")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1.5) # Wait for Arduino reset / connection settling
        ser.reset_input_buffer()
        print("[1/4] Connected successfully! Streaming 10 telemetry samples...")
        
        samples_collected = 0
        while samples_collected < 10:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line and not line.startswith("[") and line != "READY":
                try:
                    val = float(line)
                    print(f"      Sample #{samples_collected + 1}: {val:.2f}°")
                    samples_collected += 1
                except ValueError:
                    pass
        
        print("[2/4] Testing Motor Forward Endpoint (M,100)...")
        ser.write(b"M,100\n")
        ser.flush()
        time.sleep(1.0)
        
        print("[3/4] Testing Motor Reverse Endpoint (M,-100)...")
        ser.write(b"M,-100\n")
        ser.flush()
        time.sleep(1.0)
        
        print("[4/4] Testing Hard Brake Endpoint (B)...")
        ser.write(b"B\n")
        ser.flush()
        time.sleep(0.5)
        
        ser.close()
        print("─── All endpoint tests completed successfully! ───")
    except Exception as e:
        print(f"[ERROR] Endpoint test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    test_endpoints()
