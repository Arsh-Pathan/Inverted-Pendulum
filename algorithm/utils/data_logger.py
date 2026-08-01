import os
import csv
import time
from typing import Optional
from ..math.core.state import PendulumState

class TelemetryLogger:
    """
    High-speed CSV telemetry logger. Records PendulumState snapshots
    for offline system identification, FFT vibration analysis, and RL reward evaluation.
    """
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        self.file_path: Optional[str] = None
        self.file_handle = None
        self.writer = None
        self.is_recording = False
        self.sample_count = 0

    def start_recording(self, filename: Optional[str] = None) -> str:
        """Starts recording telemetry to a CSV file. Returns the absolute file path."""
        os.makedirs(self.log_dir, exist_ok=True)
        if not filename:
            timestamp_str = time.strftime("%Y%m%d_%H%M%S")
            filename = f"telemetry_{timestamp_str}.csv"
            
        self.file_path = os.path.abspath(os.path.join(self.log_dir, filename))
        try:
            self.file_handle = open(self.file_path, "w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.file_handle, fieldnames=[
                "timestamp", "raw_angle", "angle_dev", "velocity", 
                "error_from_upright", "control_output", "sample_rate_hz"
            ])
            self.writer.writeheader()
            self.file_handle.flush()
            self.is_recording = True
            self.sample_count = 0
            print(f"[LOGGER] Started telemetry recording: {self.file_path}")
            return self.file_path
        except Exception as e:
            print(f"[LOGGER ERROR] Failed to create log file {self.file_path}: {e}")
            self.is_recording = False
            return ""

    def log_state(self, state: PendulumState):
        """Writes a single state snapshot to CSV if recording is active."""
        if not self.is_recording or not self.writer:
            return
        try:
            self.writer.writerow(state.to_dict())
            self.sample_count += 1
            # Flush every 50 samples to prevent data loss without bogging down I/O
            if self.sample_count % 50 == 0 and self.file_handle:
                self.file_handle.flush()
        except Exception as e:
            print(f"[LOGGER ERROR] Write failed: {e}")

    def stop_recording(self) -> int:
        """Stops recording and closes the file. Returns total samples logged."""
        if not self.is_recording:
            return self.sample_count
        self.is_recording = False
        if self.file_handle:
            try:
                self.file_handle.flush()
                self.file_handle.close()
            except Exception as e:
                print(f"[LOGGER ERROR] Failed closing log file: {e}")
        print(f"[LOGGER] Recording stopped. Total samples saved: {self.sample_count}")
        return self.sample_count
