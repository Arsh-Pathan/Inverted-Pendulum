import unittest
import sys
import os
import shutil
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.math.core.state import PendulumState
from algorithm.utils.data_logger import TelemetryLogger

class TestTelemetryLogger(unittest.TestCase):
    """Test suite for high-speed CSV telemetry recording and file cleanup."""

    def setUp(self):
        self.test_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "test_logs_tmp"))
        os.makedirs(self.test_dir, exist_ok=True)

    def tearDown(self):
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_csv_logging_lifecycle(self):
        logger = TelemetryLogger(log_dir=self.test_dir)
        filepath = logger.start_recording("test_telemetry.csv")
        self.assertTrue(os.path.exists(filepath))
        self.assertTrue(logger.is_recording)

        # Log 5 test states
        for i in range(5):
            state = PendulumState(timestamp=time.time(), raw_angle=180.0 + i, angle_dev=180.0 + i, velocity=float(i), control_output=100)
            logger.log_state(state)

        count = logger.stop_recording()
        self.assertEqual(count, 5)
        self.assertFalse(logger.is_recording)

        # Verify file contents
        with open(filepath, "r", encoding="utf-8") as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 6) # 1 header + 5 data rows
            self.assertIn("error_from_upright", lines[0])
            self.assertIn("100", lines[1])

if __name__ == "__main__":
    unittest.main()
