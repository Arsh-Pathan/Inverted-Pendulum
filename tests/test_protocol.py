import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.comms.protocol import (cmd_motor, cmd_forward, cmd_reverse, 
                                   cmd_brake, cmd_coast, cmd_set_telemetry_rate, 
                                   cmd_query_angle, cmd_zero_tare)

class TestSerialProtocol(unittest.TestCase):
    """Test suite for serial endpoint command formatting and value clamping."""
    
    def test_cmd_motor_clamping(self):
        self.assertEqual(cmd_motor(150), "M,150\n")
        self.assertEqual(cmd_motor(-200), "M,-200\n")
        self.assertEqual(cmd_motor(0), "M,0\n")
        # Boundary clamping tests
        self.assertEqual(cmd_motor(500), "M,255\n")
        self.assertEqual(cmd_motor(-999), "M,-255\n")

    def test_cmd_forward_reverse_clamping(self):
        self.assertEqual(cmd_forward(100), "F,100\n")
        self.assertEqual(cmd_forward(-50), "F,0\n")
        self.assertEqual(cmd_forward(300), "F,255\n")

        self.assertEqual(cmd_reverse(120), "R,120\n")
        self.assertEqual(cmd_reverse(-10), "R,0\n")
        self.assertEqual(cmd_reverse(1000), "R,255\n")

    def test_static_commands(self):
        self.assertEqual(cmd_brake(), "B\n")
        self.assertEqual(cmd_coast(), "C\n")
        self.assertEqual(cmd_query_angle(), "Q\n")
        self.assertEqual(cmd_zero_tare(), "Z\n")

    def test_cmd_set_telemetry_rate(self):
        self.assertEqual(cmd_set_telemetry_rate(100), "T,100\n")
        self.assertEqual(cmd_set_telemetry_rate(0), "T,0\n")
        self.assertEqual(cmd_set_telemetry_rate(5000), "T,1000\n")
        self.assertEqual(cmd_set_telemetry_rate(-50), "T,0\n")

if __name__ == "__main__":
    unittest.main()
