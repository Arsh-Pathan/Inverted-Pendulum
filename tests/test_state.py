import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.math.core.state import PendulumState

class TestPendulumState(unittest.TestCase):
    """Test suite for PendulumState dataclass properties and angle wrapping."""

    def test_error_from_upright_wrapping(self):
        # 180.0° is exact upright vertical equilibrium -> error should be 0.0
        s1 = PendulumState(angle_dev=180.0)
        self.assertAlmostEqual(s1.error_from_upright, 0.0)

        # 185.0° is 5° tilted -> error should be -5.0°
        s2 = PendulumState(angle_dev=185.0)
        self.assertAlmostEqual(s2.error_from_upright, -5.0)

        # 175.0° is 5° tilted opposite -> error should be +5.0°
        s3 = PendulumState(angle_dev=175.0)
        self.assertAlmostEqual(s3.error_from_upright, 5.0)

        # 0.0° is hanging straight down -> error should be 180.0° (or -180.0°)
        s4 = PendulumState(angle_dev=0.0)
        self.assertTrue(abs(s4.error_from_upright) == 180.0)

        # 350.0° (-10° hanging) -> error from 180° is -170.0°
        s5 = PendulumState(angle_dev=350.0)
        self.assertAlmostEqual(s5.error_from_upright, -170.0)

    def test_theta_from_upright_is_negation_of_error(self):
        """theta is the canonical control coordinate: theta == -error_from_upright."""
        for ad in (0.0, 90.0, 170.0, 175.0, 180.0, 185.0, 190.0, 350.0):
            s = PendulumState(angle_dev=ad)
            self.assertAlmostEqual(s.theta_from_upright, -s.error_from_upright, places=6)
            self.assertTrue(-180.0 <= s.theta_from_upright <= 180.0)

        # Upright is zero; leaning to larger angle_dev is POSITIVE theta.
        self.assertAlmostEqual(PendulumState(angle_dev=180.0).theta_from_upright, 0.0)
        self.assertAlmostEqual(PendulumState(angle_dev=185.0).theta_from_upright, 5.0)
        self.assertAlmostEqual(PendulumState(angle_dev=175.0).theta_from_upright, -5.0)

    def test_hemisphere_and_basin_checks(self):
        # Upright (180°) is above horizontal and in capture basin
        upright = PendulumState(angle_dev=180.0)
        self.assertTrue(upright.is_above_horizontal)
        self.assertTrue(upright.is_near_upright)

        # Hanging down (0°) is below horizontal and not near upright
        hanging = PendulumState(angle_dev=0.0)
        self.assertFalse(hanging.is_above_horizontal)
        self.assertFalse(hanging.is_near_upright)

        # 165° is above horizontal and within ±20° capture basin (error = 15°)
        tilted = PendulumState(angle_dev=165.0)
        self.assertTrue(tilted.is_above_horizontal)
        self.assertTrue(tilted.is_near_upright)

        # 150° is above horizontal but outside ±20° capture basin (error = 30°)
        far_tilted = PendulumState(angle_dev=150.0)
        self.assertTrue(far_tilted.is_above_horizontal)
        self.assertFalse(far_tilted.is_near_upright)

    def test_to_dict_serialization(self):
        s = PendulumState(timestamp=123.456, raw_angle=180.12, angle_dev=180.12, velocity=-10.5, control_output=150)
        d = s.to_dict()
        self.assertEqual(d["control_output"], 150)
        self.assertEqual(d["velocity"], "-10.50")
        self.assertEqual(d["angle_dev"], "180.12")

if __name__ == "__main__":
    unittest.main()
