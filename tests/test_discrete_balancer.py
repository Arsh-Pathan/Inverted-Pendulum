"""
Unit tests for DiscreteTrackBalancer controller.
"""
import unittest
import math
from algorithm.math.controllers.discrete_balancer import DiscreteTrackBalancer, DiscreteAction
from algorithm.math.core.state import PendulumState

class TestDiscreteTrackBalancer(unittest.TestCase):
    def setUp(self):
        self.balancer = DiscreteTrackBalancer(
            pwm_power=255,
            stabilize_power=140,
            capture_angle_deg=25.0,
            k_omega=0.12,
            deadband=1.5
        )
        self.balancer.enable()

    def test_initialization_and_reset(self):
        self.assertTrue(self.balancer.enabled)
        self.assertEqual(self.balancer.active_mode, "SWING_UP")
        self.balancer.reset()
        self.assertEqual(self.balancer.active_mode, "SWING_UP")
        self.assertEqual(self.balancer._kick_counter, 0)

    def test_tare_calibration(self):
        self.assertEqual(self.balancer.zero_offset, 0.0)
        self.balancer.tare(12.5)
        self.assertEqual(self.balancer.zero_offset, 12.5)
        self.assertEqual(self.balancer._kick_counter, 0)

    def test_discrete_action_outputs(self):
        # Action at upright balance (180° raw) with 0 velocity should be STOP
        act, pwm = self.balancer.compute_action_enum(180.0, 0.01)
        self.assertEqual(act, DiscreteAction.STOP)
        self.assertEqual(pwm, 0)

    def test_anti_vibration_stabilize_power_scaling(self):
        self.balancer.active_mode = "STABILIZE"
        # Prime with many near-upright readings so filtered velocity stays very low
        for i in range(20):
            self.balancer.compute_action_enum(180.0, 0.01)
        # Gradually drift 0.1° per step -> 10°/s velocity (well under 60°/s threshold)
        for i in range(20):
            self.balancer.compute_action_enum(180.0 + i * 0.1, 0.01)
        
        # Small tilt with low velocity -> uses stabilize_power (140)
        act_small, pwm_small = self.balancer.compute_action_enum(182.0, 0.01)
        self.assertEqual(act_small, DiscreteAction.RIGHT)
        self.assertEqual(pwm_small, 140)

    def test_kick_start_from_dead_rest(self):
        # First call from hanging (0°) should trigger kick-start RIGHT
        act, pwm = self.balancer.compute_action_enum(0.0, 0.01)
        self.assertEqual(act, DiscreteAction.RIGHT)
        self.assertEqual(pwm, 255)
        self.assertEqual(self.balancer._kick_counter, 1)

    def test_mode_switching_and_hysteresis(self):
        self.assertEqual(self.balancer.active_mode, "SWING_UP")
        # Prime velocity estimator
        self.balancer.compute_action(180.0, 0.01)
        # Step near upright (180.0°) with low velocity -> switch to STABILIZE
        self.balancer.compute_action(182.0, 0.01)
        self.assertEqual(self.balancer.active_mode, "STABILIZE")
        # Step far from upright (e.g. 140°) -> switch back to SWING_UP
        self.balancer.compute_action(140.0, 0.01)
        self.assertEqual(self.balancer.active_mode, "SWING_UP")

    def test_velocity_estimation_returns_delta_and_filtered(self):
        self.balancer.reset()
        delta1, vel1 = self.balancer.estimate_velocity(0.0, 0.01)
        self.assertEqual(delta1, 0.0)
        self.assertEqual(vel1, 0.0)

        # 10 degrees in 0.01 sec = 1000 deg/s raw
        delta2, vel2 = self.balancer.estimate_velocity(10.0, 0.01)
        self.assertAlmostEqual(delta2, 10.0, places=1)
        # Filter alpha = 0.45: 0.45 * 1000 + 0.55 * 0 = 450.0
        self.assertAlmostEqual(vel2, 450.0, places=1)

if __name__ == "__main__":
    unittest.main()
