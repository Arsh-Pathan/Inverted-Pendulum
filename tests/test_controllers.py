import unittest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.math.core.state import PendulumState
from algorithm.math.controllers import (PIDBalancer, LQRBalancer, SwingUpController, 
                                HybridBalancer, OscillationController)

class TestControllers(unittest.TestCase):
    """Test suite verifying mathematical control laws and state switching."""

    def test_pid_balancer_deadzone(self):
        pid = PIDBalancer(deadzone_deg=0.5, deadzone_vel=10.0)
        pid.enable()
        
        # Exact upright (180.0°) with 0 velocity -> must coast (0)
        action_upright = pid.compute_action(180.0, 0.01)
        self.assertEqual(action_upright, 0)

        # Within deadzone (180.2° with 2°/s velocity) -> must coast (0)
        action_deadzone = pid.compute_action(180.2, 0.01)
        self.assertEqual(action_deadzone, 0)

    def test_pid_balancer_action_sign(self):
        pid = PIDBalancer(kp=15.0, ki=0.0, kd=0.0, min_power=50, max_power=255)
        pid.enable()

        # Tilted forward to 175° (error = +5.0°) -> output > 0 -> reverse action (-speed)
        action_fwd = pid.compute_action(175.0, 0.01)
        self.assertLess(action_fwd, 0)
        self.assertGreaterEqual(abs(action_fwd), 50)
        self.assertLessEqual(abs(action_fwd), 255)

        # Tilted backward to 185° (error = -5.0°) -> output < 0 -> forward action (+speed)
        pid.reset()
        action_bwd = pid.compute_action(185.0, 0.01)
        self.assertGreater(action_bwd, 0)
        self.assertGreaterEqual(abs(action_bwd), 50)
        self.assertLessEqual(abs(action_bwd), 255)

    def test_lqr_balancer_state_feedback(self):
        lqr = LQRBalancer(k_theta=20.0, k_omega=3.0, min_power=45, max_power=255)
        lqr.enable()

        # Tilted to 170° (error = +10.0°) with 0 velocity
        state = PendulumState(angle_dev=170.0, velocity=0.0)
        action = lqr.compute_action_from_state(state, 0.01)
        self.assertNotEqual(action, 0)
        self.assertTrue(45 <= abs(action) <= 255)

    def test_swing_up_energy_pumping(self):
        su = SwingUpController(pump_power=200)
        su.enable()

        # Hanging down (0°) swinging right (vel > 0) -> energy_term > 0 -> push right (+pump_power)
        state_right = PendulumState(angle_dev=5.0, velocity=100.0)
        action_right = su.compute_action_from_state(state_right, 0.01)
        self.assertEqual(action_right, 200)

        # Swinging left (vel < 0) -> energy_term < 0 -> push left (-pump_power)
        state_left = PendulumState(angle_dev=5.0, velocity=-100.0)
        action_left = su.compute_action_from_state(state_left, 0.01)
        self.assertEqual(action_left, -200)

    def test_hybrid_balancer_mode_switching(self):
        hybrid = HybridBalancer(capture_angle_deg=20.0)
        hybrid.enable()
        self.assertEqual(hybrid.active_mode, "SWING_UP")

        # Far from upright (100° deviation) -> should stay in SWING_UP mode
        state_far = PendulumState(angle_dev=100.0, velocity=50.0)
        hybrid.compute_action_from_state(state_far, 0.01)
        self.assertEqual(hybrid.active_mode, "SWING_UP")

        # Enter capture basin (175° deviation, low velocity) -> should switch to STABILIZE mode!
        state_near = PendulumState(angle_dev=175.0, velocity=10.0)
        hybrid.compute_action_from_state(state_near, 0.01)
        self.assertEqual(hybrid.active_mode, "STABILIZE")

    def test_oscillation_controller(self):
        osc = OscillationController(speed=150, duration_ms=50)
        osc.enable()
        
        action_1 = osc.compute_action(180.0, 0.01)
        self.assertEqual(action_1, 150)
        
        time.sleep(0.06) # Wait for duration to elapse
        action_2 = osc.compute_action(180.0, 0.01)
        self.assertEqual(action_2, -150)

if __name__ == "__main__":
    unittest.main()
