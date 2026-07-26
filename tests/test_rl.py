import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.core.state import PendulumState
from rl.rl_controller import RLBalancer

class TestRLBalancer(unittest.TestCase):
    """Test suite for Reinforcement Learning inference wrapper and fallback heuristics."""

    def test_rl_balancer_initialization_and_fallback(self):
        rl_ctrl = RLBalancer(algo="PPO", min_power=50, max_power=255)
        rl_ctrl.enable()
        self.assertFalse(rl_ctrl.is_loaded)
        self.assertEqual(rl_ctrl.name, "RL Policy (PPO)")

        # When upright (180.0°), error is 0 -> should output 0
        state_upright = PendulumState(angle_dev=180.0, velocity=0.0)
        action_upright = rl_ctrl.compute_action_from_state(state_upright, 0.01)
        self.assertEqual(action_upright, 0)

        # When tilted forward (175.0°, error +5.0°), should output reverse action (-speed)
        state_tilted = PendulumState(angle_dev=175.0, velocity=10.0)
        action_tilted = rl_ctrl.compute_action_from_state(state_tilted, 0.01)
        self.assertNotEqual(action_tilted, 0)
        self.assertTrue(50 <= abs(action_tilted) <= 255)
        self.assertLess(action_tilted, 0)

    def test_rl_balancer_disabled(self):
        rl_ctrl = RLBalancer()
        rl_ctrl.disable()
        state = PendulumState(angle_dev=170.0, velocity=20.0)
        self.assertEqual(rl_ctrl.compute_action_from_state(state, 0.01), 0)

if __name__ == "__main__":
    unittest.main()
