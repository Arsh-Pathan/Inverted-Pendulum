import unittest
import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from python.envs.inverted_pendulum_env import InvertedPendulumEnv

class TestInvertedPendulumEnv(unittest.TestCase):
    """Test suite for Gymnasium RL environment non-linear simulation and interfaces."""

    def test_env_initialization_and_reset(self):
        env = InvertedPendulumEnv(simulated=True)
        obs, info = env.reset(seed=42)
        
        self.assertIsInstance(obs, np.ndarray)
        self.assertEqual(obs.shape, (2,))
        self.assertIsInstance(info, dict)
        # Starting upright -> initial error and velocity should be near 0
        self.assertTrue(abs(obs[0]) < 0.2)
        self.assertTrue(abs(obs[1]) < 0.5)

    def test_env_step_and_reward(self):
        env = InvertedPendulumEnv(simulated=True)
        obs, _ = env.reset(seed=101)

        # Apply positive action (+0.5 -> 127 PWM)
        action = np.array([0.5], dtype=np.float32)
        next_obs, reward, terminated, truncated, info = env.step(action)

        self.assertEqual(next_obs.shape, (2,))
        self.assertIsInstance(reward, float)
        self.assertIn("in_upright_buffer", info)
        self.assertIn("holding_bonus", info)
        if info["in_upright_buffer"]:
            self.assertGreaterEqual(reward, 0.0) # Holding bonus applies inside [179°, 181°]
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        self.assertIn("pwm_command", info)
        self.assertEqual(info["pwm_command"], 127)

    def test_episode_truncation(self):
        env = InvertedPendulumEnv(simulated=True, max_episode_steps=5)
        env.reset()
        for i in range(4):
            _, _, _, truncated, _ = env.step([0.0])
            self.assertFalse(truncated)
            
        _, _, _, truncated, _ = env.step([0.0])
        self.assertTrue(truncated)

if __name__ == "__main__":
    unittest.main()
