import unittest
import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from algorithm.math.envs.inverted_pendulum_env import InvertedPendulumEnv

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
        self.assertIn("precision_bonus", info)
        self.assertIn("is_spinning", info)
        self.assertIn("spin_penalty", info)
        if info["in_upright_buffer"]:
            self.assertGreaterEqual(reward, 0.0) # Holding bonus applies inside [179°, 181°]
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        self.assertIn("pwm_command", info)
        # 0.5 * 255 = 127.5, rounded (not truncated) to nearest PWM unit.
        self.assertEqual(info["pwm_command"], 128)

    def test_spin_penalty(self):
        env = InvertedPendulumEnv(simulated=True)
        env.reset(seed=42)
        # Spin fast (>360 deg/s == 6.28 rad/s). Seed the physics vector, since reward is
        # now scored on the post-step state.
        env._sim = np.array([0.0, 0.0, 0.0, 10.0], dtype=np.float64)
        _, reward, _, _, info = env.step(np.array([0.0], dtype=np.float32))

        self.assertTrue(info["is_spinning"])
        # Penalty scales with excess speed above 360 deg/s, so it exceeds the 50.0 base.
        self.assertGreater(info["spin_penalty"], 50.0)
        self.assertLess(reward, -10.0)

    def test_progressive_holding_bonus(self):
        env = InvertedPendulumEnv(simulated=True)
        env.reset(seed=42)
        # Hold exactly upright and still. With zero action the pole is at an equilibrium.
        env._sim = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        _, _, _, _, info1 = env.step(np.array([0.0], dtype=np.float32))
        self.assertEqual(info1["upright_steps"], 1)
        self.assertAlmostEqual(info1["precision_bonus"], 10.0) # max precision bonus at exact upright

        # Second step still exact
        _, _, _, _, info2 = env.step(np.array([0.0], dtype=np.float32))
        self.assertEqual(info2["upright_steps"], 2)
        self.assertAlmostEqual(info2["precision_bonus"], 10.0)
        
        # Third step slightly off center, check Gaussian drop
        env._sim = np.array([0.0, 0.0, 0.05, 0.0], dtype=np.float64)
        _, _, _, _, info3 = env.step(np.array([0.0], dtype=np.float32))
        # At exactly 0.05 radians error, gaussian is exp(-0.5 * 1^2) = exp(-0.5) ~ 0.606
        self.assertTrue(0.0 < info3["precision_bonus"] < 10.0)

        # Knock it well outside the +-1 deg buffer -> streak resets.
        env._sim = np.array([0.0, 0.0, 0.1, 0.0], dtype=np.float64)
        _, _, _, _, info3 = env.step(np.array([0.0], dtype=np.float32))
        self.assertEqual(info3["upright_steps"], 0)
        self.assertTrue(info3["precision_bonus"] < 2.0)

    def test_catch_the_fall_sign_convention(self):
        """A positive action must REDUCE a positive tilt (cart drives into the fall)."""
        env = InvertedPendulumEnv(simulated=True)
        env.reset(seed=7)
        env._sim = np.array([0.0, 0.0, 0.05, 0.0], dtype=np.float64)
        obs, _, _, _, _ = env.step(np.array([1.0], dtype=np.float32))
        self.assertLess(float(obs[1]), 0.0, "positive drive should induce negative omega")

        # And the mirror case.
        env.reset(seed=7)
        env._sim = np.array([0.0, 0.0, -0.05, 0.0], dtype=np.float64)
        obs, _, _, _, _ = env.step(np.array([-1.0], dtype=np.float32))
        self.assertGreater(float(obs[1]), 0.0)

    def test_swingup_task_does_not_terminate_while_hanging(self):
        env = InvertedPendulumEnv(simulated=True, task="swingup")
        env.reset(seed=3, options={"start_upright": False})
        _, _, terminated, _, _ = env.step(np.array([0.0], dtype=np.float32))
        self.assertFalse(terminated)

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
