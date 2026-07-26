import os
import math
import numpy as np
from typing import Dict, Any, Optional

try:
    from stable_baselines3 import PPO, SAC, TD3, A2C
    _SB3_AVAILABLE = True
except ImportError:
    _SB3_AVAILABLE = False

from python.controllers.base_controller import BaseController
from python.core.state import PendulumState

class RLBalancer(BaseController):
    """
    Reinforcement Learning Inference Engine.
    Wraps a trained Stable-Baselines3 (or PyTorch) neural network policy and deploys
    it in real-time inside the Hardware-in-the-Loop (HIL) serial control loop.
    """
    def __init__(self, model_path: Optional[str] = None, algo: str = "PPO", min_power: int = 45, max_power: int = 255):
        super().__init__(f"RL Policy ({algo})")
        self.model_path = model_path
        self.algo = algo.upper()
        self.min_power = min_power
        self.max_power = max_power
        self.model = None
        self.is_loaded = False

        if model_path and os.path.exists(model_path):
            self.load_model(model_path, self.algo)

    def load_model(self, path: str, algo: str = "PPO") -> bool:
        """Loads a saved .zip model from disk."""
        if not _SB3_AVAILABLE:
            print("[RL ERROR] stable-baselines3 is not installed. Run: pip install stable-baselines3 torch")
            return False
            
        if not os.path.exists(path):
            print(f"[RL ERROR] Model file not found: {path}")
            return False

        try:
            if algo == "PPO":
                self.model = PPO.load(path)
            elif algo == "SAC":
                self.model = SAC.load(path)
            elif algo == "TD3":
                self.model = TD3.load(path)
            elif algo == "A2C":
                self.model = A2C.load(path)
            else:
                self.model = PPO.load(path)
                
            self.model_path = path
            self.is_loaded = True
            print(f"[RL] Successfully loaded {algo} model from {path}")
            return True
        except Exception as e:
            print(f"[RL ERROR] Failed to load model {path}: {e}")
            self.is_loaded = False
            return False

    def reset(self):
        pass

    def update_params(self, params: Dict[str, Any]):
        if "model_path" in params and params["model_path"] != self.model_path:
            self.load_model(params["model_path"], params.get("algo", self.algo))
        if "min_power" in params: self.min_power = int(params["min_power"])
        if "max_power" in params: self.max_power = int(params["max_power"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled:
            return 0

        # Construct continuous observation vector matching InvertedPendulumEnv: [error_rad, vel_rad_s]
        err_rad = math.radians(state.error_from_upright)
        vel_rad = math.radians(state.velocity)
        obs = np.array([err_rad, vel_rad], dtype=np.float32)

        if not self.is_loaded or self.model is None:
            # Fallback heuristic if model is not loaded: proportional damping
            norm_action = -float(np.clip(err_rad * 2.0 + vel_rad * 0.1, -1.0, 1.0))
        else:
            action, _ = self.model.predict(obs, deterministic=True)
            norm_action = float(action[0] if isinstance(action, (np.ndarray, list)) else action)

        # Map normalized action [-1.0, 1.0] to integer PWM [-255, 255] with deadband
        abs_act = abs(norm_action)
        if abs_act <= 0.05:
            return 0

        speed = self.min_power + int(abs_act * (self.max_power - self.min_power))
        speed = max(self.min_power, min(self.max_power, speed))

        return speed if norm_action > 0 else -speed

    def compute_action(self, angle_deg: float, dt: float) -> int:
        err = 180.0 - angle_deg
        while err > 180.0: err -= 360.0
        while err < -180.0: err += 360.0
        state_stub = PendulumState(angle_dev=angle_deg, velocity=0.0)
        return self.compute_action_from_state(state_stub, dt)
