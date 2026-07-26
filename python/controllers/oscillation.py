import time
from typing import Dict, Any
from .base_controller import BaseController

class OscillationController(BaseController):
    """
    Test mode controller that oscillates the motor forward and reverse
    periodically. Useful for checking physical rail clearance and H-bridge response.
    """
    def __init__(self, speed: int = 255, duration_ms: int = 400):
        super().__init__("Oscillation Tester")
        self.speed = speed
        self.duration_ms = duration_ms
        self.last_switch_time = 0.0
        self.moving_forward = True

    def reset(self):
        self.last_switch_time = time.time()
        self.moving_forward = True

    def update_params(self, params: Dict[str, Any]):
        if "speed" in params: self.speed = max(0, min(255, int(params["speed"])))
        if "duration_ms" in params: self.duration_ms = max(10, int(params["duration_ms"]))

    def compute_action(self, angle_deg: float, dt: float) -> int:
        if not self.enabled:
            return 0

        now = time.time()
        if (now - self.last_switch_time) * 1000.0 >= self.duration_ms:
            self.last_switch_time = now
            self.moving_forward = not self.moving_forward

        return self.speed if self.moving_forward else -self.speed
