from typing import Dict, Any, Optional
from .base_controller import BaseController
from .swing_up import SwingUpController
from .pid_balancer import PIDBalancer
from ..core.state import PendulumState

class HybridBalancer(BaseController):
    """
    Autonomous Hybrid Controller: Monitors PendulumState and dynamically switches
    between the energy-pumping SwingUpController (when hanging down or swinging)
    and the precision stabilizing controller (PID or LQR) when entering the upright
    capture basin (±20.0°).
    """
    def __init__(self,
                 stabilizer: Optional[BaseController] = None,
                 swing_up: Optional[SwingUpController] = None,
                 capture_angle_deg: float = 20.0):
        super().__init__("Hybrid Swing-Up & Balance")
        self.stabilizer = stabilizer or PIDBalancer()
        self.swing_up = swing_up or SwingUpController()
        self.capture_angle_deg = capture_angle_deg
        self.active_mode = "SWING_UP" # "SWING_UP" or "STABILIZE"
        self.prev_angle = None

    def enable(self):
        super().enable()
        self.stabilizer.enable()
        self.swing_up.enable()
        self.active_mode = "SWING_UP"

    def disable(self):
        super().disable()
        self.stabilizer.disable()
        self.swing_up.disable()

    def reset(self):
        self.stabilizer.reset()
        self.swing_up.reset()
        self.active_mode = "SWING_UP"
        self.prev_angle = None

    def update_params(self, params: Dict[str, Any]):
        self.stabilizer.update_params(params)
        self.swing_up.update_params(params)
        if "capture_angle_deg" in params:
            self.capture_angle_deg = float(params["capture_angle_deg"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled:
            return 0

        error = abs(state.error_from_upright)
        
        # Hysteresis switching logic to avoid rapid fluttering at basin boundary
        if self.active_mode == "SWING_UP":
            if error <= self.capture_angle_deg and abs(state.velocity) < 150.0:
                self.active_mode = "STABILIZE"
                print(f"[HYBRID] Capture basin entered (error={error:.1f}°). Switching to {self.stabilizer.name}!")
        else: # STABILIZE mode
            if error > self.capture_angle_deg + 10.0:
                self.active_mode = "SWING_UP"
                print(f"[HYBRID] Balance lost (error={error:.1f}°). Reverted to Energy Swing-Up!")

        if self.active_mode == "STABILIZE":
            return self.stabilizer.compute_action_from_state(state, dt)
        else:
            return self.swing_up.compute_action_from_state(state, dt)

    def compute_action(self, angle_deg: float, dt: float) -> int:
        raw_vel = 0.0
        if self.prev_angle is not None and dt > 0:
            delta = (angle_deg - self.prev_angle) % 360.0
            if delta > 180.0: delta -= 360.0
            elif delta < -180.0: delta += 360.0
            raw_vel = delta / dt
        self.prev_angle = angle_deg

        state_stub = PendulumState(angle_dev=angle_deg, velocity=raw_vel)
        return self.compute_action_from_state(state_stub, dt)
