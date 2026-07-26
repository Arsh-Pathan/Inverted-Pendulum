import math
from typing import Dict, Any
from .base_controller import BaseController
from ..core.state import PendulumState

class LQRBalancer(BaseController):
    """
    Linear Quadratic Regulator (LQR) Controller for inverted pendulum stabilization.
    Uses state-feedback matrix gain K = [k_theta, k_omega] around the linearized
    upright equilibrium point (180.0°).
    """
    def __init__(self,
                 k_theta: float = 25.0,
                 k_omega: float = 3.5,
                 alpha: float = 0.08,
                 min_power: int = 45,
                 max_power: int = 255,
                 deadzone_deg: float = 0.4,
                 deadzone_vel: float = 6.0,
                 target_angle: float = 180.0):
        super().__init__("LQR Balancer")
        self.k_theta = k_theta
        self.k_omega = k_omega
        self.alpha = alpha
        self.min_power = min_power
        self.max_power = max_power
        self.deadzone_deg = deadzone_deg
        self.deadzone_vel = deadzone_vel
        self.target_angle = target_angle

        self.filtered_velocity = 0.0
        self.prev_angle = 0.0
        self.first_run = True

    def reset(self):
        self.filtered_velocity = 0.0
        self.prev_angle = 0.0
        self.first_run = True

    def update_params(self, params: Dict[str, Any]):
        if "k_theta" in params: self.k_theta = float(params["k_theta"])
        if "k_omega" in params: self.k_omega = float(params["k_omega"])
        if "alpha" in params: self.alpha = max(0.01, min(1.0, float(params["alpha"])))
        if "min_power" in params: self.min_power = int(params["min_power"])
        if "max_power" in params: self.max_power = int(params["max_power"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled or dt <= 0.0001:
            return 0

        # Angular error from upright
        error = state.error_from_upright

        # Use EMA filtered velocity from state
        raw_vel = state.velocity
        if self.first_run:
            self.filtered_velocity = raw_vel
            self.first_run = False
        else:
            self.filtered_velocity = (self.alpha * raw_vel) + ((1.0 - self.alpha) * self.filtered_velocity)

        # State vector x = [error_deg, velocity_deg_s]
        # Control law u = k_theta * error + k_omega * velocity
        output = (self.k_theta * error) + (self.k_omega * self.filtered_velocity)

        # Equilibrium Deadzone
        if abs(error) < self.deadzone_deg and abs(self.filtered_velocity) < self.deadzone_vel:
            return 0

        abs_output = abs(output)
        if abs_output <= 0.05:
            return 0

        speed = self.min_power + int(abs_output * (self.max_power - self.min_power) / 255.0)
        speed = max(self.min_power, min(self.max_power, speed))

        return -speed if output > 0 else speed

    def compute_action(self, angle_deg: float, dt: float) -> int:
        err = 180.0 - angle_deg
        while err > 180.0: err -= 360.0
        while err < -180.0: err += 360.0

        raw_vel = 0.0
        if not self.first_run and dt > 0:
            delta = (angle_deg - self.prev_angle) % 360.0
            if delta > 180.0: delta -= 360.0
            elif delta < -180.0: delta += 360.0
            raw_vel = delta / dt
        self.prev_angle = angle_deg

        state_stub = PendulumState(angle_dev=angle_deg, velocity=raw_vel)
        return self.compute_action_from_state(state_stub, dt)
