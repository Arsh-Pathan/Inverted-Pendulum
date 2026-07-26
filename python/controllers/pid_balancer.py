import math
from typing import Dict, Any
from .base_controller import BaseController
from ..core.state import PendulumState

class PIDBalancer(BaseController):
    """
    Python implementation of the high-frequency PID balancing algorithm.
    Includes Exponential Moving Average (EMA) derivative filtering, linear
    deadband compensation, and lower hemisphere inversion.
    """
    def __init__(self, 
                 kp: float = 15.0, 
                 ki: float = 0.0, 
                 kd: float = 2.5, 
                 alpha: float = 0.08,
                 min_power: int = 45, 
                 max_power: int = 255,
                 deadzone_deg: float = 0.8, 
                 deadzone_vel: float = 12.0,
                 target_angle: float = 180.0):
        super().__init__("PID Balancer")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.min_power = min_power
        self.max_power = max_power
        self.deadzone_deg = deadzone_deg
        self.deadzone_vel = deadzone_vel
        self.target_angle = target_angle

        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True

    def update_params(self, params: Dict[str, Any]):
        if "kp" in params: self.kp = float(params["kp"])
        if "ki" in params: self.ki = float(params["ki"])
        if "kd" in params: self.kd = float(params["kd"])
        if "alpha" in params: self.alpha = max(0.01, min(1.0, float(params["alpha"])))
        if "min_power" in params: self.min_power = int(params["min_power"])
        if "max_power" in params: self.max_power = int(params["max_power"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled or dt <= 0.0001:
            return 0

        # 1) Angular error from upright target
        error = state.error_from_upright

        if self.first_run:
            self.prev_error = error
            self.first_run = False

        # 2) Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-100.0, min(100.0, self.integral))

        # 3) Raw derivative calculation
        raw_derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # 4) Exponential Moving Average (EMA) low-pass filter on derivative
        self.filtered_derivative = (self.alpha * raw_derivative) + ((1.0 - self.alpha) * self.filtered_derivative)

        # 5) PID Output calculation
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.filtered_derivative)

        # 6) Equilibrium Deadzone: when upright and quiet, coast to prevent stiction limit-cycle buzzing
        if abs(error) < self.deadzone_deg and abs(self.filtered_derivative) < self.deadzone_vel:
            return 0

        # 7) Linear Deadband Mapping: overcome static motor friction by mapping power from min_power up
        abs_output = abs(output)
        if abs_output <= 0.05:
            return 0

        speed = self.min_power + int(abs_output * (self.max_power - self.min_power) / 255.0)
        speed = max(self.min_power, min(self.max_power, speed))

        # 9) Direction assignment
        if output > 0:
            return -speed # Reverse action to catch forward tilt
        else:
            return speed  # Forward action to catch backward tilt

    def compute_action(self, angle_deg: float, dt: float) -> int:
        err = self.target_angle - angle_deg
        while err > 180.0: err -= 360.0
        while err < -180.0: err += 360.0

        raw_vel = 0.0
        if not self.first_run and dt > 0:
            raw_vel = (err - self.prev_error) / dt

        state_stub = PendulumState(angle_dev=angle_deg, velocity=raw_vel)
        return self.compute_action_from_state(state_stub, dt)
