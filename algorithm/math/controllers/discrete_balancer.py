"""
Discrete Track-Constrained Inverted Pendulum Balancer.

Combines high-performance energy-pumping swing-up with chatter-free discrete balancing,
track-boundary safety constraints, and low-pass velocity filtering.
"""

from enum import IntEnum
from typing import Tuple, Optional, Dict, Any
from .base_controller import BaseController
from ..core.state import PendulumState


class DiscreteAction(IntEnum):
    LEFT = -1
    STOP = 0
    RIGHT = 1


class DiscreteTrackBalancer(BaseController):
    """
    Discrete action balancer designed for single-sensor angle-only feedback
    and track-constrained hardware / simulation setups.
    """
    def __init__(
        self,
        pwm_power: int = 255,
        stabilize_power: int = 200,
        track_limit: float = 0.60,
        capture_angle_deg: float = 20.0,
        k_omega: float = 0.18,
        deadband: float = 0.6,
        filter_alpha: float = 0.45,
    ):
        super().__init__("Discrete Track Balancer")
        self.pwm_power = int(pwm_power)
        self.stabilize_power = int(stabilize_power)
        self.track_limit = float(track_limit)
        self.capture_angle_deg = float(capture_angle_deg)
        self.k_omega = float(k_omega)
        self.deadband = float(deadband)
        self.filter_alpha = float(filter_alpha)

        self.zero_offset: float = 0.0
        self.active_mode: str = "SWING_UP"
        self._kick_counter: int = 0
        self._filtered_vel: float = 0.0
        self._prev_angle: Optional[float] = None
        self.est_x: float = 0.0
        self.est_v: float = 0.0

    def enable(self):
        super().enable()
        self.reset()

    def disable(self):
        super().disable()

    def reset(self):
        self.active_mode = "SWING_UP"
        self._kick_counter = 0
        self._filtered_vel = 0.0
        self._prev_angle = None
        self.est_x = 0.0
        self.est_v = 0.0

    def tare(self, initial_zero: float):
        """Sets the zero reference offset angle for hanging rest position."""
        self.zero_offset = float(initial_zero)
        self._kick_counter = 0

    def estimate_velocity(self, raw_angle_deg: float, dt: float) -> Tuple[float, float]:
        """
        Estimates angular velocity from raw angle readings using exponential moving average filtering.
        Returns tuple (delta_deg, filtered_vel_deg_s).
        """
        if self._prev_angle is None or dt <= 0:
            self._prev_angle = float(raw_angle_deg)
            self._filtered_vel = 0.0
            return 0.0, 0.0

        delta = (float(raw_angle_deg) - self._prev_angle) % 360.0
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0

        raw_vel = delta / dt
        self._filtered_vel = self.filter_alpha * raw_vel + (1.0 - self.filter_alpha) * self._filtered_vel
        self._prev_angle = float(raw_angle_deg)

        return delta, self._filtered_vel

    def _update_cart_estimate(self, pwm: int, dt: float):
        """Estimate cart velocity and position via damped dead-reckoning model."""
        if dt <= 0:
            return
        a_cmd = (float(pwm) / max(1, self.pwm_power)) * 5.0
        self.est_v += (a_cmd - 6.0 * self.est_v) * dt
        self.est_x += self.est_v * dt
        # Soft clamp within max track bounds
        max_bound = self.track_limit * 1.2
        self.est_x = max(-max_bound, min(max_bound, self.est_x))

    def compute_action_enum(self, raw_angle_deg: float, dt: float) -> Tuple[DiscreteAction, int]:
        """
        Computes discrete action enum (RIGHT, LEFT, STOP) and motor PWM power (-255 to +255).
        """
        delta, vel = self.estimate_velocity(raw_angle_deg, dt)
        cal_angle_deg = (float(raw_angle_deg) - self.zero_offset) % 360.0

        # Compute angular displacement from upright equilibrium (180°)
        theta_from_upright = (cal_angle_deg - 180.0) % 360.0
        if theta_from_upright > 180.0:
            theta_from_upright -= 360.0

        abs_err = abs(theta_from_upright)

        # Hysteresis mode switching logic
        if self.active_mode == "SWING_UP":
            if abs_err <= self.capture_angle_deg and abs(vel) < 150.0:
                self.active_mode = "STABILIZE"
        else:  # STABILIZE mode
            if abs_err > self.capture_angle_deg + 10.0:
                self.active_mode = "SWING_UP"

        # Check kick start condition from dead rest
        if self.active_mode == "SWING_UP" and self._kick_counter == 0:
            self._kick_counter += 1
            pwm = self.pwm_power
            action = DiscreteAction.RIGHT
            self._update_cart_estimate(pwm, dt)
            return action, pwm

        # Action computation based on active mode
        if self.active_mode == "STABILIZE":
            # Non-linear cart position recentering bias
            cart_norm = self.est_x / max(0.01, self.track_limit)
            cart_bias = -12.0 * cart_norm * (1.0 + abs(cart_norm))
            effective_err = theta_from_upright + self.k_omega * vel + cart_bias

            power = self.pwm_power if (abs(vel) > 20.0 or abs_err > 3.0) else self.stabilize_power

            if abs(theta_from_upright) <= 0.3 and abs(vel) <= 5.0 and abs(self.est_x) < 0.1:
                action = DiscreteAction.STOP
                pwm = 0
            elif effective_err > 0:
                action = DiscreteAction.RIGHT
                pwm = power
            else:
                action = DiscreteAction.LEFT
                pwm = -power
        else:
            # SWING_UP energy pumping
            if abs(vel) > 10.0:
                if vel > 0:
                    action = DiscreteAction.RIGHT
                    pwm = self.pwm_power
                else:
                    action = DiscreteAction.LEFT
                    pwm = -self.pwm_power
            else:
                if theta_from_upright > 0:
                    action = DiscreteAction.RIGHT
                    pwm = self.pwm_power
                else:
                    action = DiscreteAction.LEFT
                    pwm = -self.pwm_power

        # Active track boundary override: prevent driving into rail limits by reversing effort
        if self.est_x > 0.4 * self.track_limit and pwm > 0:
            action = DiscreteAction.LEFT
            pwm = -self.pwm_power
        elif self.est_x < -0.4 * self.track_limit and pwm < 0:
            action = DiscreteAction.RIGHT
            pwm = self.pwm_power

        self._update_cart_estimate(pwm, dt)
        return action, pwm

    def compute_action(self, angle_deg: float, dt: float) -> int:
        action_enum, pwm = self.compute_action_enum(angle_deg, dt)
        return pwm

    def update_params(self, params: Dict[str, Any]):
        if "pwm_power" in params:
            self.pwm_power = int(params["pwm_power"])
        if "stabilize_power" in params:
            self.stabilize_power = int(params["stabilize_power"])
        if "track_limit" in params:
            self.track_limit = float(params["track_limit"])
        if "capture_angle_deg" in params:
            self.capture_angle_deg = float(params["capture_angle_deg"])
        if "k_omega" in params:
            self.k_omega = float(params["k_omega"])
        if "deadband" in params:
            self.deadband = float(params["deadband"])
