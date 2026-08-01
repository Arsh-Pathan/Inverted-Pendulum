import math
from typing import Any, Dict

import numpy as np

from .base_controller import BaseController
from ..core.state import PendulumState


class LQRBalancer(BaseController):
    """
    LQR balance controller based on the referenced Arduino implementation.

    State order is [cart_x, cart_v, theta, theta_dot]. This repo has no cart
    encoder, so cart_x/cart_v are estimated from the PWM command that was issued.
    """

    def __init__(
        self,
        k_theta: float = 25.0,
        k_omega: float = 3.5,
        alpha: float = 0.45,
        min_power: int = 35,
        max_power: int = 255,
        deadzone_deg: float = 0.0,
        deadzone_vel: float = 0.0,
        k_cart_v: float = 150.0,
        k_cart_x: float = 200.0,
        cart_accel_max: float = 6.0,
        cart_damping: float = 7.5,
        dither_power: int = 0,
        input_gain_n_per_pwm: float = 0.008,
        control_loop_rate_hz: float = 200.0,
    ):
        super().__init__("LQR Balancer")
        self.k_theta = k_theta
        self.k_omega = k_omega
        self.alpha = alpha
        self.min_power = min_power
        self.max_power = max_power
        self.deadzone_deg = deadzone_deg
        self.deadzone_vel = deadzone_vel
        self.k_cart_v = k_cart_v
        self.k_cart_x = k_cart_x
        self.cart_accel_max = cart_accel_max
        self.cart_damping = cart_damping
        self.dither_power = dither_power
        self.input_gain_n_per_pwm = input_gain_n_per_pwm
        self.control_loop_rate_hz = control_loop_rate_hz

        self.filtered_velocity = 0.0
        self.prev_angle = 0.0
        self.first_run = True
        self.est_cart_v = 0.0
        self.est_cart_x = 0.0
        self._dither_sign = 1
        self.k_lqr = self._compute_lqr_gain()

    def reset(self):
        self.filtered_velocity = 0.0
        self.prev_angle = 0.0
        self.first_run = True
        self.est_cart_v = 0.0
        self.est_cart_x = 0.0
        self._dither_sign = 1

    def update_params(self, params: Dict[str, Any]):
        if "k_theta" in params:
            self.k_theta = float(params["k_theta"])
        if "k_omega" in params:
            self.k_omega = float(params["k_omega"])
        if "alpha" in params:
            self.alpha = max(0.01, min(1.0, float(params["alpha"])))
        if "min_power" in params:
            self.min_power = int(params["min_power"])
        if "max_power" in params:
            self.max_power = int(params["max_power"])
        if "k_cart_v" in params:
            self.k_cart_v = float(params["k_cart_v"])
        if "k_cart_x" in params:
            self.k_cart_x = float(params["k_cart_x"])
        if "dither_power" in params:
            self.dither_power = int(params["dither_power"])
        if "input_gain_n_per_pwm" in params:
            self.input_gain_n_per_pwm = max(1e-6, float(params["input_gain_n_per_pwm"]))
            self.k_lqr = self._compute_lqr_gain()
        if "control_loop_rate_hz" in params:
            self.control_loop_rate_hz = max(1.0, float(params["control_loop_rate_hz"]))
            self.k_lqr = self._compute_lqr_gain()

    def _compute_lqr_gain(self) -> np.ndarray:
        cart_mass = 0.266
        pendulum_mass = 0.055
        com_length = 0.352
        inertia_com = 0.00108
        gravity = 9.81
        dt = 1.0 / self.control_loop_rate_hz

        determinant = (
            inertia_com * (cart_mass + pendulum_mass)
            + cart_mass * pendulum_mass * com_length * com_length
        )

        A = np.zeros((4, 4), dtype=np.float64)
        A[0, 1] = 1.0
        A[1, 2] = pendulum_mass * pendulum_mass * gravity * com_length * com_length / determinant
        A[2, 3] = 1.0
        A[3, 2] = pendulum_mass * gravity * com_length * (cart_mass + pendulum_mass) / determinant

        B_force = np.array(
            [
                0.0,
                (inertia_com + pendulum_mass * com_length * com_length) / determinant,
                0.0,
                -(pendulum_mass * com_length) / determinant,
            ],
            dtype=np.float64,
        )

        Ad = np.eye(4, dtype=np.float64) + A * dt
        Bd = B_force * self.input_gain_n_per_pwm * dt
        Q = np.diag([3000.0, 100.0, 8000.0, 20.0])
        R = 0.10

        P = Q.copy()
        for _ in range(200):
            PA = P @ Ad
            PB = P @ Bd
            S = R + float(Bd @ PB)
            if abs(S) < 1e-9:
                return self._fallback_gain()

            BtPA = Bd @ PA
            Pn = Ad.T @ PA - np.outer(BtPA, BtPA) / S + Q
            err = float(np.max(np.abs(Pn - P)))
            P = Pn
            if err < 1e-4:
                break

        PA = P @ Ad
        PB = P @ Bd
        S = R + float(Bd @ PB)
        if abs(S) < 1e-9 or not np.isfinite(S):
            return self._fallback_gain()

        K = (Bd @ PA) / S
        if not np.all(np.isfinite(K)):
            return self._fallback_gain()

        K[0] *= 8.0
        K[1] *= 5.0
        K[3] = max(-60.0, min(60.0, K[3]))
        return K

    def _fallback_gain(self) -> np.ndarray:
        return np.array(
            [
                -self.k_cart_x,
                -self.k_cart_v,
                -self.k_theta / math.radians(1.0),
                -self.k_omega / math.radians(1.0),
            ],
            dtype=np.float64,
        )

    def _update_cart_estimate(self, pwm: int, dt: float):
        a_cmd = (float(pwm) / 255.0) * self.cart_accel_max
        self.est_cart_v += (a_cmd - self.cart_damping * self.est_cart_v) * dt
        self.est_cart_x += self.est_cart_v * dt

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled or dt <= 0.0001:
            return 0

        theta = state.theta_from_upright
        raw_velocity = state.velocity
        if self.first_run:
            self.filtered_velocity = raw_velocity
            self.first_run = False
        else:
            self.filtered_velocity = (
                self.alpha * raw_velocity
                + (1.0 - self.alpha) * self.filtered_velocity
            )

        if (
            self.deadzone_deg > 0.0
            and abs(theta) < self.deadzone_deg
            and abs(self.filtered_velocity) < self.deadzone_vel
        ):
            self._update_cart_estimate(0, dt)
            return 0

        state_vector = np.array(
            [
                self.est_cart_x,
                self.est_cart_v,
                math.radians(theta),
                math.radians(self.filtered_velocity),
            ],
            dtype=np.float64,
        )
        command = -float(self.k_lqr @ state_vector)
        pwm = self.apply_deadband_bias(command, self.min_power, self.max_power)
        if pwm == 0:
            self._update_cart_estimate(0, dt)
            return 0

        if self.dither_power > 0 and abs(theta) < 10.0:
            pwm += self._dither_sign * self.dither_power
            self._dither_sign *= -1
            pwm = max(-self.max_power, min(self.max_power, pwm))

        self._update_cart_estimate(pwm, dt)
        return pwm

    def compute_action(self, angle_deg: float, dt: float) -> int:
        raw_velocity = 0.0
        if not self.first_run and dt > 0:
            delta = (angle_deg - self.prev_angle) % 360.0
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            raw_velocity = delta / dt
        self.prev_angle = angle_deg

        state_stub = PendulumState(angle_dev=angle_deg, velocity=raw_velocity)
        return self.compute_action_from_state(state_stub, dt)
