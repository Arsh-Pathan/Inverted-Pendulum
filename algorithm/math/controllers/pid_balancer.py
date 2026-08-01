from typing import Dict, Any
from .base_controller import BaseController
from ..core.state import PendulumState

class PIDBalancer(BaseController):
    """
    High-frequency PID balancing controller with EMA derivative filtering, deadband
    compensation, and an observer-based cart-recentering term.

    ── Why the extra cart term ──
    The rig has no cart encoder, so a pure angle PID can hold the pole vertical but
    cannot see the cart drifting. Any small residual tilt makes the cart accelerate
    steadily in one direction; it eventually runs out of rail, and the resulting
    end-stop impact is what looks like "it balances, then overshoots and falls".

    Since every PWM command we send is known, cart velocity/position are recovered by
    dead-reckoning the actuator model  v' = a_cmd - c*v  (see `k_cart_v` / `k_cart_x`).
    This restores the two missing LQR states with no additional hardware and is
    tolerant of large actuator-model error, because the terms only need to be roughly
    right to remove the drift.
    """
    def __init__(self,
                 kp: float = 20.0,
                 ki: float = 0.0,
                 kd: float = 2.5,
                 alpha: float = 0.45,
                 min_power: int = 35,
                 max_power: int = 255,
                 deadzone_deg: float = 0.0,
                 deadzone_vel: float = 0.0,
                 k_cart_v: float = 150.0,
                 k_cart_x: float = 200.0,
                 cart_accel_max: float = 6.0,
                 cart_damping: float = 7.5,
                 dither_power: int = 0):
        super().__init__("PID Balancer")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.min_power = min_power
        self.max_power = max_power
        self.deadzone_deg = deadzone_deg
        self.deadzone_vel = deadzone_vel

        # Cart-recentering gains and the actuator model used to estimate cart motion.
        self.k_cart_v = k_cart_v
        self.k_cart_x = k_cart_x
        self.cart_accel_max = cart_accel_max
        self.cart_damping = cart_damping

        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True
        self.est_cart_v = 0.0
        self.est_cart_x = 0.0
        self.dither_power = dither_power
        self._dither_sign = 1

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True
        self.est_cart_v = 0.0
        self.est_cart_x = 0.0

    def update_params(self, params: Dict[str, Any]):
        if "kp" in params: self.kp = float(params["kp"])
        if "ki" in params: self.ki = float(params["ki"])
        if "kd" in params: self.kd = float(params["kd"])
        if "alpha" in params: self.alpha = max(0.01, min(1.0, float(params["alpha"])))
        if "min_power" in params: self.min_power = int(params["min_power"])
        if "max_power" in params: self.max_power = int(params["max_power"])
        if "k_cart_v" in params: self.k_cart_v = float(params["k_cart_v"])
        if "k_cart_x" in params: self.k_cart_x = float(params["k_cart_x"])
        if "dither_power" in params: self.dither_power = int(params["dither_power"])

    def _update_cart_estimate(self, pwm: int, dt: float):
        """Dead-reckon cart velocity/position from the command actually issued."""
        a_cmd = (float(pwm) / 255.0) * self.cart_accel_max
        self.est_cart_v += (a_cmd - self.cart_damping * self.est_cart_v) * dt
        self.est_cart_x += self.est_cart_v * dt

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled or dt <= 0.0001:
            return 0

        # 1) Signed tilt from upright. Uses the canonical `theta_from_upright` so that
        #    theta and its derivative share one sign convention (see core/state.py).
        theta = state.theta_from_upright

        if self.first_run:
            self.prev_error = theta
            self.filtered_derivative = 0.0
            self.first_run = False

        # 2) Integral term with anti-windup clamping
        self.integral += theta * dt
        self.integral = max(-100.0, min(100.0, self.integral))

        # 3) Derivative. Prefer the measured angular velocity when the caller supplied
        #    one: differencing a 12-bit quantised angle at 100 Hz produces ~8.8 deg/s of
        #    step noise per LSB, which the EMA can only partly hide.
        if state.velocity != 0.0:
            raw_derivative = state.velocity
        else:
            delta = (theta - self.prev_error) % 360.0
            if delta > 180.0: delta -= 360.0
            elif delta < -180.0: delta += 360.0
            raw_derivative = delta / dt
        self.prev_error = theta

        # 4) Exponential Moving Average (EMA) low-pass filter on derivative
        self.filtered_derivative = (self.alpha * raw_derivative) + ((1.0 - self.alpha) * self.filtered_derivative)

        # 5) PID output plus cart recentering. The angle terms catch the fall; the
        #    estimated cart terms drive back toward the center of the rail.
        output = ((self.kp * theta)
                  + (self.ki * self.integral)
                  + (self.kd * self.filtered_derivative)
                  - (self.k_cart_v * self.est_cart_v)
                  - (self.k_cart_x * self.est_cart_x))

        # 6) Equilibrium deadzone: when upright and quiet, coast to avoid stiction buzz.
        #    Defaults to disabled (0.0): a deadzone near the target withholds exactly the
        #    small corrections needed to arrest an incipient fall, which measurably
        #    increased overshoot in simulation.
        if (self.deadzone_deg > 0.0
                and abs(theta) < self.deadzone_deg
                and abs(self.filtered_derivative) < self.deadzone_vel):
            self._update_cart_estimate(0, dt)
            return 0

        pwm = self.apply_deadband_bias(output, self.min_power, self.max_power)
        if pwm == 0:
            self._update_cart_estimate(0, dt)
            return 0
        
        # 9) High-frequency dither (vibration) to break static friction
        if self.dither_power > 0 and abs(theta) < 10.0:
            pwm += self._dither_sign * self.dither_power
            self._dither_sign *= -1
            pwm = max(-self.max_power, min(self.max_power, pwm))

        self._update_cart_estimate(pwm, dt)
        return pwm

    def compute_action(self, angle_deg: float, dt: float) -> int:
        # No external velocity available, so let compute_action_from_state difference
        # theta itself (velocity=0.0 signals "not measured").
        state_stub = PendulumState(angle_dev=angle_deg, velocity=0.0)
        return self.compute_action_from_state(state_stub, dt)
