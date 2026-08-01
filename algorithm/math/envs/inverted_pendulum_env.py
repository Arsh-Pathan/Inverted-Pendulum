import math
import time
import numpy as np
from typing import Optional, Tuple, Dict, Any

try:
    import gymnasium as gym
    from gymnasium import spaces
    _GYM_AVAILABLE = True
except ImportError:
    try:
        import gym
        from gym import spaces
        _GYM_AVAILABLE = True
    except ImportError:
        _GYM_AVAILABLE = False

try:
    from ...comms.serial_client import SerialClient
except ImportError:
    SerialClient = None

from ...comms.protocol import cmd_motor, cmd_coast

# When Gymnasium is unavailable we still want a usable class, so fall back to `object`.
_EnvBase = gym.Env if _GYM_AVAILABLE else object


class InvertedPendulumEnv(_EnvBase):
    """
    Gymnasium environment for the cart-pole inverted pendulum.

    Two operating modes:
      1. Hardware-in-the-Loop (HIL): drives the ESP32 endpoint over USB serial.
      2. Numerical simulation: non-linear cart-pole physics for offline training.

    ── Sign / coordinate conventions (IMPORTANT) ────────────────────────────────
    `theta` is the pendulum deviation from the UPRIGHT vertical, in radians:
        theta = 0      -> balanced upright
        theta = +-pi   -> hanging straight down
    A positive `theta` means the pole leans toward +x.

    The action is a normalised cart drive command `a` in [-1, 1] which maps to the
    cart's horizontal acceleration. Because the pendulum is driven only through the
    pivot, the rotational equation of motion about the pivot is

        J * theta_ddot = m*g*l*sin(theta) - m*l*a_cart*cos(theta) - b*theta_dot

    Note the MINUS sign on the cart term. This is the "catch-the-fall" physics: to
    arrest a fall toward +theta the cart must accelerate toward +x, i.e. in the SAME
    direction the pole is falling. A positive action therefore *reduces* a positive
    tilt. Getting this sign wrong makes a controller drive the cart away from the
    fall, which is unrecoverable on real hardware.
    """

    metadata = {"render_modes": ["ansi"], "render_fps": 100}

    def __init__(self,
                 serial_port: Optional[str] = None,
                 baud_rate: int = 115200,
                 simulated: bool = True,
                 max_episode_steps: int = 1000,
                 task: str = "balance"):
        self.simulated = simulated or (serial_port is None) or (SerialClient is None)
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.upright_steps = 0
        # "balance" terminates when the pole falls out of the capture basin.
        # "swingup" lets the pole hang and swing freely, so it must not terminate on tilt.
        self.task = task

        # ── Physical constants ──
        self.g = 9.81            # Gravity (m/s^2)
        self.l = 0.16            # Pivot to pendulum centre of mass (m)
        self.m = 0.05            # Pendulum mass (kg)
        self.b = 0.0002          # Viscous pivot damping (N*m*s/rad)
        self.dt = 0.01           # Control / integration step (100 Hz)

        # Moment of inertia about the PIVOT for a uniform rod of length 2l whose COM
        # sits at l:  J = (1/3)*m*(2l)^2 = (4/3)*m*l^2.  This matches the Lagrangian
        # derivation in docs/math/energy.md, which the previous m*l^2 (point mass) did not.
        self.J = (4.0 / 3.0) * self.m * self.l * self.l

        # ── Actuator limits (THE binding constraint on this platform) ──
        # A DC motor's available acceleration falls linearly with speed due to back-EMF:
        #     a(v) = a_stall * (1 - v/v_max)
        # which is exactly the model below with cart_damping = a_stall / v_max. That makes
        # `cart_top_speed` a hard ceiling, not a soft drag term.
        #
        # `cart_top_speed` is the single most important number here: balancing requires the
        # cart to out-run the pole's fall, and a cart that cannot exceed ~0.5 m/s CANNOT
        # balance this pendulum at any gain (see docs/math/ANALYSIS_math_review.md §14).
        # These defaults are deliberately optimistic-but-plausible; MEASURE them on the rig
        # (command M,255 and time the cart across a known distance) before trusting results.
        self.max_cart_accel = 6.0    # a_stall (m/s^2) at full PWM
        self.cart_top_speed = 0.8    # v_max (m/s) - measure this!

        # Track geometry: the physical rail is ~0.40 m of usable travel.
        self.track_limit = 0.20  # +- metres from centre

        # Internal simulation state: [cart_x, cart_v, theta, theta_dot].
        self._sim = np.zeros(4, dtype=np.float64)

        # Observation exposed to the agent. The real rig has an AS5600 on the pendulum
        # pivot ONLY - there is no cart position encoder - so the observation is limited
        # to [theta, theta_dot] to keep simulation and hardware observations identical.
        self.state = np.zeros(2, dtype=np.float32)
        self.serial_client = None

        if _GYM_AVAILABLE:
            high_obs = np.array([math.pi, 25.0], dtype=np.float32)
            self.observation_space = spaces.Box(low=-high_obs, high=high_obs, dtype=np.float32)
            self.action_space = spaces.Box(low=np.array([-1.0], dtype=np.float32),
                                           high=np.array([1.0], dtype=np.float32),
                                           dtype=np.float32)

        if not self.simulated and serial_port and SerialClient:
            self.serial_client = SerialClient(serial_port, baud_rate)
            self.serial_client.start()
            time.sleep(1.0)  # Allow connection settling

    # ────────────────────────────────────────────────────────────────────────────
    @property
    def cart_damping(self) -> float:
        """Back-EMF drag coefficient implied by a_stall and v_max (units 1/s)."""
        return self.max_cart_accel / max(1e-6, self.cart_top_speed)

    @cart_damping.setter
    def cart_damping(self, value: float):
        """Setting damping directly redefines the implied top speed."""
        self.cart_top_speed = self.max_cart_accel / max(1e-6, float(value))

    def _read_hardware_state(self) -> np.ndarray:
        """
        Reads the latest telemetry and converts it to [theta, theta_dot] in radians.

        Firmware tares to the HANGING rest position, so `angle_dev` is 0 when hanging
        and 180 when upright. Upright deviation is therefore `angle_dev - 180`, and its
        time derivative has the SAME sign as the reported angular velocity. (The old
        code used `180 - angle`, which silently negated theta relative to theta_dot.)
        """
        if not self.serial_client:
            return np.zeros(2, dtype=np.float32)

        angle_dev = float(getattr(self.serial_client, "last_angle", 180.0))
        vel_deg_s = float(getattr(self.serial_client, "last_velocity", 0.0))

        theta_deg = (angle_dev % 360.0)
        return np.array([math.radians(theta_deg), math.radians(vel_deg_s)], dtype=np.float32)

    def _integrate(self, norm_action: float):
        """Semi-implicit Euler integration of the non-linear cart-pole dynamics."""
        x, v, theta, omega = self._sim
        a_cart = float(norm_action) * self.max_cart_accel

        # Commanded acceleration minus drag gives the cart's realised acceleration.
        v_dot = a_cart - self.cart_damping * v

        # Rotational EOM about the pivot. The -m*l*v_dot*cos(theta) term is the inertial
        # reaction that lets the cart catch the pole.
        omega_dot = (self.m * self.g * self.l * math.sin(theta)
                     - self.m * self.l * v_dot * math.cos(theta)
                     - self.b * omega) / self.J

        v += v_dot * self.dt
        x += v * self.dt
        omega += omega_dot * self.dt
        theta += omega * self.dt

        # Wrap theta into [-pi, pi]
        theta = (theta + math.pi) % (2.0 * math.pi) - math.pi

        # A rail end-stop is an inelastic collision: kill the cart velocity.
        if abs(x) >= self.track_limit:
            x = math.copysign(self.track_limit, x)
            v = 0.0

        self._sim = np.array([x, v, theta, omega], dtype=np.float64)

    # ────────────────────────────────────────────────────────────────────────────
    def reset(self, seed: Optional[int] = None,
              options: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        if _GYM_AVAILABLE and seed is not None:
            super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        elif not hasattr(self, "_rng"):
            self._rng = np.random.default_rng()

        self.current_step = 0
        self.upright_steps = 0

        start_upright = options.get("start_upright", True) if options else True

        if self.simulated:
            if start_upright:
                theta0 = float(self._rng.uniform(-0.05, 0.05))    # ~ +-3 degrees
                omega0 = float(self._rng.uniform(-0.1, 0.1))
            else:
                # Hanging down (+-pi) for the swing-up task.
                theta0 = math.pi + float(self._rng.uniform(-0.05, 0.05))
                theta0 = (theta0 + math.pi) % (2.0 * math.pi) - math.pi
                omega0 = 0.0
            self._sim = np.array([0.0, 0.0, theta0, omega0], dtype=np.float64)
            self.state = np.array([theta0, omega0], dtype=np.float32)
        else:
            if self.serial_client:
                self.serial_client.send_command(cmd_coast())
                time.sleep(0.05)
                self.state = self._read_hardware_state()

        return self.state.copy(), {}

    def step(self, action) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self.current_step += 1

        arr = np.asarray(action, dtype=np.float64).reshape(-1)
        norm_action = float(np.clip(arr[0], -1.0, 1.0))
        pwm_command = int(round(norm_action * 255.0))

        if self.simulated:
            self._integrate(norm_action)
            _, cart_v, theta, omega = self._sim
            cart_x = float(self._sim[0])
            self.state = np.array([theta, omega], dtype=np.float32)
        else:
            if self.serial_client:
                self.serial_client.send_command(cmd_motor(pwm_command))
                time.sleep(self.dt)
                self.state = self._read_hardware_state()
            theta, omega = float(self.state[0]), float(self.state[1])
            cart_x, cart_v = 0.0, 0.0

        # ── Reward is computed on the state the action actually PRODUCED ──
        # (The previous implementation scored the pre-step state, so every reward and
        # termination signal was attributed to the wrong transition.)
        err_deg = math.degrees(theta)
        vel_deg_s = math.degrees(omega)

        # ── Improved Continuous Reward System ──
        
        # 1. Broad shaping: [0, 1] scale, encourages moving towards upright from any angle
        broad_shaping = (math.cos(theta) + 1.0) / 2.0
        
        # 2. Precision bonus: steep Gaussian peak near upright (std dev ~2.8 degrees)
        # Replaces the discontinuous, non-Markovian 'holding_bonus' with a continuous state-based reward.
        precision_bonus = 10.0 * math.exp(-0.5 * (theta / 0.05)**2)
        
        # Track upright steps for info/logging only, not for reward computation
        in_upright_buffer = bool(abs(err_deg) <= 2.0)
        if in_upright_buffer:
            self.upright_steps += 1
        else:
            self.upright_steps = 0
            
        # 3. Penalties (smooth quadratic costs)
        omega_penalty = 0.1 * (omega ** 2)
        action_penalty = 0.05 * (norm_action ** 2)
        
        # 4. Cart centering: gentle penalty for leaving center, harsh penalty for nearing edges
        cart_centering_penalty = 2.0 * (cart_x / self.track_limit)**2
        cart_edge_penalty = 20.0 * max(0.0, abs(cart_x) - 0.8 * self.track_limit) ** 2
        
        # 5. Spin penalty for out of control swinging
        is_spinning = bool(abs(vel_deg_s) > 360.0)
        spin_penalty = (50.0 + 0.1 * (abs(vel_deg_s) - 360.0)) if is_spinning else 0.0

        reward = broad_shaping + precision_bonus - omega_penalty - action_penalty - cart_centering_penalty - cart_edge_penalty - spin_penalty


        # The balance task terminates once the pole leaves the linear capture basin.
        # The swing-up task must NOT, or it would terminate on its very first step
        # while hanging at +-pi.
        if self.task == "balance":
            terminated = bool(abs(theta) > math.radians(45.0))
        else:
            terminated = False
        # Running off the end of the rail ends the episode in either task.
        terminated = terminated or bool(abs(cart_x) >= self.track_limit)
        truncated = bool(self.current_step >= self.max_episode_steps)

        info = {
            "error_deg": err_deg,
            "velocity_deg_s": vel_deg_s,
            "pwm_command": pwm_command,
            "cart_x": cart_x,
            "cart_v": cart_v,
            "in_upright_buffer": in_upright_buffer,
            "upright_steps": self.upright_steps,
            "upright_duration_s": float(self.upright_steps * self.dt),
            "precision_bonus": precision_bonus,
            "is_spinning": is_spinning,
            "spin_penalty": spin_penalty,
        }

        return self.state.copy(), float(reward), terminated, truncated, info

    def render(self):
        """ASCII terminal rendering of pendulum orientation for headless inspection."""
        theta_deg = math.degrees(self.state[0])
        bar_len = 20
        idx = int((theta_deg + 45.0) / 90.0 * bar_len)
        idx = max(0, min(bar_len - 1, idx))
        bar = ["-"] * bar_len
        bar[idx] = "O"
        print(f"[ENV RENDER] Step {self.current_step:4d} | Upright Dev: {theta_deg:+.1f}deg | [{''.join(bar)}]")

    def close(self):
        if self.serial_client:
            self.serial_client.send_command(cmd_coast())
            self.serial_client.stop_client()
