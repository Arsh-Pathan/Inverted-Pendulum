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
    from ..comms.serial_client import SerialClient
except ImportError:
    SerialClient = None

from ..core.state import PendulumState
from ..comms.protocol import cmd_motor, cmd_coast

class InvertedPendulumEnv:
    """
    OpenAI Gym / Gymnasium Environment for the Inverted Pendulum.
    Supports two operating modes:
      1. Hardware-in-the-Loop (HIL): Connects directly to the ESP32 endpoint via USB serial.
      2. Numerical Simulation Mode: Uses non-linear Euler physics integration for offline training/testing.
    """
    def __init__(self,
                 serial_port: Optional[str] = None,
                 baud_rate: int = 115200,
                 simulated: bool = True,
                 max_episode_steps: int = 1000):
        self.simulated = simulated or (serial_port is None) or (SerialClient is None)
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.upright_steps = 0
        
        # Physical simulation constants
        self.g = 9.81           # Gravity (m/s^2)
        self.l = 0.16           # Pendulum pole length to COM (m)
        self.m = 0.05           # Bob mass (kg)
        self.b = 0.0002         # Viscous rotational damping (N*m*s/rad)
        self.kt = 0.0015        # Motor torque conversion factor (N*m / PWM_unit)
        self.dt = 0.01          # Simulation / control step (10 ms = 100 Hz)

        # State vector: [angle_from_upright_rad, angular_velocity_rad_s]
        self.state = np.zeros(2, dtype=np.float32)
        self.serial_client = None

        if _GYM_AVAILABLE:
            # Observation space: [-pi, pi] angle error, [-max_speed, max_speed] angular velocity
            high_obs = np.array([math.pi, 25.0], dtype=np.float32)
            self.observation_space = spaces.Box(low=-high_obs, high=high_obs, dtype=np.float32)
            # Action space: normalized continuous command [-1.0, 1.0] (maps to -255..+255 PWM)
            self.action_space = spaces.Box(low=np.array([-1.0]), high=np.array([1.0]), dtype=np.float32)

        if not self.simulated and serial_port and SerialClient:
            self.serial_client = SerialClient(serial_port, baud_rate)
            self.serial_client.start()
            time.sleep(1.0) # Allow connection settling

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        self.current_step = 0
        self.upright_steps = 0
        if seed is not None:
            np.random.seed(seed)

        if self.simulated:
            # Start with small random perturbation around upright (0.0 rad error) or hanging (-pi)
            start_upright = options.get("start_upright", True) if options else True
            if start_upright:
                init_err = np.random.uniform(-0.1, 0.1) # ~±5 degrees
                init_vel = np.random.uniform(-0.2, 0.2)
            else:
                init_err = math.pi + np.random.uniform(-0.1, 0.1) # Hanging down
                init_vel = 0.0
            self.state = np.array([init_err, init_vel], dtype=np.float32)
        else:
            if self.serial_client:
                self.serial_client.send_command(cmd_coast())
                time.sleep(0.05)
                # Read latest state from telemetry
                err_rad = math.radians(180.0 - getattr(self.serial_client, 'last_angle', 180.0))
                self.state = np.array([err_rad, 0.0], dtype=np.float32)

        return self.state.copy(), {}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self.current_step += 1
        
        # Unpack and scale action from [-1.0, 1.0] to [-255, 255] integer PWM
        norm_action = float(np.clip(action[0] if isinstance(action, (np.ndarray, list)) else action, -1.0, 1.0))
        pwm_command = int(norm_action * 255.0)

        if self.simulated:
            # Non-linear Euler integration of pendulum dynamics
            # theta_err = 0 is upright vertical. Equation of motion around upright:
            # J * d2(theta)/dt2 = m * g * l * sin(theta_err) - b * d(theta)/dt + torque
            theta_err, vel = self.state[0], self.state[1]
            torque = pwm_command * self.kt
            
            accel = (self.m * self.g * self.l * math.sin(theta_err) - self.b * vel + torque) / (self.m * self.l * self.l)
            new_vel = vel + accel * self.dt
            new_theta = theta_err + new_vel * self.dt
            
            # Wrap angle into [-pi, pi]
            new_theta = (new_theta + math.pi) % (2.0 * math.pi) - math.pi
            self.state = np.array([new_theta, new_vel], dtype=np.float32)
        else:
            if self.serial_client:
                self.serial_client.send_command(cmd_motor(pwm_command))
                time.sleep(self.dt)
                # Fetch hardware observation
                err_rad = math.radians(180.0 - getattr(self.serial_client, 'last_angle', 180.0))
                vel_rad = math.radians(getattr(self.serial_client, 'last_velocity', 0.0))
                self.state = np.array([err_rad, vel_rad], dtype=np.float32)

        # Upright Holding Buffer [179°, 181°]: ±1.0 degree around 180° upright
        err_deg = math.degrees(theta_err)
        vel_deg_s = math.degrees(vel)
        in_upright_buffer = bool(abs(err_deg) <= 1.0)
        
        if in_upright_buffer:
            self.upright_steps += 1
            # Progressive time bonus: base 10.0 + 0.1 per continuous step (up to +50.0 max)
            holding_bonus = 10.0 + min(40.0, 0.1 * float(self.upright_steps))
        else:
            self.upright_steps = 0
            holding_bonus = 0.0

        # Spin Penalty: strongly penalize high angular velocities and spinning (> 360 deg/s)
        is_spinning = bool(abs(vel_deg_s) > 360.0)
        spin_penalty = 20.0 if is_spinning else 0.0

        # Reward function: holding bonus minus spin penalty minus quadratic tracking costs
        reward = holding_bonus - spin_penalty - (float(theta_err**2) + 0.2 * float(vel**2) + 0.001 * float(norm_action**2))
        
        # Check termination (if pendulum falls beyond ±45 degrees in stabilize task)
        terminated = bool(abs(theta_err) > math.radians(45.0))
        truncated = bool(self.current_step >= self.max_episode_steps)

        info = {
            "error_deg": err_deg,
            "velocity_deg_s": vel_deg_s,
            "pwm_command": pwm_command,
            "in_upright_buffer": in_upright_buffer,
            "upright_steps": self.upright_steps,
            "upright_duration_s": float(self.upright_steps * self.dt),
            "holding_bonus": holding_bonus,
            "is_spinning": is_spinning,
            "spin_penalty": spin_penalty
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
        print(f"[ENV RENDER] Step {self.current_step:4d} | Upright Dev: {theta_deg:+.1f}° | [{'' .join(bar)}]")

    def close(self):
        if self.serial_client:
            self.serial_client.send_command(cmd_coast())
            self.serial_client.stop_client()
