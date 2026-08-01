from abc import ABC, abstractmethod
from typing import Dict, Any
from ..core.state import PendulumState

class BaseController(ABC):
    """
    Abstract base class for all inverted pendulum controllers.
    Custom classical algorithms or Reinforcement Learning policies should inherit from this.
    """
    def __init__(self, name: str):
        self.name = name
        self.enabled = False

    def enable(self):
        """Enables the controller and resets internal state."""
        self.enabled = True
        self.reset()

    def disable(self):
        """Disables the controller."""
        self.enabled = False

    @abstractmethod
    def reset(self):
        """Resets internal integrators, timers, or hidden states."""
        pass

    @abstractmethod
    def compute_action(self, angle_deg: float, dt: float) -> int:
        """
        Computes the motor control action based on current angle and time delta.
        
        Args:
            angle_deg (float): Current calibrated angular deviation [0.0, 360.0).
            dt (float): Time delta in seconds since last loop execution.
            
        Returns:
            int: Motor power command between -255 and +255 (0 = coast/brake).
        """
        pass

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        """
        Computes the motor control action from a structured PendulumState snapshot.
        Defaults to delegating to compute_action(state.angle_dev, dt).
        """
        return self.compute_action(state.angle_dev, dt)

    @staticmethod
    def apply_deadband_bias(command: float, min_power: int, max_power: int,
                            ramp_width: float = 40.0) -> int:
        """
        Applies a smooth stiction-compensation bias to a PWM-like command.

        Tiny commands coast, commands inside the ramp get a proportional bias, and
        larger commands receive the full minimum-power offset before saturation.
        """
        if abs(command) <= 2.0:
            return 0

        sign = 1 if command > 0.0 else -1
        magnitude = abs(float(command))
        bias = min_power if magnitude >= ramp_width else min_power * magnitude / ramp_width
        pwm = sign * (magnitude + bias)
        return int(max(-max_power, min(max_power, round(pwm))))

    def update_params(self, params: Dict[str, Any]):
        """Optional hook to update gain parameters or hyperparameters live."""
        pass
