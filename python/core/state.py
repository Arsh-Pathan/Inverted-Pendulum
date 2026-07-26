from dataclasses import dataclass, field
import time

@dataclass
class PendulumState:
    """
    Immutable snapshot of the physical inverted pendulum state at a single timestamp.
    Provides clean, type-safe data encapsulation for control algorithms, logging, and UI.
    """
    timestamp: float = 0.0          # Epoch seconds
    raw_angle: float = 0.0          # Raw sensor reading [0.0, 360.0)
    angle_dev: float = 0.0          # Calibrated angular deviation from zero [0.0, 360.0)
    velocity: float = 0.0           # Angular velocity in deg/s
    control_output: int = 0         # Last motor power applied [-255, +255]
    sample_rate_hz: float = 0.0     # Observed telemetry stream frequency

    @property
    def error_from_upright(self) -> float:
        """
        Returns the angular error in degrees relative to the vertical upright position (180.0°).
        Wrapped smoothly into the shortest path [-180.0, +180.0].
        """
        err = 180.0 - self.angle_dev
        while err > 180.0:
            err -= 360.0
        while err < -180.0:
            err += 360.0
        return err

    @property
    def is_above_horizontal(self) -> bool:
        """Returns True if the pendulum is in the upper hemisphere (90° to 270°)."""
        norm = self.angle_dev % 360.0
        return 90.0 < norm < 270.0

    @property
    def is_near_upright(self) -> bool:
        """Returns True if the pendulum is within ±20° of vertical upright."""
        return abs(self.error_from_upright) <= 20.0

    def to_dict(self) -> dict:
        """Converts state to dictionary for JSON or CSV serialization."""
        return {
            "timestamp": f"{self.timestamp:.6f}",
            "raw_angle": f"{self.raw_angle:.2f}",
            "angle_dev": f"{self.angle_dev:.2f}",
            "velocity": f"{self.velocity:.2f}",
            "error_from_upright": f"{self.error_from_upright:.2f}",
            "control_output": self.control_output,
            "sample_rate_hz": f"{self.sample_rate_hz:.1f}"
        }
