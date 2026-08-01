from dataclasses import dataclass

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
    def theta_from_upright(self) -> float:
        """
        CANONICAL control coordinate: signed tilt in degrees away from upright (180.0°).

        theta = angle_dev - 180, wrapped to [-180, +180]:
            0    -> balanced upright
            +-180 -> hanging straight down
        Positive theta means the pole is leaning the same way that a positive
        reported `velocity` moves it, so `theta` and `velocity` form a consistent
        (position, derivative) pair: d(theta)/dt == velocity.

        Control laws MUST use this rather than `error_from_upright`, whose sign is
        opposite to `velocity` and therefore turns any velocity/derivative gain into
        destabilising positive feedback.
        """
        theta = (self.angle_dev - 180.0) % 360.0
        if theta > 180.0:
            theta -= 360.0
        return theta

    @property
    def error_from_upright(self) -> float:
        """
        Angular error to drive to zero, defined as (180 - angle_dev), i.e. -theta.

        DEPRECATED for control use: this is the negative of `theta_from_upright`, so
        pairing it with the raw `velocity` field mixes two opposite sign conventions.
        Retained only for display/logging continuity.
        """
        return -self.theta_from_upright

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
