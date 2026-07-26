import math
from typing import Dict, Any
from .base_controller import BaseController
from ..core.state import PendulumState

class SwingUpController(BaseController):
    """
    Energy-Pumping Swing-Up Controller based on the Åström-Furuta Lyapunov energy law.
    Pumps kinetic energy into a hanging pendulum until it reaches the vertical capture
    basin of an upright stabilizer (PID or LQR).
    """
    def __init__(self, pump_power: int = 200, energy_gain: float = 5.0):
        super().__init__("Energy Swing-Up")
        self.pump_power = max(0, min(255, pump_power))
        self.energy_gain = energy_gain

    def reset(self):
        pass

    def update_params(self, params: Dict[str, Any]):
        if "pump_power" in params: self.pump_power = max(0, min(255, int(params["pump_power"])))
        if "energy_gain" in params: self.energy_gain = float(params["energy_gain"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled:
            return 0

        # Angle in radians from bottom equilibrium (0.0 is hanging down)
        theta_rad = math.radians(state.angle_dev)
        vel_rad_s = math.radians(state.velocity)

        # Simplified energy pumping law: u = sign(vel * cos(theta)) * pump_power
        # When moving upward in the lower hemisphere, accelerate in swing direction
        energy_term = vel_rad_s * math.cos(theta_rad)
        
        if abs(energy_term) < 0.05:
            # Give a small initial kick if completely still at bottom
            if abs(state.angle_dev) < 2.0 and abs(state.velocity) < 1.0:
                return self.pump_power
            return 0

        if energy_term > 0:
            return self.pump_power
        else:
            return -self.pump_power

    def compute_action(self, angle_deg: float, dt: float) -> int:
        state_stub = PendulumState(angle_dev=angle_deg, velocity=0.0)
        return self.compute_action_from_state(state_stub, dt)
