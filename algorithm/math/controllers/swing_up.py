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
    def __init__(self, pump_power: int = 200, energy_gain: float = 5.0,
                 m: float = 0.05, l: float = 0.16, g: float = 9.81):
        super().__init__("Energy Swing-Up")
        self.pump_power = max(0, min(255, pump_power))
        self.energy_gain = energy_gain
        # Physical parameters, needed to evaluate the true energy error.
        self.m = m
        self.l = l
        self.g = g
        # Inertia about the pivot for a uniform rod with COM at l: J = (4/3) m l^2.
        self.J = (4.0 / 3.0) * m * l * l

    def reset(self):
        pass

    def update_params(self, params: Dict[str, Any]):
        if "pump_power" in params: self.pump_power = max(0, min(255, int(params["pump_power"])))
        if "energy_gain" in params: self.energy_gain = float(params["energy_gain"])

    def compute_action_from_state(self, state: PendulumState, dt: float) -> int:
        if not self.enabled:
            return 0

        # `phi` measures from the HANGING equilibrium (0 = hanging, +-pi = upright),
        # which is the natural coordinate for an energy argument.
        phi = math.radians(((state.angle_dev + 180.0) % 360.0) - 180.0)
        omega = math.radians(state.velocity)

        # Åström-Furuta energy pumping. Energy relative to hanging rest:
        #   E     = 1/2 J omega^2 + m g l (1 - cos(phi))
        #   E_top = 2 m g l                (the upright energy target)
        # The control law u = k * Etilde * omega * cos(phi) drives E -> E_top, giving
        # Vdot = -k (Etilde omega cos phi)^2 <= 0.
        #
        # The previous implementation used sign(omega * cos(phi)) with NO energy error
        # term, so it pumped energy without bound and never stopped at the top - the
        # pendulum just span continuously, which is exactly the failure the RL spin
        # penalty was trying to paper over.
        energy = 0.5 * self.J * omega * omega + self.m * self.g * self.l * (1.0 - math.cos(phi))
        energy_target = 2.0 * self.m * self.g * self.l
        energy_error = energy - energy_target

        drive = self.energy_gain * energy_error * omega * math.cos(phi)

        # Kick-start from dead rest: at phi=0 with omega=0 the law yields exactly 0.
        if abs(omega) < 0.05 and abs(phi) < math.radians(2.0):
            return self.pump_power

        if abs(drive) < 1e-6:
            return 0

        # Pump when short of the target energy, brake when past it. `drive` already
        # carries the correct sign for removing energy, so invert it to ADD energy.
        power = -drive
        magnitude = min(1.0, abs(power) / max(1e-9, energy_target))
        speed = int(magnitude * self.pump_power)
        speed = max(0, min(self.pump_power, speed))
        if speed == 0:
            return 0
        return speed if power > 0 else -speed

    def compute_action(self, angle_deg: float, dt: float) -> int:
        state_stub = PendulumState(angle_dev=angle_deg, velocity=0.0)
        return self.compute_action_from_state(state_stub, dt)
