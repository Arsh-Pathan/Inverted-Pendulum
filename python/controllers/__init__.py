"""
Control algorithms and modular balancing engines.
"""
from .base_controller import BaseController
from .pid_balancer import PIDBalancer
from .lqr_balancer import LQRBalancer
from .swing_up import SwingUpController
from .hybrid_balancer import HybridBalancer
from .oscillation import OscillationController

try:
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
    from rl.rl_controller import RLBalancer
except ImportError:
    RLBalancer = None

__all__ = [
    "BaseController",
    "PIDBalancer",
    "LQRBalancer",
    "SwingUpController",
    "HybridBalancer",
    "OscillationController",
    "RLBalancer"
]
