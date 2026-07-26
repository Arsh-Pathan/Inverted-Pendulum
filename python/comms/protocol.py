"""
Protocol definitions and formatting helpers for the ESP32 Hardware Endpoint.
"""

def cmd_motor(power: int) -> str:
    """Format command to set motor power directly (-255 to +255)."""
    power = max(-255, min(255, int(power)))
    return f"M,{power}\n"

def cmd_forward(speed: int) -> str:
    """Format command to spin motor forward (0 to 255)."""
    speed = max(0, min(255, int(speed)))
    return f"F,{speed}\n"

def cmd_reverse(speed: int) -> str:
    """Format command to spin motor reverse (0 to 255)."""
    speed = max(0, min(255, int(speed)))
    return f"R,{speed}\n"

def cmd_brake() -> str:
    """Format command to hard brake the H-bridge."""
    return "B\n"

def cmd_coast() -> str:
    """Format command to coast (open-circuit free wheel)."""
    return "C\n"

def cmd_set_telemetry_rate(rate_hz: int) -> str:
    """Format command to set telemetry streaming frequency in Hz."""
    rate_hz = max(0, min(1000, int(rate_hz)))
    return f"T,{rate_hz}\n"

def cmd_query_angle() -> str:
    """Format command to query a single immediate angle reading."""
    return "Q\n"

def cmd_zero_tare() -> str:
    """Format command to trigger on-demand zero/tare calibration on hardware."""
    return "Z\n"
