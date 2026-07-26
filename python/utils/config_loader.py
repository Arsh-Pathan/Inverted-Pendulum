import os
import json

def get_project_root():
    """Returns the absolute path to the project root directory."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # python/utils -> python -> project root
    return os.path.abspath(os.path.join(current_dir, "..", ".."))

def get_config_path():
    """Returns the path to config/default_config.json."""
    return os.path.join(get_project_root(), "config", "default_config.json")

def load_config():
    """Loads default_config.json, returning default dict if file missing or corrupted."""
    config_path = get_config_path()
    defaults = {
        "serial": {
            "preferred_port": "COM3",
            "baud_rate": 115200,
            "timeout": 0.01,
            "telemetry_rate_hz": 100
        },
        "encoder": {
            "angle_offset": 0.0,
            "angle_scale": 1.0,
            "angle_invert": False
        },
        "control": {
            "kp": 15.0,
            "ki": 0.0,
            "kd": 2.5,
            "alpha": 0.08,
            "control_loop_rate_hz": 100,
            "min_motor_power": 45,
            "max_motor_power": 255,
            "equilibrium_deadzone_deg": 0.4,
            "equilibrium_deadzone_vel": 6.0
        },
        "oscillation": {
            "speed": 255,
            "duration_ms": 400
        }
    }

    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                # Deep merge defaults with loaded data
                for section, values in data.items():
                    if section in defaults and isinstance(values, dict):
                        defaults[section].update(values)
                    else:
                        defaults[section] = values
        except Exception as e:
            print(f"[CONFIG ERROR] Failed reading {config_path}: {e}")
    else:
        save_config(defaults)

    return defaults

def save_config(config_data):
    """Saves the provided dictionary to config/default_config.json."""
    config_path = get_config_path()
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    try:
        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(config_data, f, indent=4)
        print(f"[CONFIG] Successfully saved configurations to {config_path}")
    except Exception as e:
        print(f"[CONFIG ERROR] Failed saving {config_path}: {e}")
