import csv
import os
import time
from typing import Any, Dict, Iterable

import numpy as np

try:
    from stable_baselines3.common.callbacks import BaseCallback
except ImportError:
    BaseCallback = object


class TrainingDataCallback(BaseCallback):
    """Writes per-step RL training transitions to CSV."""

    def __init__(self, csv_path: str, verbose: int = 0):
        super().__init__(verbose)
        self.csv_path = csv_path
        self._file = None
        self._writer = None
        self._episode = 0
        self._episode_step = 0
        self._start_time = time.time()

    def _on_training_start(self) -> None:
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self._file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "wall_time_s",
                "timestep",
                "episode",
                "episode_step",
                "theta_rad",
                "theta_dot_rad_s",
                "action",
                "reward",
                "done",
                "cart_x",
                "cart_v",
                "error_deg",
                "velocity_deg_s",
                "pwm_command",
            ],
        )
        self._writer.writeheader()

    def _on_step(self) -> bool:
        if self._writer is None:
            return True

        observations = self._as_rows(self.locals.get("new_obs"))
        actions = self._as_rows(self.locals.get("actions"))
        rewards = np.asarray(self.locals.get("rewards", []), dtype=np.float64).reshape(-1)
        dones = np.asarray(self.locals.get("dones", []), dtype=bool).reshape(-1)
        infos = self.locals.get("infos", [{}])

        row_count = max(len(observations), len(actions), len(rewards), len(dones), len(infos))
        for index in range(row_count):
            obs = observations[index] if index < len(observations) else []
            action = actions[index] if index < len(actions) else []
            info = infos[index] if index < len(infos) else {}
            done = bool(dones[index]) if index < len(dones) else False

            self._writer.writerow(
                {
                    "wall_time_s": f"{time.time() - self._start_time:.6f}",
                    "timestep": self.num_timesteps,
                    "episode": self._episode,
                    "episode_step": self._episode_step,
                    "theta_rad": self._field(obs, 0),
                    "theta_dot_rad_s": self._field(obs, 1),
                    "action": self._field(action, 0),
                    "reward": self._field(rewards, index),
                    "done": int(done),
                    "cart_x": self._info(info, "cart_x"),
                    "cart_v": self._info(info, "cart_v"),
                    "error_deg": self._info(info, "error_deg"),
                    "velocity_deg_s": self._info(info, "velocity_deg_s"),
                    "pwm_command": self._info(info, "pwm_command"),
                }
            )

            self._episode_step += 1
            if done:
                self._episode += 1
                self._episode_step = 0

        return True

    def _on_training_end(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    @staticmethod
    def _as_rows(value: Any) -> list:
        if value is None:
            return []
        array = np.asarray(value)
        if array.ndim == 0:
            return [[array.item()]]
        if array.ndim == 1:
            return [array.tolist()]
        return array.reshape((array.shape[0], -1)).tolist()

    @staticmethod
    def _field(values: Iterable, index: int) -> str:
        try:
            return f"{float(values[index]):.9g}"
        except (IndexError, TypeError, ValueError):
            return ""

    @staticmethod
    def _info(info: Dict[str, Any], key: str) -> str:
        value = info.get(key, "")
        if value == "":
            return ""
        try:
            return f"{float(value):.9g}"
        except (TypeError, ValueError):
            return str(value)
