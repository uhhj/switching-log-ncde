from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Tuple

import numpy as np


@dataclass
class EndpointServo:
    start_x: float
    y_target: float
    forward_speed: float
    kp: float
    kd: float
    force_limit: float

    @classmethod
    def from_config(
        cls,
        start_position,
        y_target: float,
        config: Mapping[str, Any],
    ) -> "EndpointServo":
        values = config["controller"]
        return cls(
            start_x=float(start_position[0]),
            y_target=float(y_target),
            forward_speed=float(values["forward_speed_mps"]),
            kp=float(values["position_kp_npm"]),
            kd=float(values["velocity_kd_ns_per_m"]),
            force_limit=float(values["force_limit_n"]),
        )

    def command(
        self, elapsed_s: float, position, velocity, active: bool
    ) -> Tuple[np.ndarray, float, float, bool]:
        if not active:
            return np.zeros(3), 0.0, 0.0, False
        target = np.asarray(
            [
                self.start_x + self.forward_speed * float(elapsed_s),
                self.y_target,
            ],
            dtype=np.float64,
        )
        desired_velocity = np.asarray([self.forward_speed, 0.0])
        force_xy = self.kp * (target - np.asarray(position[:2])) + self.kd * (
            desired_velocity - np.asarray(velocity[:2])
        )
        norm = float(np.linalg.norm(force_xy))
        if norm > self.force_limit:
            force_xy *= self.force_limit / norm
        return (
            np.asarray([force_xy[0], force_xy[1], 0.0]),
            self.forward_speed,
            self.y_target,
            True,
        )
