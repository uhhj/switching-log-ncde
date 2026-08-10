from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

import numpy as np


@dataclass
class EndpointServo:
    start_x: float
    y_target: float
    forward_speed: float
    kp: float
    kd: float
    force_limit: float
    max_reference_travel: Optional[float] = None

    @classmethod
    def from_config(
        cls,
        start_position,
        y_target: float,
        config: Mapping[str, Any],
        max_reference_travel: Optional[float] = None,
    ) -> "EndpointServo":
        values = config["controller"]
        return cls(
            start_x=float(start_position[0]),
            y_target=float(y_target),
            forward_speed=float(values["forward_speed_mps"]),
            kp=float(values["position_kp_npm"]),
            kd=float(values["velocity_kd_ns_per_m"]),
            force_limit=float(values["force_limit_n"]),
            max_reference_travel=(
                None
                if max_reference_travel is None
                else float(max_reference_travel)
            ),
        )

    def reference_travel(self, elapsed_s: float) -> float:
        travel = self.forward_speed * max(0.0, float(elapsed_s))
        if self.max_reference_travel is not None:
            travel = min(travel, self.max_reference_travel)
        return float(travel)

    def command(
        self,
        elapsed_s: float,
        position,
        velocity,
        active: bool,
        hold_reference: bool = False,
    ) -> Tuple[np.ndarray, float, float, bool]:
        if not active and not hold_reference:
            return np.zeros(3), 0.0, 0.0, False
        reference_travel = self.reference_travel(elapsed_s)
        target = np.asarray(
            [
                self.start_x + reference_travel,
                self.y_target,
            ],
            dtype=np.float64,
        )
        target_is_moving = bool(
            active
            and (
                self.max_reference_travel is None
                or reference_travel < self.max_reference_travel
            )
        )
        desired_velocity = np.asarray(
            [self.forward_speed if target_is_moving else 0.0, 0.0]
        )
        force_xy = self.kp * (target - np.asarray(position[:2])) + self.kd * (
            desired_velocity - np.asarray(velocity[:2])
        )
        norm = float(np.linalg.norm(force_xy))
        if norm > self.force_limit:
            force_xy *= self.force_limit / norm
        return (
            np.asarray([force_xy[0], force_xy[1], 0.0]),
            self.forward_speed if active else 0.0,
            self.y_target if active else 0.0,
            bool(active),
        )
