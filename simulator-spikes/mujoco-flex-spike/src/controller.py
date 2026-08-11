from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

import numpy as np


def bounded_impedance_force(
    position,
    velocity,
    target_position,
    target_velocity,
    stiffness,
    damping,
    force_limit,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = np.asarray(position, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    target_velocity = np.asarray(target_velocity, dtype=np.float64)
    stiffness = np.asarray(stiffness, dtype=np.float64)
    damping = np.asarray(damping, dtype=np.float64)
    force_limit = np.asarray(force_limit, dtype=np.float64)
    raw = stiffness * (target_position - position) + damping * (
        target_velocity - velocity
    )
    clipped = np.clip(raw, -force_limit, force_limit)
    return clipped, raw, np.abs(raw) > force_limit


def lateral_control_active(
    leading4_mean_x: float,
    funnel_entry_x: float,
    local_spacing_m: float,
    activation_before_funnel_spacing: float,
) -> bool:
    activation_x = float(funnel_entry_x) - float(
        activation_before_funnel_spacing
    ) * float(local_spacing_m)
    return bool(float(leading4_mean_x) >= activation_x)


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


@dataclass
class BoundedCartesianImpedance:
    start_x: float
    y_target: float
    forward_speed: float
    stiffness_xy: np.ndarray
    damping_xy: np.ndarray
    force_limit_xy: np.ndarray
    max_reference_travel: float

    @classmethod
    def from_config(
        cls,
        start_position,
        y_target: float,
        config: Mapping[str, Any],
        max_reference_travel: float,
    ) -> "BoundedCartesianImpedance":
        values = config["controller"]
        return cls(
            start_x=float(start_position[0]),
            y_target=float(y_target),
            forward_speed=float(values["forward_speed_mps"]),
            stiffness_xy=np.asarray(
                [values["position_kp_npm"], values["position_kp_npm"]],
                dtype=np.float64,
            ),
            damping_xy=np.asarray(
                [
                    values["velocity_kd_ns_per_m"],
                    values["velocity_kd_ns_per_m"],
                ],
                dtype=np.float64,
            ),
            force_limit_xy=np.asarray(
                [values["force_limit_n"], values["force_limit_n"]],
                dtype=np.float64,
            ),
            max_reference_travel=float(max_reference_travel),
        )

    def reference_travel(self, elapsed_s: float) -> float:
        return float(
            min(
                self.forward_speed * max(0.0, float(elapsed_s)),
                self.max_reference_travel,
            )
        )

    def command(
        self,
        elapsed_s: float,
        position,
        velocity,
        active: bool,
        lateral_active: bool,
        hold_reference: bool = False,
    ) -> Tuple[np.ndarray, float, float, bool, Mapping[str, Any]]:
        reference_travel = self.reference_travel(elapsed_s)
        target_position = np.asarray(
            [self.start_x + reference_travel, self.y_target], dtype=np.float64
        )
        target_is_moving = bool(
            active and reference_travel < self.max_reference_travel
        )
        target_velocity = np.asarray(
            [self.forward_speed if target_is_moving else 0.0, 0.0],
            dtype=np.float64,
        )
        if not active and not hold_reference:
            force_xy = np.zeros(2, dtype=np.float64)
            raw_xy = np.zeros(2, dtype=np.float64)
            saturated_xy = np.zeros(2, dtype=bool)
        else:
            force_xy, raw_xy, saturated_xy = bounded_impedance_force(
                np.asarray(position[:2], dtype=np.float64),
                np.asarray(velocity[:2], dtype=np.float64),
                target_position,
                target_velocity,
                self.stiffness_xy,
                self.damping_xy,
                self.force_limit_xy,
            )
            if not lateral_active:
                force_xy[1] = 0.0
                raw_xy[1] = 0.0
                saturated_xy[1] = False
        diagnostics = {
            "target_position": target_position,
            "target_velocity": target_velocity,
            "tracking_error": target_position
            - np.asarray(position[:2], dtype=np.float64),
            "raw_force": raw_xy,
            "force_saturated": saturated_xy,
            "lateral_control_active": bool(lateral_active),
        }
        return (
            np.asarray([force_xy[0], force_xy[1], 0.0]),
            self.forward_speed if active else 0.0,
            self.y_target if active else 0.0,
            bool(active),
            diagnostics,
        )
