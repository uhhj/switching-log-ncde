from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping

import numpy as np


def required_success_translation(
    leading_x0: Iterable[float], success_plane_x: float
) -> float:
    """Geometric lower bound for translating every leading vertex past success."""
    values = np.asarray(leading_x0, dtype=np.float64)
    if values.shape != (4,):
        raise ValueError("leading_x0 must contain exactly four x coordinates")
    return max(0.0, float(np.max(float(success_plane_x) - values)))


def ordered_local_spacing(ordered_vertices: np.ndarray) -> float:
    vertices = np.asarray(ordered_vertices, dtype=np.float64)
    if vertices.ndim != 2 or vertices.shape[0] < 2 or vertices.shape[1] != 3:
        raise ValueError("ordered_vertices must have shape (V, 3), V >= 2")
    return float(np.median(np.linalg.norm(np.diff(vertices, axis=0), axis=1)))


def compute_drive_protocol(
    common_states: Iterable[Mapping[str, Any]],
    success_plane_x: float,
    forward_speed_mps: float,
    local_spacing_m: float,
    safety_spacing_count: float = 2.0,
    max_extra_time_s: float = 0.75,
) -> Dict[str, float]:
    speed = float(forward_speed_mps)
    if speed <= 0.0:
        raise ValueError("forward_speed_mps must be positive")
    states = list(common_states)
    if not states:
        raise ValueError("at least one common state is required")
    required = max(
        required_success_translation(
            np.asarray(state["leading_positions"], dtype=np.float64)[:, 0],
            success_plane_x,
        )
        for state in states
    )
    safety = float(safety_spacing_count) * float(local_spacing_m)
    command_distance = required + safety
    nominal_drive_time = command_distance / speed
    return {
        "required_translation_m": required,
        "safety_m": safety,
        "command_distance_m": command_distance,
        "nominal_drive_time_s": nominal_drive_time,
        "max_duration_s": nominal_drive_time + float(max_extra_time_s),
    }
