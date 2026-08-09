from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Sequence

import numpy as np
import pybullet as p


def canonical_active_endpoint_index(num_beads: int) -> int:
    num_beads = int(num_beads)
    if num_beads <= 0:
        raise ValueError("num_beads must be positive")
    return num_beads - 1


def endpoint_local_indices(num_beads: int, endpoint_index: int, count: int):
    num_beads = int(num_beads)
    endpoint_index = int(endpoint_index)
    count = int(count)
    if count <= 0 or count > num_beads:
        raise ValueError("count must be between one and num_beads")
    if endpoint_index == 0:
        return list(range(count))
    if endpoint_index == num_beads - 1:
        return list(range(num_beads - 1, num_beads - 1 - count, -1))
    raise ValueError("endpoint_index must be a cable endpoint")


def local_segment_diagnostics_from_state(
    positions: np.ndarray,
    velocities: np.ndarray,
    entry_center: Sequence[float],
    insertion_axis: Sequence[float],
    lateral_axis: Sequence[float],
    nominal_spacing_m: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    positions = np.asarray(positions, dtype=np.float64)
    velocities = np.asarray(velocities, dtype=np.float64)
    entry = np.asarray(entry_center, dtype=np.float64)
    insertion = np.asarray(insertion_axis, dtype=np.float64)
    lateral = np.asarray(lateral_axis, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("positions must have shape (N, 3)")
    if velocities.shape != positions.shape:
        raise ValueError("velocities must match positions")

    signed = (positions - entry) @ insertion
    segments = positions[:-1] - positions[1:]
    norms = np.linalg.norm(segments, axis=1)
    cosines = np.full(norms.shape, -1.0, dtype=np.float64)
    valid = norms > 1e-12
    cosines[valid] = (segments[valid] @ insertion) / norms[valid]
    speeds = np.linalg.norm(velocities, axis=1)
    lateral_offsets = (positions - entry) @ lateral
    if nominal_spacing_m is None:
        spacing_errors = np.zeros_like(norms)
    else:
        nominal_spacing = float(nominal_spacing_m)
        if nominal_spacing <= 0.0:
            raise ValueError("nominal_spacing_m must be positive")
        spacing_errors = np.abs(norms - nominal_spacing)
    return {
        "positions": positions,
        "velocities": velocities,
        "signed_entry_distances_m": signed,
        "max_signed_entry_distance_m": np.asarray(float(np.max(signed))),
        "alignment_cosines": cosines,
        "median_alignment_cosine": np.asarray(float(np.median(cosines))),
        "local_speeds_mps": speeds,
        "max_local_speed_mps": np.asarray(float(np.max(speeds))),
        "lateral_offsets_m": lateral_offsets,
        "segment_lengths_m": norms,
        "spacing_errors_m": spacing_errors,
        "max_local_spacing_error_m": np.asarray(
            float(np.max(spacing_errors)) if spacing_errors.size else 0.0
        ),
        "median_local_spacing_error_m": np.asarray(
            float(np.median(spacing_errors)) if spacing_errors.size else 0.0
        ),
    }


def local_segment_diagnostics(
    task,
    indices: Sequence[int],
    entry_center: Sequence[float],
    insertion_axis: Sequence[float],
    lateral_axis: Sequence[float],
    nominal_spacing_m: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    bead_ids = list(task.cable_bead_IDs)
    positions = []
    velocities = []
    for index in indices:
        bead_id = int(bead_ids[int(index)])
        positions.append(p.getBasePositionAndOrientation(bead_id)[0])
        velocities.append(p.getBaseVelocity(bead_id)[0])
    return local_segment_diagnostics_from_state(
        np.asarray(positions, dtype=np.float64),
        np.asarray(velocities, dtype=np.float64),
        entry_center,
        insertion_axis,
        lateral_axis,
        nominal_spacing_m,
    )


def local_geometry_passes(
    diagnostics: Mapping[str, np.ndarray], preparation: Mapping[str, Any]
) -> bool:
    spacing_ok = (
        "max_local_spacing_error_m" not in preparation
        or float(diagnostics["max_local_spacing_error_m"])
        <= float(preparation["max_local_spacing_error_m"])
    )
    return bool(
        float(diagnostics["max_signed_entry_distance_m"])
        <= -float(preparation["entry_clearance_margin_m"])
        and float(diagnostics["median_alignment_cosine"])
        >= float(preparation["alignment_cosine_min"])
        and float(diagnostics["max_local_speed_mps"])
        <= float(preparation["max_local_speed_mps"])
        and spacing_ok
    )
