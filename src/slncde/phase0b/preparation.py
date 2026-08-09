from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

import numpy as np
import pybullet as p


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


def canonical_local_targets(
    staging_xyz: Sequence[float],
    insertion_axis: Sequence[float],
    spacing_m: float,
    count: int,
) -> np.ndarray:
    staging = np.asarray(staging_xyz, dtype=np.float64)
    axis = np.asarray(insertion_axis, dtype=np.float64)
    spacing = float(spacing_m)
    if staging.shape != (3,) or axis.shape != (3,):
        raise ValueError("staging_xyz and insertion_axis must be XYZ")
    if spacing <= 0.0:
        raise ValueError("spacing_m must be positive")
    return np.asarray(
        [staging - rank * spacing * axis for rank in range(int(count))],
        dtype=np.float64,
    )


def canonicalize_local_segment(
    task,
    endpoint_index: int,
    staging_xyz: Sequence[float],
    insertion_axis: Sequence[float],
    local_bead_count: int,
) -> Dict[str, Any]:
    bead_ids = list(task.cable_bead_IDs)
    indices = endpoint_local_indices(
        len(bead_ids), endpoint_index, int(local_bead_count)
    )
    spacing = float(task.length) / float(task.num_parts)
    if spacing <= 0.0:
        raise ValueError("nominal bead spacing must be positive")
    targets = canonical_local_targets(
        staging_xyz, insertion_axis, spacing, len(indices)
    )
    for index, target in zip(indices, targets):
        bead_id = int(bead_ids[index])
        _, orientation = p.getBasePositionAndOrientation(bead_id)
        p.resetBasePositionAndOrientation(
            bead_id, target.tolist(), orientation
        )
        p.resetBaseVelocity(
            bead_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
        )
    return {
        "indices": indices,
        "spacing_m": spacing,
        "targets": targets,
    }


def local_segment_diagnostics_from_state(
    positions: np.ndarray,
    velocities: np.ndarray,
    entry_center: Sequence[float],
    insertion_axis: Sequence[float],
    lateral_axis: Sequence[float],
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
    }


def local_segment_diagnostics(
    task,
    indices: Sequence[int],
    entry_center: Sequence[float],
    insertion_axis: Sequence[float],
    lateral_axis: Sequence[float],
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
    )


def local_geometry_passes(
    diagnostics: Mapping[str, np.ndarray], preparation: Mapping[str, Any]
) -> bool:
    return bool(
        float(diagnostics["max_signed_entry_distance_m"])
        <= -float(preparation["entry_clearance_margin_m"])
        and float(diagnostics["median_alignment_cosine"])
        >= float(preparation["alignment_cosine_min"])
        and float(diagnostics["max_local_speed_mps"])
        <= float(preparation["max_local_speed_mps"])
    )
