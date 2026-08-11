"""Native-contact measurements and fixed R1 construction rules."""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


def point_tangent_velocity(velocities: Sequence[Sequence[float]], element_id: int, tangent: Sequence[float]) -> float:
    velocity = np.asarray(velocities, dtype=float)[int(element_id), :3]
    return float(np.dot(velocity, np.asarray(tangent, dtype=float)))


def line_contact_velocity(
    positions: Sequence[Sequence[float]],
    velocities: Sequence[Sequence[float]],
    first: int,
    second: int,
    contact_point: Sequence[float],
) -> np.ndarray:
    points = np.asarray(positions, dtype=float)
    velocity = np.asarray(velocities, dtype=float)
    start, end, point = points[first, :3], points[second, :3], np.asarray(contact_point, dtype=float)[:3]
    segment = end - start
    alpha = float(np.clip(np.dot(point - start, segment) / max(float(np.dot(segment, segment)), 1e-12), 0.0, 1.0))
    return (1.0 - alpha) * velocity[first, :3] + alpha * velocity[second, :3]


def contact_episode_stats(mask: Sequence[bool], dt_s: float) -> dict[str, float | int]:
    values = np.asarray(mask, dtype=bool)
    lengths: list[int] = []
    start = 0
    while start < values.size:
        if not values[start]:
            start += 1
            continue
        end = start
        while end < values.size and values[end]:
            end += 1
        lengths.append(end - start)
        start = end
    durations = np.asarray(lengths, dtype=float) * float(dt_s)
    return {
        "occupancy": float(values.mean()) if values.size else 0.0,
        "episodes": len(lengths),
        "longest_duration_s": float(durations.max()) if durations.size else 0.0,
        "median_duration_s": float(np.median(durations)) if durations.size else 0.0,
    }


def reaction_proxy_clean(native_contact_counts: Sequence[int], constraint_sizes: Sequence[int], reactions: Sequence[float]) -> bool:
    return bool(
        np.all(np.asarray(native_contact_counts, dtype=int) == 0)
        and np.all(np.asarray(constraint_sizes, dtype=int) == 0)
        and np.allclose(np.asarray(reactions, dtype=float), 0.0)
    )


def find_breakaway(
    forces: Sequence[float],
    native_tangent_speeds: Sequence[float],
    native_contact_counts: Sequence[int],
    measurement_active: Sequence[bool],
    *,
    dt_s: float,
    window_ms: int,
    radius_m: float,
) -> dict[str, float] | None:
    samples = int(round((window_ms / 1000.0) / dt_s))
    speeds = np.asarray(native_tangent_speeds, dtype=float)
    contacts = np.asarray(native_contact_counts, dtype=int) > 0
    active = np.asarray(measurement_active, dtype=bool)
    if samples <= 0 or speeds.size < samples:
        return None
    for end in range(samples - 1, speeds.size):
        current = slice(end - samples + 1, end + 1)
        if not active[current].all() or contacts[current].mean() < 0.90:
            continue
        travel = float(np.sum(np.abs(speeds[current])) * dt_s)
        if travel >= 0.25 * radius_m:
            return {"force_n": float(forces[end]), "travel_m": travel, "window_end_index": int(end)}
    return None


def derived_excitation(breakaway_force_n: float) -> tuple[float, float]:
    return (0.50 * float(breakaway_force_n), 1.25 * float(breakaway_force_n))


def progress_windows(progress: Sequence[float], dt_s: float, window_ms: int) -> np.ndarray:
    samples = int(round((window_ms / 1000.0) / dt_s))
    values = np.asarray(progress, dtype=float)
    if values.size <= samples:
        raise ValueError("trace is shorter than progress window")
    return values[samples:] - values[:-samples]


def percentile(values: Iterable[float], q: float) -> float:
    array = np.asarray(list(values), dtype=float)
    if not array.size or not np.isfinite(array).all():
        raise ValueError("metric has no finite samples")
    return float(np.percentile(array, q))
