"""Numerical SOFA Oracle calibration rules shared by the runner and tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class CalibrationResult:
    passed: bool
    values: dict[str, float]
    failures: tuple[str, ...]


def _percentile(values: Sequence[float], q: float, label: str) -> float:
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0 or not np.isfinite(array).all():
        raise ValueError(f"{label} has no finite samples")
    return float(np.percentile(array, q))


def progress_windows(progress: Sequence[float], dt_s: float, window_ms: int) -> np.ndarray:
    samples = int(round((window_ms / 1000.0) / dt_s))
    values = np.asarray(progress, dtype=np.float64)
    if samples <= 0 or values.size <= samples:
        raise ValueError("progress trace is shorter than the jam window")
    return values[samples:] - values[:-samples]


def calibrate(
    traces: Mapping[str, Mapping[str, Sequence[float]]],
    *,
    dt_s: float,
    window_ms: int,
) -> CalibrationResult:
    free_force_p99 = _percentile(traces["free_calibration"]["reaction_force_proxy"], 99, "free reaction")
    stick_force_p10 = _percentile(traces["stick_calibration"]["contact_reaction_force_proxy"], 10, "stick reaction")
    slip_force_p10 = _percentile(traces["slip_calibration"]["contact_reaction_force_proxy"], 10, "slip reaction")
    contact_force_p10 = min(stick_force_p10, slip_force_p10)

    stick_speed_p95 = _percentile(traces["stick_calibration"]["contact_tangent_speed_mps"], 95, "stick speed")
    slip_speed_p05 = _percentile(traces["slip_calibration"]["contact_tangent_speed_mps"], 5, "slip speed")

    free_progress_p10 = _percentile(
        progress_windows(traces["free_forward_calibration"]["progress_m"], dt_s, window_ms),
        10,
        "free progress",
    )
    jam_progress_p90 = _percentile(
        progress_windows(traces["jam_calibration"]["progress_m"], dt_s, window_ms),
        90,
        "jam progress",
    )

    failures: list[str] = []
    if not free_force_p99 < contact_force_p10:
        failures.append("contact_reaction_overlap")
    if not stick_speed_p95 < slip_speed_p05:
        failures.append("stick_slip_speed_overlap")
    if not jam_progress_p90 < free_progress_p10:
        failures.append("jam_free_progress_overlap")

    values = {
        "free_force_p99": free_force_p99,
        "stick_force_p10": stick_force_p10,
        "slip_force_p10": slip_force_p10,
        "contact_force_p10": contact_force_p10,
        "stick_speed_p95": stick_speed_p95,
        "slip_speed_p05": slip_speed_p05,
        "free_progress_p10": free_progress_p10,
        "jam_progress_p90": jam_progress_p90,
    }
    if not failures:
        force_gap = contact_force_p10 - free_force_p99
        speed_gap = slip_speed_p05 - stick_speed_p95
        values.update(
            {
                "reaction_force_floor": 0.5 * (free_force_p99 + contact_force_p10),
                "stick_tangent_speed_max_mps": stick_speed_p95 + 0.25 * speed_gap,
                "slip_tangent_speed_min_mps": slip_speed_p05 - 0.25 * speed_gap,
                "jam_progress_max_m": 0.5 * (jam_progress_p90 + free_progress_p10),
                "reaction_force_separation_gap": force_gap,
            }
        )
    return CalibrationResult(not failures, values, tuple(failures))


def sustained_true(values: Sequence[bool], dt_s: float, minimum_ms: int) -> np.ndarray:
    required = int(round((minimum_ms / 1000.0) / dt_s))
    result = np.zeros(len(values), dtype=bool)
    start = 0
    values = np.asarray(values, dtype=bool)
    while start < len(values):
        if not values[start]:
            start += 1
            continue
        end = start
        while end < len(values) and values[end]:
            end += 1
        if end - start >= required:
            result[start:end] = True
        start = end
    return result
