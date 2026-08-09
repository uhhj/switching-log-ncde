from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

import numpy as np


def sustained_mask(mask: Sequence[bool], min_samples: int) -> np.ndarray:
    """Keep complete true runs whose length reaches ``min_samples``."""
    values = np.asarray(mask, dtype=bool)
    minimum = int(min_samples)
    if minimum <= 0:
        raise ValueError("min_samples must be positive")
    output = np.zeros(values.shape, dtype=bool)
    start = 0
    while start < values.size:
        if not values[start]:
            start += 1
            continue
        end = start + 1
        while end < values.size and values[end]:
            end += 1
        if end - start >= minimum:
            output[start:end] = True
        start = end
    return output


def longest_true_run_samples(mask: Sequence[bool]) -> int:
    values = np.asarray(mask, dtype=bool)
    longest = 0
    current = 0
    for value in values:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return int(longest)


def compress_transitions(modes: Sequence[str]) -> str:
    compressed = []
    for mode in np.asarray(modes).astype(str):
        if not compressed or compressed[-1] != mode:
            compressed.append(mode)
    return " -> ".join(compressed)


def ordered_modes_present(modes: Sequence[str], required: Sequence[str]) -> bool:
    """Return whether required modes occur in order, allowing intermediates."""
    position = 0
    for mode in np.asarray(modes).astype(str):
        if position < len(required) and mode == required[position]:
            position += 1
    return position == len(required)


def derive_labels(
    trace: Mapping[str, np.ndarray], config: Mapping[str, Any]
) -> Dict[str, np.ndarray]:
    labels = config["labels"]
    simulator = config["simulator"]
    effective_trace_hz = float(simulator["hz"]) / float(
        simulator["trace_stride"]
    )
    minimum_samples = max(
        1,
        int(
            np.ceil(
                float(labels["minimum_mode_duration_ms"])
                * effective_trace_hz
                / 1000.0
            )
        ),
    )
    jam_window_samples = max(
        1,
        int(
            round(
                float(labels["jam_window_ms"])
                * effective_trace_hz
                / 1000.0
            )
        ),
    )

    active = np.asarray(trace["contact_active_beads"], dtype=np.int64)
    normal_force = np.asarray(trace["normal_force_sum"], dtype=np.float64)
    tangent_speed = np.asarray(
        trace["tangential_speed_mean"], dtype=np.float64
    )
    command = np.asarray(trace["command_velocity_xyz"], dtype=np.float64)
    command_speed = np.linalg.norm(command, axis=1)
    progress = np.asarray(trace["progress_m"], dtype=np.float64)
    phase = np.asarray(trace["phase"]).astype(str)

    contact = (active > 0) & (
        normal_force >= float(labels["contact_normal_force_min_n"])
    )
    slip = contact & (
        tangent_speed >= float(labels["slip_tangent_speed_min_mps"])
    )
    stick = (
        contact
        & (tangent_speed <= float(labels["stick_tangent_speed_max_mps"]))
        & (command_speed >= float(labels["command_active_speed_min_mps"]))
    )

    progress_delta = np.zeros(progress.shape, dtype=np.float64)
    if progress.size > jam_window_samples:
        progress_delta[jam_window_samples:] = (
            progress[jam_window_samples:] - progress[:-jam_window_samples]
        )
    jam_candidate = np.zeros(progress.shape, dtype=bool)
    eligible = np.arange(progress.size) >= jam_window_samples
    jam_candidate = (
        eligible
        & (phase == "insertion")
        & contact
        & (normal_force >= float(labels["jam_normal_force_min_n"]))
        & (command_speed >= float(labels["command_active_speed_min_mps"]))
        & (progress_delta <= float(labels["jam_progress_max_m"]))
    )
    jam = sustained_mask(jam_candidate, minimum_samples)

    modes = np.full(progress.shape, "contact_transition", dtype="U24")
    modes[stick] = "stick"
    modes[slip] = "slip"
    modes[~contact] = "free"
    modes[jam] = "jam"

    touch = np.zeros(contact.shape, dtype=bool)
    release = np.zeros(contact.shape, dtype=bool)
    if contact.size:
        touch[0] = bool(contact[0])
        touch[1:] = contact[1:] & ~contact[:-1]
        release[1:] = ~contact[1:] & contact[:-1]

    return {
        "contact": contact,
        "free": ~contact,
        "slip": slip,
        "stick": stick,
        "jam_candidate": jam_candidate,
        "jam": jam,
        "touch": touch,
        "release": release,
        "mode": modes,
        "command_speed": command_speed,
        "progress_delta": progress_delta,
        "minimum_mode_samples": np.asarray(minimum_samples),
        "jam_window_samples": np.asarray(jam_window_samples),
    }
