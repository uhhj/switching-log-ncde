from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

import numpy as np


EVENT_NAMES = (
    "touch",
    "release",
    "stick_to_slip",
    "slip_to_stick",
    "jam_onset",
    "jam_release",
)


def sustained_mask(mask: Sequence[bool], minimum_samples: int) -> np.ndarray:
    values = np.asarray(mask, dtype=bool)
    output = np.zeros(values.shape, dtype=bool)
    start = 0
    while start < values.size:
        if not values[start]:
            start += 1
            continue
        end = start + 1
        while end < values.size and values[end]:
            end += 1
        if end - start >= int(minimum_samples):
            output[start:end] = True
        start = end
    return output


def progress_over_window(progress_m: Sequence[float], window_samples: int) -> np.ndarray:
    progress = np.asarray(progress_m, dtype=np.float64)
    delta = np.zeros(progress.shape, dtype=np.float64)
    if progress.size > int(window_samples):
        delta[int(window_samples) :] = (
            progress[int(window_samples) :] - progress[: -int(window_samples)]
        )
    return delta


def derive_labels(
    trace: Mapping[str, np.ndarray], thresholds: Mapping[str, Any], hz: float
) -> Dict[str, np.ndarray]:
    """Frozen Phase 0C0 hierarchy adapted to MuJoCo trace field names."""
    minimum_samples = max(
        1,
        int(
            np.ceil(
                float(thresholds["minimum_mode_duration_ms"])
                * float(hz)
                / 1000.0
            )
        ),
    )
    jam_window_samples = max(
        1,
        int(
            round(
                float(thresholds["jam_window_ms"])
                * float(hz)
                / 1000.0
            )
        ),
    )
    count = np.asarray(trace["contact_count"], dtype=np.int64)
    normal = np.asarray(trace["normal_force"], dtype=np.float64)
    tangent_velocity = np.asarray(trace["tangent_velocity"], dtype=np.float64)
    command = np.asarray(trace["command"], dtype=np.float64)
    command_speed = np.linalg.norm(command, axis=1)
    progress = np.asarray(trace["progress"], dtype=np.float64)
    phase = np.asarray(trace["phase"]).astype(str)

    contact = (count > 0) & (
        normal >= float(thresholds["contact_normal_force_min_n"])
    )
    slip = contact & (
        tangent_velocity >= float(thresholds["slip_tangent_speed_min_mps"])
    )
    stick = (
        contact
        & (tangent_velocity <= float(thresholds["stick_tangent_speed_max_mps"]))
        & (command_speed >= float(thresholds["command_active_speed_min_mps"]))
    )
    progress_delta = progress_over_window(progress, jam_window_samples)
    eligible = np.arange(progress.size) >= jam_window_samples
    jam_candidate = (
        eligible
        & (phase == "insertion")
        & contact
        & (normal >= float(thresholds["jam_normal_force_min_n"]))
        & (command_speed >= float(thresholds["command_active_speed_min_mps"]))
        & (progress_delta <= float(thresholds["jam_progress_max_m"]))
    )
    jam = sustained_mask(jam_candidate, minimum_samples)

    contact_label = np.where(contact, "contact", "free")
    friction_label = np.full(progress.shape, "none", dtype="U8")
    friction_label[contact] = "transition"
    friction_label[stick] = "stick"
    friction_label[slip] = "slip"
    failure_label = np.where(jam, "jam", "normal")

    touch = np.zeros(contact.shape, dtype=bool)
    release = np.zeros(contact.shape, dtype=bool)
    stick_to_slip = np.zeros(contact.shape, dtype=bool)
    slip_to_stick = np.zeros(contact.shape, dtype=bool)
    jam_onset = np.zeros(contact.shape, dtype=bool)
    jam_release = np.zeros(contact.shape, dtype=bool)
    if contact.size:
        touch[0] = bool(contact[0])
        touch[1:] = contact[1:] & ~contact[:-1]
        release[1:] = ~contact[1:] & contact[:-1]
    if contact.size > 1:
        stick_to_slip[1:] = slip[1:] & stick[:-1]
        slip_to_stick[1:] = stick[1:] & slip[:-1]
        jam_onset[1:] = jam[1:] & ~jam[:-1]
        jam_release[1:] = ~jam[1:] & jam[:-1]
    events = np.column_stack(
        (touch, release, stick_to_slip, slip_to_stick, jam_onset, jam_release)
    )
    return {
        "contact": contact,
        "free": ~contact,
        "stick": stick,
        "slip": slip,
        "jam": jam,
        "contact_label": contact_label,
        "friction_label": friction_label,
        "failure_label": failure_label,
        "events": events,
        "event_names": np.asarray(EVENT_NAMES),
        "progress_delta": progress_delta,
        "minimum_samples": np.asarray(minimum_samples),
        "jam_window_samples": np.asarray(jam_window_samples),
    }
