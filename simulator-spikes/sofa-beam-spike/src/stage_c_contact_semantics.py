"""Frozen Stage-C V2.1 LocalMinDistance contact semantics.

Interpret native DetectionOutput point pairs only for the frozen SOFA
v26.06.00 PointCollisionModel <-> TriangleCollisionModel scene using
LocalMinDistance. This module does not change NativeContactBridge semantics.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def local_min_distance_gap_m(
    beam_contact_point_m: Sequence[float],
    fixture_contact_point_m: Sequence[float],
    effective_contact_distance_m: float,
) -> float:
    """Reconstruct DetectionOutput::value for the frozen Point/Triangle LMD path."""
    beam = np.asarray(beam_contact_point_m, dtype=float).reshape(3)
    fixture = np.asarray(fixture_contact_point_m, dtype=float).reshape(3)
    return float(np.linalg.norm(beam - fixture)) - float(effective_contact_distance_m)


def contact_gaps_m(record: dict, effective_contact_distance_m: float) -> list[float]:
    return [
        local_min_distance_gap_m(
            contact["beam_contact_point_m"],
            contact["fixture_contact_point_m"],
            effective_contact_distance_m,
        )
        for contact in record["native_contacts"]
    ]


def classify_record(
    record: dict,
    *,
    effective_contact_distance_m: float,
    reaction_floor: float,
) -> dict:
    gaps = contact_gaps_m(record, effective_contact_distance_m)
    native_count = int(record["native_contact_count"])
    if native_count != len(record["native_contacts"]):
        raise ValueError("native_contact_count/native_contacts mismatch")

    geometric_count = sum(gap <= 0.0 for gap in gaps)
    native_detection = native_count > 0
    geometric_contact = geometric_count > 0
    lcp_rows_present = int(record["constraint_vector_size"]) > 0
    reaction_active = float(record["reaction_force_proxy"]) > float(reaction_floor)

    return {
        "native_detection": bool(native_detection),
        "native_detection_count": native_count,
        "local_min_distance_gaps_m": [float(value) for value in gaps],
        "geometric_contact": bool(geometric_contact),
        "geometric_contact_count": int(geometric_count),
        "proximity_only": bool(native_detection and not geometric_contact),
        "lcp_rows_present": bool(lcp_rows_present),
        "reaction_active": bool(reaction_active),
        "load_bearing_contact": bool(geometric_contact and reaction_active),
    }


def first_active_time_s(records: Sequence[dict], classifications: Sequence[dict], key: str):
    if len(records) != len(classifications):
        raise ValueError("record/classification length mismatch")
    for record, classification in zip(records, classifications):
        if bool(classification[key]):
            return float(record["step_end_time_s"])
    return None


def selected_geometric_contacts(
    records: Sequence[dict],
    classifications: Sequence[dict],
    effective_contact_distance_m: float,
) -> list[dict]:
    if len(records) != len(classifications):
        raise ValueError("record/classification length mismatch")

    selected: list[dict] = []
    for record, classification in zip(records, classifications):
        if not classification["geometric_contact"]:
            continue
        gaps = contact_gaps_m(record, effective_contact_distance_m)
        for contact, gap in zip(record["native_contacts"], gaps):
            if gap <= 0.0:
                selected.append({**contact, "local_min_distance_gap_m": float(gap)})
    return selected
