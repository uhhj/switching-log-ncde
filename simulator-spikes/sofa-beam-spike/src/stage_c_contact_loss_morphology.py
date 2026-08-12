"""Offline morphology audit for the frozen Phase-0S Stage-C V2.1 trace.

Formal contact semantics remain in stage_c_contact_semantics.py. The node
geometry here is diagnostic-only and must not be used as an Oracle, a contact
label, a Stage-C pass gate, or a training target.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

VERDICT_CROSS_THROUGH = "PHASE0S_SOFA_STAGE_C_LOSS_CROSS_THROUGH"
VERDICT_REBOUND_ESCAPE = "PHASE0S_SOFA_STAGE_C_LOSS_REBOUND_ESCAPE"
VERDICT_FIXTURE_EDGE_ESCAPE = "PHASE0S_SOFA_STAGE_C_LOSS_FIXTURE_EDGE_ESCAPE"
VERDICT_CONTACT_MIGRATION = "PHASE0S_SOFA_STAGE_C_LOSS_CONTACT_MIGRATION"
VERDICT_REACTION_COLLAPSE = "PHASE0S_SOFA_STAGE_C_LOSS_REACTION_COLLAPSE"
VERDICT_AMBIGUOUS = "PHASE0S_SOFA_STAGE_C_LOSS_MORPHOLOGY_AMBIGUOUS"


def trailing_true_run(mask: Sequence[bool]) -> int:
    """Length of the final contiguous True episode."""
    count = 0
    for value in reversed(mask):
        if not bool(value):
            break
        count += 1
    return int(count)


def _positions(record: dict) -> np.ndarray:
    positions = np.asarray(record["beam_node_positions_m"], dtype=float)
    if positions.ndim != 2 or positions.shape[1] < 3:
        raise ValueError("beam_node_positions_m must have shape [N, >=3]")
    positions = positions[:, :3]
    if not np.all(np.isfinite(positions)):
        raise ValueError("beam_node_positions_m contains non-finite values")
    return positions


def _inside_footprint(point: np.ndarray, wall: dict, tol: float) -> bool:
    return bool(
        float(wall["x_min_m"]) - tol <= point[0] <= float(wall["x_max_m"]) + tol
        and -float(wall["z_half_extent_m"]) - tol <= point[2] <= float(wall["z_half_extent_m"]) + tol
    )


def frame_wall_geometry(record: dict, *, wall: dict, effective_contact_distance_m: float, plane_tol_m: float) -> dict:
    positions = _positions(record)
    ids = [i for i, point in enumerate(positions) if _inside_footprint(point, wall, float(plane_tol_m))]
    if not ids:
        return {"footprint_node_ids": [], "footprint_node_count": 0, "minimum_abs_plane_distance_m": None, "surrogate_gap_m": None, "within_effective_envelope": False, "all_positive_outside_envelope": False, "any_negative_outside_envelope": False, "all_negative_outside_envelope": False, "minimum_signed_plane_distance_m": None, "maximum_signed_plane_distance_m": None}
    signed = positions[ids, 1] - float(wall["plane_y_m"])
    minimum_abs = float(np.min(np.abs(signed)))
    effective, tol = float(effective_contact_distance_m), float(plane_tol_m)
    surrogate_gap = minimum_abs - effective
    return {"footprint_node_ids": [int(i) for i in ids], "footprint_node_count": len(ids), "minimum_abs_plane_distance_m": minimum_abs, "surrogate_gap_m": float(surrogate_gap), "within_effective_envelope": bool(surrogate_gap <= tol), "all_positive_outside_envelope": bool(np.all(signed > effective + tol)), "any_negative_outside_envelope": bool(np.any(signed < -effective - tol)), "all_negative_outside_envelope": bool(np.all(signed < -effective - tol)), "minimum_signed_plane_distance_m": float(np.min(signed)), "maximum_signed_plane_distance_m": float(np.max(signed))}


def positive_to_negative_crossings(records: Sequence[dict], *, wall: dict, plane_tol_m: float, start_index: int) -> list[dict]:
    """Detect gradual + -> deadband -> - crossings inside one footprint episode."""
    if start_index < 0 or start_index >= len(records):
        raise ValueError("start_index out of range")
    positions = [_positions(record) for record in records]
    node_count = positions[0].shape[0]
    if any(value.shape != positions[0].shape for value in positions):
        raise ValueError("beam node count changed inside frozen trace")
    tol, plane = float(plane_tol_m), float(wall["plane_y_m"])
    positive_anchor = [None] * node_count
    events: list[dict] = []
    for i in range(start_index, len(records)):
        for node_id in range(node_count):
            point = positions[i][node_id]
            if not _inside_footprint(point, wall, tol):
                positive_anchor[node_id] = None
                continue
            signed = float(point[1] - plane)
            if signed > tol:
                positive_anchor[node_id] = (i, signed)
            elif signed < -tol and positive_anchor[node_id] is not None:
                from_index, from_signed = positive_anchor[node_id]
                events.append({"node_id": int(node_id), "from_record_index": int(from_index), "to_record_index": int(i), "from_time_s": float(records[from_index]["step_end_time_s"]), "to_time_s": float(records[i]["step_end_time_s"]), "transition_duration_s": float(records[i]["step_end_time_s"]) - float(records[from_index]["step_end_time_s"]), "from_signed_plane_distance_m": float(from_signed), "to_signed_plane_distance_m": signed})
                positive_anchor[node_id] = None
    return events


def first_true_index(classifications: Sequence[dict], key: str):
    return next((i for i, c in enumerate(classifications) if bool(c[key])), None)


def last_true_index(classifications: Sequence[dict], key: str):
    return next((i for i in range(len(classifications) - 1, -1, -1) if bool(classifications[i][key])), None)


def _time(records: Sequence[dict], index):
    return None if index is None else float(records[int(index)]["step_end_time_s"])


def observed_timestamp_span_s(records: Sequence[dict], start_index, end_index) -> float:
    """Conservative observed dwell: last observation time minus first."""
    if start_index is None or end_index is None:
        return 0.0
    start, end = int(start_index), int(end_index)
    if start < 0 or end < start or end >= len(records):
        raise ValueError("invalid observed timestamp-span indices")
    return float(records[end]["step_end_time_s"]) - float(records[start]["step_end_time_s"])


def classify_contact_loss_morphology(records: Sequence[dict], classifications: Sequence[dict], *, config: dict, effective_contact_distance_m: float) -> dict:
    if len(records) != len(classifications):
        raise ValueError("record/classification length mismatch")
    if not records:
        raise ValueError("empty Stage-C trace")
    wall = config["contact_wall"]
    plane_tol = float(config["gates"]["maximum_fixture_plane_error_m"])
    weak_minimum_dwell_s = float(config["gates"]["minimum_observed_contact_dwell_ms"]) / 1000.0
    geometries = [frame_wall_geometry(r, wall=wall, effective_contact_distance_m=effective_contact_distance_m, plane_tol_m=plane_tol) for r in records]
    geometric_first, geometric_last = first_true_index(classifications, "geometric_contact"), last_true_index(classifications, "geometric_contact")
    load_first, load_last = first_true_index(classifications, "load_bearing_contact"), last_true_index(classifications, "load_bearing_contact")
    onset_index = geometric_first if geometric_first is not None else load_first
    onsets = {"diagnostic_onset_index": onset_index, "diagnostic_onset_source": "geometric_contact" if geometric_first is not None else ("load_bearing_contact_fallback" if load_first is not None else None), "geometric_first_index": geometric_first, "geometric_first_time_s": _time(records, geometric_first), "geometric_last_index": geometric_last, "geometric_last_time_s": _time(records, geometric_last), "load_bearing_first_index": load_first, "load_bearing_first_time_s": _time(records, load_first), "load_bearing_last_index": load_last, "load_bearing_last_time_s": _time(records, load_last)}
    if onset_index is None:
        return {"verdict": VERDICT_AMBIGUOUS, "selected_mechanism": "ambiguous", "reason": "No formal geometric/load-bearing onset exists.", "formal_onsets": onsets, "candidate_flags": {}, "crossing_events": [], "crossing_event_count": 0, "offline_geometry_surrogate_is_formal_contact_signal": False}
    crossing_scan_start = max(0, onset_index - 1)
    crossings = positive_to_negative_crossings(records, wall=wall, plane_tol_m=plane_tol, start_index=crossing_scan_start)
    terminal_cls, terminal_geo = classifications[-1], geometries[-1]
    migration_mask = [(not bool(c["native_detection"])) and bool(g["within_effective_envelope"]) for c, g in zip(classifications[onset_index:], geometries[onset_index:])]
    migration_run = trailing_true_run(migration_mask)
    migration_anchor, migration_end = (len(records) - migration_run, len(records) - 1) if migration_run else (None, None)
    migration_duration_s = observed_timestamp_span_s(records, migration_anchor, migration_end)
    reaction_mask = [bool(c["geometric_contact"]) and not bool(c["reaction_active"]) for c in classifications[: geometric_last + 1]] if geometric_last is not None else []
    reaction_run = trailing_true_run(reaction_mask)
    reaction_anchor = geometric_last - reaction_run + 1 if reaction_run else None
    reaction_end = geometric_last if reaction_run else None
    reaction_duration_s = observed_timestamp_span_s(records, reaction_anchor, reaction_end)
    post_geo = geometries[onset_index:]
    ever_footprint = any(g["footprint_node_count"] > 0 for g in post_geo)
    terminal_footprint, terminal_native_absent = terminal_geo["footprint_node_count"] > 0, not bool(terminal_cls["native_detection"])
    cross = bool(crossings and terminal_footprint and terminal_geo["any_negative_outside_envelope"] and terminal_native_absent)
    edge = bool(not cross and ever_footprint and not terminal_footprint and terminal_native_absent)
    rebound = bool(not cross and not edge and not crossings and terminal_footprint and terminal_geo["all_positive_outside_envelope"] and terminal_native_absent)
    migration = bool(not cross and not edge and not rebound and migration_duration_s + 1e-12 >= weak_minimum_dwell_s)
    reaction = bool(not cross and not edge and not rebound and reaction_duration_s + 1e-12 >= weak_minimum_dwell_s)
    if cross: verdict, mechanism = VERDICT_CROSS_THROUGH, "cross_through"
    elif edge: verdict, mechanism = VERDICT_FIXTURE_EDGE_ESCAPE, "fixture_edge_escape"
    elif rebound: verdict, mechanism = VERDICT_REBOUND_ESCAPE, "rebound_escape"
    elif int(migration) + int(reaction) == 1: verdict, mechanism = (VERDICT_CONTACT_MIGRATION, "contact_migration") if migration else (VERDICT_REACTION_COLLAPSE, "reaction_collapse")
    else: verdict, mechanism = VERDICT_AMBIGUOUS, "ambiguous"
    gaps = np.asarray([g["surrogate_gap_m"] for g in geometries if g["surrogate_gap_m"] is not None], dtype=float)
    mins = np.asarray([g["minimum_signed_plane_distance_m"] for g in geometries if g["minimum_signed_plane_distance_m"] is not None], dtype=float)
    maxs = np.asarray([g["maximum_signed_plane_distance_m"] for g in geometries if g["maximum_signed_plane_distance_m"] is not None], dtype=float)
    counts = np.asarray([g["footprint_node_count"] for g in geometries], dtype=int)
    return {"verdict": verdict, "selected_mechanism": mechanism, "formal_onsets": onsets, "terminal": {"time_s": float(records[-1]["step_end_time_s"]), "native_detection": bool(terminal_cls["native_detection"]), **terminal_geo}, "crossing_scan_start_index": crossing_scan_start, "crossing_scan_start_time_s": float(records[crossing_scan_start]["step_end_time_s"]), "crossing_events": crossings, "crossing_event_count": len(crossings), "weak_mechanism_anchors": {"migration_terminal_suffix_start_index": migration_anchor, "migration_terminal_suffix_start_time_s": _time(records, migration_anchor), "migration_terminal_suffix_end_index": migration_end, "migration_terminal_suffix_end_time_s": _time(records, migration_end), "reaction_final_geometric_suffix_start_index": reaction_anchor, "reaction_final_geometric_suffix_start_time_s": _time(records, reaction_anchor), "reaction_final_geometric_suffix_end_index": reaction_end, "reaction_final_geometric_suffix_end_time_s": _time(records, reaction_end)}, "final_loss_no_native_near_wall_terminal_run_steps": migration_run, "final_loss_no_native_near_wall_terminal_duration_s": float(migration_duration_s), "final_geometric_without_reaction_terminal_run_steps": reaction_run, "final_geometric_without_reaction_terminal_duration_s": float(reaction_duration_s), "weak_mechanism_duration_semantics": "timestamp_span_end_minus_start", "weak_minimum_dwell_s": float(weak_minimum_dwell_s), "frozen_max_native_lcp_onset_lag_steps_diagnostic_only": int(config["gates"]["maximum_native_lcp_onset_lag_steps"]), "candidate_flags": {"cross_through": cross, "rebound_escape": rebound, "fixture_edge_escape": edge, "contact_migration": migration, "reaction_collapse": reaction}, "trajectory_summary": {"frames": len(records), "footprint_node_count_min": int(counts.min()), "footprint_node_count_max": int(counts.max()), "surrogate_gap_min_m": float(gaps.min()) if gaps.size else None, "surrogate_gap_max_m": float(gaps.max()) if gaps.size else None, "minimum_signed_plane_distance_min_m": float(mins.min()) if mins.size else None, "maximum_signed_plane_distance_max_m": float(maxs.max()) if maxs.size else None}, "offline_geometry_surrogate_is_formal_contact_signal": False}
