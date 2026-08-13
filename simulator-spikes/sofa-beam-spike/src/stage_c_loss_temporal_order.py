"""Temporal ordering of Stage-C crossing and permanent fixture exit.

Formal contact semantics remain exclusively in stage_c_contact_semantics.py.
Node geometry in this module is diagnostic only. Temporal precedence is not
causal proof.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from stage_c_contact_loss_morphology import positive_to_negative_crossings

VERDICT_CROSSING_FIRST = "PHASE0S_SOFA_STAGE_C_V2_2L_CROSSING_PRECEDES_FIRST_PERMANENT_NODE_EXIT"
VERDICT_EXIT_FIRST = "PHASE0S_SOFA_STAGE_C_V2_2L_FIRST_PERMANENT_NODE_EXIT_PRECEDES_CROSSING"
VERDICT_AMBIGUOUS = "PHASE0S_SOFA_STAGE_C_V2_2L_TEMPORAL_ORDER_AMBIGUOUS"


def _positions(record: dict) -> np.ndarray:
    value = np.asarray(record["beam_node_positions_m"], dtype=float)
    if value.ndim != 2 or value.shape[1] < 3:
        raise ValueError("beam_node_positions_m must have shape [N, >=3]")
    value = value[:, :3]
    if not np.isfinite(value).all():
        raise ValueError("beam_node_positions_m contains non-finite values")
    return value


def translation_fixed_node_ids(config: dict) -> set[int]:
    tail = config["tail_constraint"]
    directions = [int(value) for value in tail["fixed_directions"]]
    if len(directions) < 3:
        raise ValueError("tail fixed_directions must expose xyz translation flags")
    return {int(tail["node_index"])} if directions[:3] == [1, 1, 1] else set()


def mobile_node_ids(config: dict) -> list[int]:
    all_ids = set(range(int(config["beam"]["nodes"])))
    fixed = translation_fixed_node_ids(config)
    if not fixed.issubset(all_ids):
        raise ValueError("translation-fixed node outside beam node range")
    result = sorted(all_ids - fixed)
    if not result:
        raise ValueError("no mobile beam nodes")
    return result


def _inside_fixture_footprint(point: Sequence[float], *, wall: dict, tolerance_m: float) -> bool:
    x, _, z = [float(value) for value in point[:3]]
    tol = float(tolerance_m)
    return bool(float(wall["x_min_m"]) - tol <= x <= float(wall["x_max_m"]) + tol and -float(wall["z_half_extent_m"]) - tol <= z <= float(wall["z_half_extent_m"]) + tol)


def mobile_footprint_node_ids(record: dict, *, config: dict) -> list[int]:
    positions = _positions(record)
    if positions.shape[0] != int(config["beam"]["nodes"]):
        raise ValueError("beam node count mismatch")
    return [node for node in mobile_node_ids(config) if _inside_fixture_footprint(positions[node], wall=config["contact_wall"], tolerance_m=float(config["gates"]["maximum_fixture_plane_error_m"]))]


def mobile_native_detection_ids(record: dict, *, config: dict) -> list[int]:
    mobile = set(mobile_node_ids(config))
    return sorted({int(contact["beam_element_id"]) for contact in record["native_contacts"] if int(contact["beam_element_id"]) in mobile})


def strong_mobile_edge_mask(records: Sequence[dict], *, config: dict) -> list[bool]:
    """All-mobile terminal-edge condition, retained as diagnostic only."""
    return [not mobile_footprint_node_ids(record, config=config) and not mobile_native_detection_ids(record, config=config) for record in records]


def first_permanent_true_suffix_index(mask: Sequence[bool], *, start_index: int):
    if start_index < 0 or start_index >= len(mask):
        raise ValueError("suffix start index out of range")
    all_true, earliest = True, None
    for index in range(len(mask) - 1, start_index - 1, -1):
        all_true = bool(mask[index]) and all_true
        if all_true:
            earliest = index
    return earliest


def first_geometric_index(classifications: Sequence[dict]):
    return next((index for index, item in enumerate(classifications) if bool(item["geometric_contact"])), None)


def last_geometric_index(classifications: Sequence[dict]):
    return next((index for index in range(len(classifications) - 1, -1, -1) if bool(classifications[index]["geometric_contact"])), None)


def reconstruct_crossings(records: Sequence[dict], classifications: Sequence[dict], *, config: dict) -> list[dict]:
    onset = first_geometric_index(classifications)
    return [] if onset is None else positive_to_negative_crossings(records, wall=config["contact_wall"], plane_tol_m=float(config["gates"]["maximum_fixture_plane_error_m"]), start_index=max(0, onset - 1))


def crossing_consistency(reconstructed: Sequence[dict], frozen: Sequence[dict], *, time_tolerance_s: float) -> dict:
    tol, failures = float(time_tolerance_s), []
    if len(reconstructed) != len(frozen): failures.append("count_mismatch")
    for index in range(min(len(reconstructed), len(frozen))):
        current, expected = reconstructed[index], frozen[index]
        for key in ("node_id", "from_record_index", "to_record_index"):
            if int(current[key]) != int(expected[key]): failures.append(f"event_{index}_{key}")
        for key in ("from_time_s", "to_time_s"):
            if abs(float(current[key]) - float(expected[key])) > tol: failures.append(f"event_{index}_{key}")
    return {"pass": not failures, "failures": failures, "reconstructed_count": len(reconstructed), "frozen_count": len(frozen)}


def _boundary_violations(point: Sequence[float], *, wall: dict, tolerance_m: float) -> list[str]:
    x, _, z = [float(value) for value in point[:3]]; tol = float(tolerance_m); result = []
    if x < float(wall["x_min_m"]) - tol: result.append("x_min")
    if x > float(wall["x_max_m"]) + tol: result.append("x_max")
    if z < -float(wall["z_half_extent_m"]) - tol: result.append("z_min")
    if z > float(wall["z_half_extent_m"]) + tol: result.append("z_max")
    return result


def per_node_permanent_fixture_exits(records: Sequence[dict], *, config: dict, start_index: int) -> list[dict]:
    wall, tol, positions = config["contact_wall"], float(config["gates"]["maximum_fixture_plane_error_m"]), [_positions(record) for record in records]
    result = []
    for node in mobile_node_ids(config):
        inside = [index for index in range(start_index, len(records)) if _inside_fixture_footprint(positions[index][node], wall=wall, tolerance_m=tol)]
        if not inside:
            result.append({"node_id": node, "status": "never_inside_after_formal_onset", "last_inside_index": None, "permanent_exit_index": None, "permanent_exit_time_s": None, "boundary": []}); continue
        last = inside[-1]; exit_index = last + 1
        if exit_index >= len(records):
            result.append({"node_id": node, "status": "inside_at_terminal", "last_inside_index": last, "last_inside_time_s": float(records[last]["step_end_time_s"]), "permanent_exit_index": None, "permanent_exit_time_s": None, "boundary": []}); continue
        result.append({"node_id": node, "status": "permanently_outside_fixture_footprint", "last_inside_index": last, "last_inside_time_s": float(records[last]["step_end_time_s"]), "permanent_exit_index": exit_index, "permanent_exit_time_s": float(records[exit_index]["step_end_time_s"]), "boundary": _boundary_violations(positions[exit_index][node], wall=wall, tolerance_m=tol)})
    return result


def per_node_crossing_exit_table(crossings: Sequence[dict], exits: Sequence[dict]) -> list[dict]:
    by_node = {}
    for event in crossings:
        node = int(event["node_id"])
        if node not in by_node or float(event["to_time_s"]) < float(by_node[node]["to_time_s"]): by_node[node] = event
    rows = []
    for exit_record in exits:
        crossing = by_node.get(int(exit_record["node_id"])); crossing_time = float(crossing["to_time_s"]) if crossing else None; exit_time = float(exit_record["permanent_exit_time_s"]) if exit_record["permanent_exit_time_s"] is not None else None
        rows.append({"node_id": int(exit_record["node_id"]), "crossing_time_s": crossing_time, "permanent_exit_time_s": exit_time, "exit_minus_crossing_s": exit_time - crossing_time if exit_time is not None and crossing_time is not None else None, "exit_boundary": list(exit_record["boundary"]), "exit_status": exit_record["status"]})
    return rows


def audit_temporal_order(records: Sequence[dict], classifications: Sequence[dict], *, config: dict, frozen_crossing_events: Sequence[dict]) -> dict:
    if len(records) != len(classifications): raise ValueError("record/classification length mismatch")
    if not records: raise ValueError("empty Stage-C trace")
    first, last = first_geometric_index(classifications), last_geometric_index(classifications)
    if first is None: return {"verdict": VERDICT_AMBIGUOUS, "reason": "no formal geometric-contact onset", "crossing_consistency": {"pass": False, "failures": ["no_formal_geometric_onset"]}, "offline_geometry_is_formal_contact_signal": False, "temporal_precedence_is_causal_proof": False}
    crossings = reconstruct_crossings(records, classifications, config=config); consistency = crossing_consistency(crossings, frozen_crossing_events, time_tolerance_s=float(config["integrity"]["time_tolerance_s"])); first_crossing = min(crossings, key=lambda event: float(event["to_time_s"]), default=None); crossing_time = float(first_crossing["to_time_s"]) if first_crossing else None
    exits = per_node_permanent_fixture_exits(records, config=config, start_index=first); first_exit = min((item for item in exits if item["permanent_exit_time_s"] is not None), key=lambda item: float(item["permanent_exit_time_s"]), default=None); exit_time = float(first_exit["permanent_exit_time_s"]) if first_exit else None; tol = float(config["integrity"]["time_tolerance_s"])
    if not consistency["pass"]: verdict, reason = VERDICT_AMBIGUOUS, "new crossing reconstruction disagrees with frozen V2.2 crossing evidence"
    elif crossing_time is None or exit_time is None: verdict, reason = VERDICT_AMBIGUOUS, "crossing or earliest permanent mobile-node fixture exit cannot be established"
    elif crossing_time < exit_time - tol: verdict, reason = VERDICT_CROSSING_FIRST, "the first completed in-footprint wall-plane crossing precedes the earliest permanent mobile-node fixture exit"
    elif exit_time < crossing_time - tol: verdict, reason = VERDICT_EXIT_FIRST, "the earliest permanent mobile-node fixture exit precedes the first completed in-footprint wall-plane crossing"
    else: verdict, reason = VERDICT_AMBIGUOUS, "first crossing and earliest permanent mobile-node fixture exit are simultaneous within the frozen time tolerance"
    mask = strong_mobile_edge_mask(records, config=config); final_index = first_permanent_true_suffix_index(mask, start_index=first)
    return {"verdict": verdict, "reason": reason, "formal_comparison": "first_completed_in_footprint_crossing_vs_earliest_permanent_mobile_node_fixture_exit", "geometric_first_index": first, "geometric_first_time_s": float(records[first]["step_end_time_s"]), "geometric_last_index": last, "geometric_last_time_s": float(records[last]["step_end_time_s"]) if last is not None else None, "crossing_consistency": consistency, "crossing_events": crossings, "crossing_event_count": len(crossings), "first_crossing": first_crossing, "first_crossing_time_s": crossing_time, "first_permanent_node_exit": first_exit, "first_permanent_node_exit_time_s": exit_time, "first_exit_minus_first_crossing_s": exit_time - crossing_time if exit_time is not None and crossing_time is not None else None, "per_node_crossing_exit_table": per_node_crossing_exit_table(crossings, exits), "per_node_permanent_fixture_exits": exits, "final_all_mobile_permanent_edge_loss_onset_index_diagnostic": final_index, "final_all_mobile_permanent_edge_loss_onset_time_s_diagnostic": float(records[final_index]["step_end_time_s"]) if final_index is not None else None, "terminal_mobile_footprint_node_ids": mobile_footprint_node_ids(records[-1], config=config), "terminal_mobile_native_detection_ids": mobile_native_detection_ids(records[-1], config=config), "mobile_node_ids": mobile_node_ids(config), "excluded_translation_fixed_node_ids": sorted(translation_fixed_node_ids(config)), "time_tolerance_s": tol, "offline_geometry_is_formal_contact_signal": False, "temporal_precedence_is_causal_proof": False}
