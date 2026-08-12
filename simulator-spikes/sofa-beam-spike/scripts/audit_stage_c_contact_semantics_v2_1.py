"""Offline Stage-C V2.1 contact-semantics re-audit."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
sys.path.insert(0, str(SPIKE / "src"))

from contact_native import contact_episode_stats_timestamps  # noqa: E402
from stage_c_contact_semantics import (  # noqa: E402
    classify_record,
    first_active_time_s,
    selected_geometric_contacts,
)

EXPECTED_HEAD = "fca0db8a4230b13865760fa68323d50072c0b737"
EXPECTED_BRANCH = "phase0s-sofa-beamadapter"
CONFIG = SPIKE / "configs" / "stage_c_stable_contact_v2.json"
SCENE = SPIKE / "src" / "stage_c_stable_contact_v2_scene.py"
BRIDGE = SPIKE / "native-contact-bridge" / "src" / "NativeContactBridge" / "NativeContactBridge.cpp"
REPORT_DIR = SPIKE / "reports" / "phase0s_sofa"
C0_TRACE = REPORT_DIR / "data" / "stage_c_v2_material_anchor_baseline_trace.json"
C1_TRACE = REPORT_DIR / "data" / "stage_c_v2_stable_contact_trace.json"
OLD_METRICS = REPORT_DIR / "stage_c_stable_contact_v2_metrics.json"
OUT_METRICS = REPORT_DIR / "stage_c_contact_semantics_v2_1_metrics.json"
OUT_REPORT = REPORT_DIR / "STAGE_C_CONTACT_SEMANTICS_V2_1.md"
EXPECTED_BLOBS = {
    CONFIG: "1aaab0991983b65c2acf7613b173c5920615e86c",
    SCENE: "96df7abc3a72bf2221bf5393424203d432ac0c82",
    BRIDGE: "e6fb41f9b903e53c33583a3cbde3334f9c117dbb",
    C0_TRACE: "4e00923dfe37b7ffa6bf778095fa9e7994ec2844",
    C1_TRACE: "e1aa1477d5fe70b92a441a55a2225d85a016825e",
    OLD_METRICS: "9f89484d2b862793b2d12790ddde2d80ccef1fe6",
}
SOFA_RELEASE = {
    "tag": "v26.06.00",
    "commit": "7c18e95d5c5f2839079892c69e7d89a313c79603",
    "local_min_distance_source": "Sofa/Component/Collision/Detection/Intersection/src/sofa/component/collision/detection/intersection/LocalMinDistance.cpp",
    "collision_model_source": "Sofa/framework/Core/src/sofa/core/CollisionModel.cpp",
    "intersector": "TriangleCollisionModel<Vec3> <-> PointCollisionModel<Vec3>",
    "model_contact_distance_default_m": 0.0,
}


def _git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _input_provenance() -> dict:
    actual_blobs = {str(path.relative_to(ROOT)): _git("hash-object", str(path)) for path in EXPECTED_BLOBS}
    expected_blobs = {str(path.relative_to(ROOT)): expected for path, expected in EXPECTED_BLOBS.items()}
    head, branch, status = _git("rev-parse", "HEAD"), _git("branch", "--show-current"), _git("status", "--short")
    return {
        "head": head, "required_head": EXPECTED_HEAD,
        "branch": branch, "required_branch": EXPECTED_BRANCH,
        "working_tree_status_at_audit": status.splitlines() if status else [],
        "blob_hashes": actual_blobs, "expected_blob_hashes": expected_blobs,
        "trace_sha256": {str(C0_TRACE.relative_to(ROOT)): _sha256(C0_TRACE), str(C1_TRACE.relative_to(ROOT)): _sha256(C1_TRACE)},
        "pass": bool(head == EXPECTED_HEAD and branch == EXPECTED_BRANCH and actual_blobs == expected_blobs),
    }


def _episode(records: list[dict], classifications: list[dict], key: str) -> dict:
    return contact_episode_stats_timestamps([float(r["step_end_time_s"]) for r in records], [bool(c[key]) for c in classifications])


def _fraction(values: list[bool]) -> float:
    return float(np.mean(values)) if values else 0.0


def _gap_summary(classifications: list[dict]) -> dict:
    values = np.asarray([gap for c in classifications for gap in c["local_min_distance_gaps_m"]], dtype=float)
    if not values.size:
        return {"samples": 0, "min_m": None, "p10_m": None, "p50_m": None, "p90_m": None, "max_m": None}
    return {"samples": int(values.size), "min_m": float(values.min()), "p10_m": float(np.percentile(values, 10)), "p50_m": float(np.percentile(values, 50)), "p90_m": float(np.percentile(values, 90)), "max_m": float(values.max())}


def _zero_load_semantics(records: list[dict], classifications: list[dict]) -> dict:
    return {
        "samples": len(records),
        "native_detection_frames": int(sum(c["native_detection"] for c in classifications)),
        "native_detection_count_total": int(sum(c["native_detection_count"] for c in classifications)),
        "proximity_only_frames": int(sum(c["proximity_only"] for c in classifications)),
        "geometric_contact_frames": int(sum(c["geometric_contact"] for c in classifications)),
        "lcp_rows_present_frames": int(sum(c["lcp_rows_present"] for c in classifications)),
        "reaction_active_frames": int(sum(c["reaction_active"] for c in classifications)),
        "gap_summary": _gap_summary(classifications),
        "pass": bool(not any(c["geometric_contact"] for c in classifications) and not any(c["reaction_active"] for c in classifications)),
    }


def _measurement_contact_gate(records: list[dict], classifications: list[dict], config: dict) -> dict:
    gates = config["gates"]
    detection_episode = _episode(records, classifications, "native_detection")
    geometric_episode = _episode(records, classifications, "geometric_contact")
    load_bearing_episode = _episode(records, classifications, "load_bearing_contact")
    required_occupancy = float(gates["minimum_measurement_contact_occupancy"])
    required_dwell_s = float(gates["minimum_observed_contact_dwell_ms"]) / 1000.0
    return {
        "native_detection_episode": detection_episode,
        "geometric_contact_episode": geometric_episode,
        "load_bearing_episode": load_bearing_episode,
        "gap_summary": _gap_summary(classifications),
        "required_geometric_occupancy": required_occupancy,
        "required_geometric_dwell_s": required_dwell_s,
        "pass": bool(geometric_episode["occupancy"] >= required_occupancy and geometric_episode["longest_duration_s"] + 1e-12 >= required_dwell_s),
    }


def _reaction_gate(all_records, all_classifications, measurement_records, measurement_classifications, config, reaction_floor):
    geometric_pairs = [(r, c) for r, c in zip(measurement_records, measurement_classifications) if c["geometric_contact"]]
    lcp_fraction = _fraction([c["lcp_rows_present"] for _, c in geometric_pairs])
    positive_fraction = _fraction([c["reaction_active"] for _, c in geometric_pairs])
    reactions = np.asarray([float(r["reaction_force_proxy"]) for r, _ in geometric_pairs], dtype=float)
    geometric_onset = first_active_time_s(all_records, all_classifications, "geometric_contact")
    reaction_onset = first_active_time_s(all_records, all_classifications, "reaction_active")
    onset_lag_s = abs(float(reaction_onset) - float(geometric_onset)) if geometric_onset is not None and reaction_onset is not None else None
    no_detection_pairs = [(r, c) for r, c in zip(all_records, all_classifications) if not c["native_detection"]]
    clean = all(int(r["constraint_vector_size"]) == 0 and float(r["reaction_force_proxy"]) <= reaction_floor for r, _ in no_detection_pairs)
    gates = config["gates"]
    allowed_lag_s = float(gates["maximum_native_lcp_onset_lag_steps"]) * float(config["simulation"]["dt_s"])
    passed = bool(geometric_pairs and lcp_fraction >= float(gates["minimum_contact_lcp_fraction"]) and positive_fraction >= float(gates["minimum_contact_positive_reaction_fraction"]) and reactions.size and float(np.percentile(reactions, 10)) > reaction_floor and onset_lag_s is not None and onset_lag_s <= allowed_lag_s and clean)
    return {"evaluated": True, "geometric_measurement_frames": len(geometric_pairs), "lcp_fraction_on_geometric_contact": lcp_fraction, "positive_reaction_fraction_on_geometric_contact": positive_fraction, "reaction_p10": float(np.percentile(reactions, 10)) if reactions.size else None, "reaction_p50": float(np.percentile(reactions, 50)) if reactions.size else None, "reaction_p90": float(np.percentile(reactions, 90)) if reactions.size else None, "reaction_max": float(reactions.max()) if reactions.size else None, "reaction_floor_c0_p99": float(reaction_floor), "geometric_onset_s": geometric_onset, "reaction_onset_s": reaction_onset, "onset_lag_s": onset_lag_s, "allowed_onset_lag_s": allowed_lag_s, "no_detection_reaction_clean": bool(clean), "pass": passed}


def _geometry_gate(records, classifications, config, effective_contact_distance_m):
    contacts = selected_geometric_contacts(records, classifications, effective_contact_distance_m)
    if not contacts:
        return {"evaluated": True, "samples": 0, "pass": False}
    normals = np.asarray([c["beam_outward_normal"] for c in contacts], dtype=float)
    fixture_points = np.asarray([c["fixture_contact_point_m"] for c in contacts], dtype=float)
    unique_per_frame, x_span_per_frame = [], []
    for record, classification in zip(records, classifications):
        if not classification["geometric_contact"]:
            continue
        geometric = [c for c, gap in zip(record["native_contacts"], classification["local_min_distance_gaps_m"]) if gap <= 0.0]
        unique_per_frame.append(len({int(c["beam_element_id"]) for c in geometric}))
        xs = [float(c["beam_contact_point_m"][0]) for c in geometric]
        x_span_per_frame.append(float(np.ptp(xs)) if xs else 0.0)
    gates = config["gates"]
    plane_error = float(np.max(np.abs(fixture_points[:, 1] - float(config["contact_wall"]["plane_y_m"]))))
    alignment = float(np.percentile(np.abs(normals[:, 1]), 10))
    primitives = float(np.percentile(unique_per_frame, 95))
    span = float(np.percentile(x_span_per_frame, 95))
    passed = bool(plane_error <= float(gates["maximum_fixture_plane_error_m"]) and alignment >= float(gates["minimum_normal_alignment_p10"]) and primitives <= float(gates["maximum_unique_beam_primitives_p95"]) and span <= float(gates["maximum_contact_x_span_p95_m"]))
    return {"evaluated": True, "samples": len(contacts), "fixture_plane_error_max_m": plane_error, "normal_alignment_p10": alignment, "unique_beam_primitives_p95": primitives, "contact_x_span_p95_m": span, "pass": passed}


def _relation_table(records, classifications):
    counts = {}
    for record, classification in zip(records, classifications):
        key = str((int(classification["native_detection_count"]), int(classification["geometric_contact_count"]), int(record["constraint_vector_size"]), bool(classification["reaction_active"])))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _write_outputs(metrics):
    OUT_METRICS.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    OUT_REPORT.write_text("# Phase 0S-SOFA Stage C Contact Semantics V2.1\n\n## Verdict\n\n`%s`\n\n## Scope\n\nOffline-only re-audit of immutable Stage-C V2 C0/C1 traces. No simulator execution and no physics/configuration change occurred.\n\n## Metrics\n\n```json\n%s\n```\n\n## Fact\n\n%s\n\n## Inference\n\n%s\n\n## Unknown\n\n%s\n\n## Next action\n\n%s\n" % (metrics["verdict"], json.dumps(metrics, indent=2), metrics["fact"], metrics["inference"], metrics["unknown"], metrics["next_action"]), encoding="utf-8")


def main() -> int:
    provenance = _input_provenance()
    metrics = {"stage": "Phase 0S-SOFA Stage C V2.1", "offline_only": True, "simulator_executed": False, "physics_changed": False, "config_changed": False, "native_bridge_changed": False, "stage_d_executed": False, "stages_e_f_executed": False, "oracle_fitted": False, "oracle_frozen": False, "input_provenance": provenance, "sofa_release": SOFA_RELEASE}
    if not provenance["pass"]:
        metrics.update(verdict="PHASE0S_SOFA_STAGE_C_V2_1_INPUT_INVALID", fact="The frozen Stage-C V2 branch, HEAD, or input blob identity check failed.", inference="No contact-semantics reinterpretation is permitted from non-frozen inputs.", unknown="All Stage-C V2.1 science outcomes remain unknown.", next_action="Restore or explicitly re-qualify the exact frozen Stage-C V2 inputs.")
        _write_outputs(metrics); print(metrics["verdict"]); return 2
    config = json.loads(CONFIG.read_text(encoding="utf-8")); c0_trace = json.loads(C0_TRACE.read_text(encoding="utf-8")); c1_trace = json.loads(C1_TRACE.read_text(encoding="utf-8")); old = json.loads(OLD_METRICS.read_text(encoding="utf-8"))
    if not isinstance(c0_trace.get("records"), list) or not isinstance(c1_trace.get("records"), list):
        raise RuntimeError("Stage-C trace records missing")
    if not bool(old["c0_instrumentation"]["pass"] and old["c0_baseline"]["pass"] and old["c1_instrumentation"]["pass"]):
        raise RuntimeError("Historical Stage-C V2 provenance prerequisite is not PASS")
    intersection = float(config["simulation"]["contact_distance_m"]); model = float(SOFA_RELEASE["model_contact_distance_default_m"]); effective = intersection + model + model
    if effective != 0.002: raise RuntimeError("Frozen effective contact distance is not exactly 0.002 m")
    reaction_floor = float(old["c0_baseline"]["reaction_p99"])
    records = c1_trace["records"]; classifications = [classify_record(r, effective_contact_distance_m=effective, reaction_floor=reaction_floor) for r in records]
    zero_pairs = [(r, c) for r, c in zip(records, classifications) if bool(r["zero_load_phase"])]
    measurement_pairs = [(r, c) for r, c in zip(records, classifications) if bool(r["measurement_active"])]
    zero_records, zero_classifications = [r for r, _ in zero_pairs], [c for _, c in zero_pairs]
    measurement_records, measurement_classifications = [r for r, _ in measurement_pairs], [c for _, c in measurement_pairs]
    zero = _zero_load_semantics(zero_records, zero_classifications); contact = _measurement_contact_gate(measurement_records, measurement_classifications, config)
    metrics["effective_contact_distance"] = {"intersection_contact_distance_m": intersection, "beam_model_contact_distance_m": model, "fixture_model_contact_distance_m": model, "effective_contact_distance_m": effective, "source_qualified": True}
    metrics["historical_stage_c_v2"] = {"verdict": old["verdict"], "c0_instrumentation_pass": bool(old["c0_instrumentation"]["pass"]), "c0_baseline_pass": bool(old["c0_baseline"]["pass"]), "c1_instrumentation_pass": bool(old["c1_instrumentation"]["pass"]), "note": "Historical evidence is preserved. V2.1 replaces only the analysis-layer equation DetectionOutput == physical contact."}
    metrics["c1_zero_load_semantics"] = zero; metrics["c1_measurement_contact"] = contact; metrics["c1_relation"] = _relation_table(records, classifications)
    metrics["onsets"] = {key: first_active_time_s(records, classifications, key) for key in ("native_detection", "geometric_contact", "reaction_active", "load_bearing_contact")}
    if not zero["pass"]:
        metrics["reaction"] = {"evaluated": False, "reason": "corrected zero-load geometric-contact/reaction gate failed"}; metrics["geometry"] = {"evaluated": False, "reason": "corrected zero-load geometric-contact/reaction gate failed"}
        metrics.update(verdict="PHASE0S_SOFA_STAGE_C_CONTACT_CONSTRUCTION_FAIL", fact="Stage-C V2 provenance remains qualified, but corrected LocalMinDistance semantics show geometric contact or positive reaction during the nominal zero-load phase.", inference="The frozen C1 construction does not provide the required contact-free baseline.", unknown="Sustained loaded contact, reaction coupling, contact geometry, breakaway, stick/slip, jam, Oracle calibration, held-out capability, and passage remain unqualified.", next_action="Design one bounded Stage-C contact-construction correction while keeping material, friction, dt, solver, Oracle thresholds, and later stages frozen.")
    elif not contact["pass"]:
        metrics["reaction"] = {"evaluated": False, "reason": "sustained geometric-contact prerequisite failed"}; metrics["geometry"] = {"evaluated": False, "reason": "sustained geometric-contact prerequisite failed"}
        metrics.update(verdict="PHASE0S_SOFA_STAGE_C_CONTACT_CONSTRUCTION_FAIL", fact="Stage-C V2 provenance remains qualified and corrected zero-load semantics are contact-free, but the immutable measurement window fails the frozen geometric-contact occupancy/dwell gate.", inference="The previous zero-load failure was a proximity-detection semantic error; the actual frozen C1 construction still fails to sustain the required wall contact.", unknown="Loaded-contact reaction coupling, contact geometry, breakaway, stick/slip, jam, Oracle calibration, held-out capability, and passage remain unqualified.", next_action="Design one bounded Stage-C contact-construction correction while keeping material, friction, dt, solver, Oracle thresholds, and later stages frozen.")
    else:
        reaction = _reaction_gate(records, classifications, measurement_records, measurement_classifications, config, reaction_floor); geometry = _geometry_gate(measurement_records, measurement_classifications, config, effective); metrics["reaction"] = reaction; metrics["geometry"] = geometry
        if not reaction["pass"]:
            metrics.update(verdict="PHASE0S_SOFA_STAGE_C_REACTION_PROXY_FAIL", fact="Corrected LocalMinDistance semantics pass zero-load and sustained-contact gates, but reaction coupling fails the frozen Stage-C criteria.", inference="The global LCP reaction proxy is not qualified for the current real wall-contact regime.", unknown="Breakaway, stick/slip, jam, Oracle calibration, held-out capability, and passage remain unqualified.", next_action="Repair only the Stage-C reaction-coupling measurement path without changing physics or later-stage thresholds.")
        elif not geometry["pass"]:
            metrics.update(verdict="PHASE0S_SOFA_STAGE_C_GEOMETRY_FAIL", fact="Corrected LocalMinDistance semantics and reaction coupling pass, but the frozen contact-geometry gate fails.", inference="The current contact patch is not geometrically controlled enough for Stage D.", unknown="Breakaway, stick/slip, jam, Oracle calibration, held-out capability, and passage remain unqualified.", next_action="Repair only the Stage-C contact geometry while keeping material, friction, dt, solver, and later-stage thresholds frozen.")
        else:
            metrics.update(verdict="PHASE0S_SOFA_STAGE_C_STABLE_CONTACT_PASS", fact="Corrected LocalMinDistance semantics, zero-load baseline, sustained geometric contact, reaction coupling, and contact geometry all pass.", inference="Stage C is qualified under the frozen SOFA Point/Triangle construction.", unknown="Breakaway, stick/slip, jam, Oracle calibration, held-out capability, and passage remain unqualified.", next_action="Execute the preregistered single monotonic Stage-D breakaway characterization.")
    _write_outputs(metrics); print(metrics["verdict"]); return 0


if __name__ == "__main__":
    raise SystemExit(main())
