"""Offline-only Phase-0S Stage-C V2.1-L contact-loss morphology audit."""
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

from stage_c_contact_loss_morphology import (  # noqa: E402
    VERDICT_AMBIGUOUS, VERDICT_CONTACT_MIGRATION, VERDICT_CROSS_THROUGH,
    VERDICT_FIXTURE_EDGE_ESCAPE, VERDICT_REACTION_COLLAPSE,
    VERDICT_REBOUND_ESCAPE, classify_contact_loss_morphology,
)
from stage_c_contact_semantics import classify_record  # noqa: E402

BASE_HEAD = "69f6bf95152d8f89678033be7d13515cafc6a39b"
EXPECTED_BRANCH = "phase0s-sofa-beamadapter"
EXPECTED_TRACE_SHA256 = "9e4b8680978286cb020aa52a64af1376c748394c39d34d19062929316c294df3"
CONFIG = SPIKE / "configs" / "stage_c_stable_contact_v2.json"
SCENE = SPIKE / "src" / "stage_c_stable_contact_v2_scene.py"
SEMANTICS = SPIKE / "src" / "stage_c_contact_semantics.py"
TRACE = SPIKE / "reports" / "phase0s_sofa" / "data" / "stage_c_v2_stable_contact_trace.json"
OLD_METRICS = SPIKE / "reports" / "phase0s_sofa" / "stage_c_stable_contact_v2_metrics.json"
V21_METRICS = SPIKE / "reports" / "phase0s_sofa" / "stage_c_contact_semantics_v2_1_metrics.json"
REPORT_DIR = SPIKE / "reports" / "phase0s_sofa"
OUT_METRICS = REPORT_DIR / "stage_c_contact_loss_morphology_v2_1l_metrics.json"
OUT_REPORT = REPORT_DIR / "STAGE_C_CONTACT_LOSS_MORPHOLOGY_V2_1L.md"
IMPLEMENTATION_FILES = {
    "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py",
    "simulator-spikes/sofa-beam-spike/scripts/audit_stage_c_contact_loss_morphology_v2_1l.py",
    "simulator-spikes/sofa-beam-spike/tests/test_stage_c_contact_loss_morphology_v2_1l.py",
}
EXPECTED_BLOBS = {CONFIG: "1aaab0991983b65c2acf7613b173c5920615e86c", SCENE: "96df7abc3a72bf2221bf5393424203d432ac0c82", SEMANTICS: "a130d83e11ba4702c44b94344f766f44df6abbc7", TRACE: "e1aa1477d5fe70b92a441a55a2225d85a016825e", OLD_METRICS: "9f89484d2b862793b2d12790ddde2d80ccef1fe6", V21_METRICS: "f4bac4d185b820fb2ec70900f82312013f748fc9"}
VALID_VERDICTS = {VERDICT_CROSS_THROUGH, VERDICT_REBOUND_ESCAPE, VERDICT_FIXTURE_EDGE_ESCAPE, VERDICT_CONTACT_MIGRATION, VERDICT_REACTION_COLLAPSE, VERDICT_AMBIGUOUS}


def _git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _provenance() -> dict:
    head, parent = _git("rev-parse", "HEAD"), _git("rev-parse", "HEAD^")
    branch, status = _git("branch", "--show-current"), _git("status", "--short")
    changed = set(filter(None, _git("diff", "--name-only", f"{BASE_HEAD}..HEAD").splitlines()))
    actual = {str(path.relative_to(ROOT)): _git("hash-object", str(path)) for path in EXPECTED_BLOBS}
    expected = {str(path.relative_to(ROOT)): value for path, value in EXPECTED_BLOBS.items()}
    trace_sha = _sha256(TRACE)
    return {"base_head": BASE_HEAD, "implementation_head": head, "implementation_parent": parent, "implementation_parent_matches_base": bool(parent == BASE_HEAD), "branch": branch, "required_branch": EXPECTED_BRANCH, "working_tree_status_at_audit_start": status.splitlines() if status else [], "working_tree_clean_at_audit_start": not bool(status), "implementation_changed_files": sorted(changed), "required_implementation_files": sorted(IMPLEMENTATION_FILES), "implementation_scope_exact": bool(changed == IMPLEMENTATION_FILES), "blob_hashes": actual, "expected_blob_hashes": expected, "trace_sha256": trace_sha, "expected_trace_sha256": EXPECTED_TRACE_SHA256, "pass": bool(branch == EXPECTED_BRANCH and parent == BASE_HEAD and not status and changed == IMPLEMENTATION_FILES and actual == expected and trace_sha == EXPECTED_TRACE_SHA256)}


def _validate_trace(records: list[dict], config: dict) -> dict:
    expected_steps, dt = int(config["simulation"]["contact_steps"]), float(config["simulation"]["dt_s"])
    tolerance = float(config["integrity"]["time_tolerance_s"])
    measurement_start = float(config["simulation"]["measurement_start_ms"]) / 1000.0
    if len(records) != expected_steps: raise RuntimeError(f"expected {expected_steps} records, found {len(records)}")
    ends = np.asarray([float(record["step_end_time_s"]) for record in records], dtype=float)
    if not np.all(np.isfinite(ends)) or (ends.size > 1 and not np.all(np.abs(np.diff(ends) - dt) <= tolerance)): raise RuntimeError("frozen trace time increments do not match dt")
    expected_measurement = np.asarray([float(r["step_start_time_s"]) >= measurement_start for r in records], dtype=bool)
    actual_measurement = np.asarray([bool(r["measurement_active"]) for r in records], dtype=bool)
    if not np.array_equal(expected_measurement, actual_measurement): raise RuntimeError("measurement_active does not match frozen schedule")
    nodes = {int(np.asarray(r["beam_node_positions_m"], dtype=float).shape[0]) for r in records}
    expected_nodes = int(config["beam"]["nodes"])
    if nodes != {expected_nodes}: raise RuntimeError(f"expected node count {expected_nodes}, observed {nodes}")
    return {"records": len(records), "expected_records": expected_steps, "dt_s": dt, "first_end_time_s": float(ends[0]), "last_end_time_s": float(ends[-1]), "measurement_frames": int(np.count_nonzero(actual_measurement)), "node_count": expected_nodes, "pass": True}


def _narrative(verdict: str) -> dict:
    if verdict == VERDICT_CROSS_THROUGH:
        return {"fact": "The final contact-loss episode contains a footprint-continuous positive-to-negative wall crossing, including gradual crossings through the plane deadband.", "inference": "The dominant Stage-C construction failure is cross-through of the frozen zero-thickness collision representation.", "unknown": "The correct radius-carrying collision representation and its exact SOFA v26.06 contact-distance composition remain unqualified; all later mode/capability questions remain unqualified.", "next_action": "STOP. Next, verify SOFA v26.06 radius/contactDistance composition and design exactly one Stage-C V2.2 cross-through correction; do not run Stage D/E/F."}
    if verdict == VERDICT_REBOUND_ESCAPE:
        return {"fact": "No through-wall crossing is present; after formal geometric onset the terminal beam is on the positive side outside the effective envelope with native contact absent.", "inference": "The dominant Stage-C construction failure is rebound/escape under the frozen construction.", "unknown": "The responsible preload/initialization detail and all later mode/capability questions remain unqualified.", "next_action": "STOP. Design one bounded Stage-C V2.2 preload/construction correction only; do not sweep parameters or run later stages."}
    if verdict == VERDICT_FIXTURE_EDGE_ESCAPE:
        return {"fact": "After formal contact onset, all beam nodes leave the finite fixture x/z footprint before the terminal frame.", "inference": "The finite fixture footprint is the dominant Stage-C contact-loss mechanism.", "unknown": "Reaction coupling and all later mode/capability questions remain unqualified.", "next_action": "STOP. Design one bounded fixture-extent correction only; keep all other frozen physics unchanged."}
    if verdict == VERDICT_CONTACT_MIGRATION:
        return {"fact": "The terminal native-loss episode remains inside the diagnostic effective envelope for at least the frozen Stage-C minimum observed-contact dwell; earlier recovered native-loss anomalies were excluded.", "inference": "Collision representation/primitive migration is the dominant Stage-C contact-loss suspect; only the sustained terminal suffix was used.", "unknown": "The exact replacement collision representation and all later mode/capability questions remain unqualified.", "next_action": "STOP. Design one collision-representation correction only after verifying the relevant SOFA v26.06 semantics."}
    if verdict == VERDICT_REACTION_COLLAPSE:
        return {"fact": "The final geometric-contact suffix remains reactionless for at least the frozen Stage-C minimum observed-contact dwell, with no stronger geometric escape mechanism.", "inference": "Constraint/contact-response coupling is the dominant Stage-C contact-loss suspect; earlier recovered reaction gaps were excluded.", "unknown": "The exact coupling fault and all later mode/capability questions remain unqualified.", "next_action": "STOP. Audit and correct one contact-response/constraint-coupling path only; do not change geometry or later-stage gates."}
    return {"fact": "The frozen C1 trace does not uniquely identify one preregistered final contact-loss morphology.", "inference": "A physical Stage-C correction is not scientifically justified from this trace alone.", "unknown": "The dominant contact-loss mechanism and all later-stage questions remain unresolved.", "next_action": "STOP. Do not change physics; inspect the emitted evidence and authorize at most one additional bounded observation if it is needed to disambiguate the final loss episode."}


def _write_outputs(metrics: dict) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_METRICS.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    OUT_REPORT.write_text("# Phase 0S-SOFA Stage C V2.1-L Contact-Loss Morphology Audit\n\n## Verdict\n\n`%s`\n\n## Scope\n\nOffline-only adjudication of the immutable Stage-C V2 C1 trace. No SOFA, Docker, GPU, trace regeneration, physics change, Oracle fitting, or later-stage execution occurred.\n\n## Metrics\n\n~~~json\n%s\n~~~\n\n## Fact\n\n%s\n\n## Inference\n\n%s\n\n## Unknown\n\n%s\n\n## Next action\n\n%s\n" % (metrics["verdict"], json.dumps(metrics, indent=2), metrics["fact"], metrics["inference"], metrics["unknown"], metrics["next_action"]), encoding="utf-8")


def main() -> int:
    provenance = _provenance()
    metrics = {"stage": "Phase 0S-SOFA Stage C V2.1-L", "offline_only": True, "sofa_executed": False, "docker_executed": False, "gpu_used": False, "trace_regenerated": False, "physics_changed": False, "config_changed": False, "scene_changed": False, "native_bridge_changed": False, "stage_d_executed": False, "stages_e_f_executed": False, "oracle_fitted": False, "oracle_frozen": False, "model_training_executed": False, "offline_geometry_surrogate_is_formal_contact_signal": False, "input_provenance": provenance}
    if not provenance["pass"]:
        metrics.update(verdict="PHASE0S_SOFA_STAGE_C_LOSS_MORPHOLOGY_INPUT_INVALID", fact="The frozen Stage-C V2.1-L implementation-parent, implementation-scope, clean-worktree, branch, blob, or trace identity gate failed.", inference="No morphology conclusion is permitted from an invalid execution state.", unknown="The contact-loss mechanism remains unknown.", next_action="Restore BASE_HEAD, rebuild one clean direct-child implementation commit, and rerun the offline audit only.")
        _write_outputs(metrics); print(metrics["verdict"]); return 2
    config = json.loads(CONFIG.read_text(encoding="utf-8")); trace = json.loads(TRACE.read_text(encoding="utf-8")); old = json.loads(OLD_METRICS.read_text(encoding="utf-8")); v21 = json.loads(V21_METRICS.read_text(encoding="utf-8"))
    if v21.get("verdict") != "PHASE0S_SOFA_STAGE_C_CONTACT_CONSTRUCTION_FAIL": raise RuntimeError("Stage-C V2.1 prerequisite verdict is not the frozen construction FAIL")
    if not isinstance(trace.get("records"), list) or trace.get("metadata", {}).get("mode") != "stable_contact": raise RuntimeError("frozen C1 trace invalid")
    records = trace["records"]; integrity = _validate_trace(records, config); effective = float(config["simulation"]["contact_distance_m"])
    if not np.isclose(effective, 0.002, atol=0.0, rtol=0.0): raise RuntimeError("frozen effective contact distance is not exactly 0.002 m")
    floor = float(old["c0_baseline"]["reaction_p99"])
    classifications = [classify_record(r, effective_contact_distance_m=effective, reaction_floor=floor) for r in records]
    morphology = classify_contact_loss_morphology(records, classifications, config=config, effective_contact_distance_m=effective)
    verdict = morphology["verdict"]
    if verdict not in VALID_VERDICTS: raise RuntimeError(f"unexpected morphology verdict: {verdict}")
    measurement = [c for r, c in zip(records, classifications) if bool(r["measurement_active"])]
    metrics.update(frozen_stage_c_v2_1_verdict=v21["verdict"], trace_integrity=integrity, effective_contact_distance_m=effective, reaction_floor_c0_p99=floor, formal_measurement_summary={"frames": len(measurement), "native_detection_frames": int(sum(c["native_detection"] for c in measurement)), "geometric_contact_frames": int(sum(c["geometric_contact"] for c in measurement)), "load_bearing_contact_frames": int(sum(c["load_bearing_contact"] for c in measurement))}, morphology=morphology, verdict=verdict, **_narrative(verdict))
    _write_outputs(metrics); print(verdict); return 0


if __name__ == "__main__":
    raise SystemExit(main())
