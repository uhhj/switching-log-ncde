"""Gated Phase 0S R1 controlled-contact construction audit."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
sys.path.insert(0, str(SPIKE / "src"))
from contact_native import (  # noqa: E402
    contact_episode_stats,
    derived_excitation,
    find_breakaway,
    percentile,
    progress_windows,
    reaction_proxy_clean,
)


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _run_scene(config_path: Path, config: dict, mode: str, steps: int, work_dir: Path, forward_force: float = 0.0) -> dict:
    trace = work_dir / f"{mode}.json"
    command = [
        "docker", "run", "--rm", "--network", "none",
        "-e", "PYTHONPATH=/work/simulator-spikes/sofa-beam-spike/src",
        "-e", f"SOFA_CONTACT_R1_CONFIG=/work/{config_path.relative_to(ROOT).as_posix()}",
        "-e", f"SOFA_CONTACT_R1_MODE={mode}",
        "-e", f"SOFA_CONTACT_R1_STEPS={steps}",
        "-e", f"SOFA_CONTACT_R1_FORWARD_FORCE={forward_force:.17g}",
        "-e", f"SOFA_CONTACT_R1_TRACE=/work/{trace.relative_to(ROOT).as_posix()}",
        "-v", f"{ROOT}:/work",
        "-v", f"{config['runtime']['sofa_root']}:/sofa:ro",
        config["runtime"]["image"],
        "/sofa/bin/runSofa", "-l", "SofaPython3", "-g", "batch", "-n", str(steps),
        "/work/simulator-spikes/sofa-beam-spike/src/calibration_scene.py",
    ]
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (work_dir / f"{mode}.log").write_text(completed.stdout)
    if completed.returncode != 0 or not trace.exists():
        raise RuntimeError(f"{mode} did not save a trace")
    if "[ERROR]" in completed.stdout or "NaN" in completed.stdout:
        raise RuntimeError(f"{mode} emitted SOFA ERROR/NaN")
    return json.loads(trace.read_text())


def _measurement(records: list[dict]) -> list[dict]:
    return [record for record in records if record["measurement_active"]]


def _stats(records: list[dict], dt_s: float) -> dict:
    measured = _measurement(records)
    contact_counts = [record["native_contact_count"] for record in measured]
    mask = [count > 0 for count in contact_counts]
    episode = contact_episode_stats(mask, dt_s)
    contacts = [record for record in measured if record["native_contact_count"] > 0]
    result = {
        "samples": len(records),
        "measurement_samples": len(measured),
        "native_contact": episode,
        "constraint_vector_size_max": max((record["constraint_vector_size"] for record in records), default=0),
        "reaction_p10": percentile((record["reaction_force_proxy"] for record in contacts), 10) if contacts else None,
        "reaction_p50": percentile((record["reaction_force_proxy"] for record in contacts), 50) if contacts else None,
        "reaction_p90": percentile((record["reaction_force_proxy"] for record in contacts), 90) if contacts else None,
        "native_tangent_p05": percentile((record["native_contact_tangent_speed_median"] for record in contacts), 5) if contacts else None,
        "native_tangent_p50": percentile((record["native_contact_tangent_speed_median"] for record in contacts), 50) if contacts else None,
        "native_tangent_p95": percentile((record["native_contact_tangent_speed_median"] for record in contacts), 95) if contacts else None,
        "native_tangent_travel_m": float(sum(record["native_contact_tangent_speed_median"] for record in contacts) * dt_s),
        "endpoint_tangent_p95": percentile((record["endpoint_tangent_speed_mps"] for record in measured), 95) if measured else None,
        "nearest_plane_tangent_p95": percentile((record["nearest_plane_proxy_tangent_speed_mps"] for record in measured), 95) if measured else None,
        "nearest_node_tangent_p95": percentile((record["nearest_beam_node_proxy_tangent_speed_mps"] for record in measured), 95) if measured else None,
    }
    if measured:
        progress = [record["task_progress_m"] for record in measured]
        if len(progress) > int(round(0.1 / dt_s)):
            windows = progress_windows(progress, dt_s, 100)
            result["progress_p10_at_100ms_m"] = percentile(windows, 10)
            result["progress_p50_at_100ms_m"] = percentile(windows, 50)
            result["progress_p90_at_100ms_m"] = percentile(windows, 90)
    return result


def _report(path: Path, metrics: dict) -> None:
    def value(mapping, name):
        result = mapping.get(name)
        return "n/a" if result is None else f"{result:.12g}" if isinstance(result, float) else str(result)

    stages = metrics["stages"]
    native = metrics.get("native_contact_extraction", {})
    lines = [
        "# Phase 0S-SOFA Controlled Contact Construction R1 Revised", "",
        "## Verdict", "", metrics["verdict"], "",
        "## Native contact extraction", "",
        f"- API/component: {native.get('component', 'ContactListener')}",
        f"- primitive type: {native.get('primitive_type', 'PointCollisionModel')}",
        f"- primitive IDs: {native.get('example_element_ids', [])}",
        f"- contact point: {native.get('example_contact_point_m', 'n/a')}",
        f"- contact normal: {native.get('example_contact_normal', 'n/a')} (from static fixture triangle winding; v26.06 Python does not expose DetectionOutput normals).",
        "- formal velocity source: collision-point velocity indexed by ContactListener beam primitive ID, projected onto wall tangent.",
        "- nearest-plane used for gate: no.", "",
        "## Tail contamination control", "",
        f"- tail constraint: PartialFixedProjectiveConstraint fixedDirections=1 1 1 0 0 0.",
        f"- translation-only: yes.",
        f"- no-wall contact count: {stages.get('anchored_free_control', {}).get('native_contact', {}).get('occupancy', 'n/a')}",
        f"- LCP vector size max: {stages.get('anchored_free_control', {}).get('constraint_vector_size_max', 'n/a')}",
        f"- reaction p99: {value(stages.get('anchored_free_control', {}), 'reaction_p99')}", "",
        "## Stable normal contact", "",
        f"- occupancy: {stages.get('normal_contact_hold', {}).get('native_contact', {}).get('occupancy', 'n/a')}",
        f"- longest: {stages.get('normal_contact_hold', {}).get('native_contact', {}).get('longest_duration_s', 'n/a')} s",
        f"- reaction p10/p50/p90: {value(stages.get('normal_contact_hold', {}), 'reaction_p10')} / {value(stages.get('normal_contact_hold', {}), 'reaction_p50')} / {value(stages.get('normal_contact_hold', {}), 'reaction_p90')}", "",
        "## Breakaway", "",
        f"- ramp max: {metrics['ramp_max_n']:.12g} N", f"- result: {metrics.get('breakaway')}", "",
        "## Derived forces", "", f"- F_stick: {metrics.get('f_stick_n', 'n/a')} N", f"- F_slip: {metrics.get('f_slip_n', 'n/a')} N", "",
        "## Fresh stick", "", str(stages.get('fresh_stick', 'not run')), "",
        "## Fresh slip", "", str(stages.get('fresh_slip', 'not run')), "",
        "## Free-forward", "", str(stages.get('free_forward', 'not run')), "",
        "## Jam", "", str(stages.get('blocked_jam', 'not run')), "",
        "## Separation without Oracle fitting", "", str(metrics.get('separation', 'not reached')), "",
        "## Fact", "", metrics["fact"], "",
        "## Inference", "", metrics["inference"], "",
        "## Unknown", "", metrics["unknown"], "",
        "## Next action", "", metrics["next_action"], "",
    ]
    path.write_text("\n".join(lines))


def _finish(report_dir: Path, work_dir: Path, metrics: dict) -> int:
    (report_dir / "contact_construction_r1_revised_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    _report(report_dir / "CONTACT_CONSTRUCTION_R1_REVISED.md", metrics)
    shutil.rmtree(work_dir, ignore_errors=True)
    return 0


def _fail(metrics: dict, reason: str, *, engineering: bool) -> None:
    metrics["verdict"] = "PHASE0S_SOFA_ENGINEERING_BLOCKED" if engineering else "PHASE0S_SOFA_CONTACT_CONSTRUCTION_FAIL"
    metrics["failure_reason"] = reason
    metrics["fact"] = reason
    metrics["inference"] = "No Oracle threshold was fitted or frozen; later R1 stages were not run."
    metrics["unknown"] = "Held-out capability and unified-passage behavior remain untested."
    metrics["next_action"] = "Review the failed controlled physical separation before any capability or passage experiment."


def _preflight(config_path: Path, config: dict, work_dir: Path) -> dict:
    payload = _run_scene(config_path, config, "native_contact_audit", 20, work_dir)
    records, api = payload["records"], payload["api"]
    contacts = [contact for record in records for contact in record["native_contacts"]]
    required = {"getNumberOfContacts", "getContactElements", "getContactPoints"}
    if not required.issubset(set(api.get("listener_methods", []))) or not contacts:
        raise RuntimeError("v26.06 ContactListener binding did not expose native IDs, points, and normals")
    first = contacts[0]
    if not first["contact_normal"] or first["beam_element_id"] < 0:
        raise RuntimeError("native contact payload lacks primitive identity or normal")
    return {
        "component": "ContactListener",
        "primitive_type": "PointCollisionModel",
        "listener_methods": api["listener_methods"],
        "example_element_ids": [first["beam_element_id"], first["fixture_element_id"]],
        "example_contact_point_m": first["contact_point_m"],
        "example_contact_normal": first["contact_normal"],
        "blocker_clearance_preflight_m": float(config["simulation"]["initial_clearance_m"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--preflight", action="store_true")
    args = parser.parse_args()
    if os.environ.get("CUDA_VISIBLE_DEVICES") not in {"", None}:
        raise RuntimeError("CPU-only R1 requires CUDA_VISIBLE_DEVICES to be empty")
    config_path = args.config.resolve()
    config = _load(config_path)
    if int(config["seed"]) != 84000:
        raise ValueError("R1 is restricted to seed 84000")
    report_dir = SPIKE / "reports" / "phase0s_sofa"
    work_dir = report_dir / ".contact_construction_r1_work"
    shutil.rmtree(work_dir, ignore_errors=True)
    work_dir.mkdir(parents=True)
    try:
        native = _preflight(config_path, config, work_dir)
        if args.preflight:
            print(json.dumps(native, indent=2))
            shutil.rmtree(work_dir)
            return 0
        dt_s = float(config["simulation"]["dt_s"])
        normal = float(config["simulation"]["normal_preload_force_n"])
        radius = float(config["beam"]["radius_m"])
        ramp_max = 2.0 * float(config["simulation"]["friction"]) * normal
        metrics = {"seed": 84000, "cpu_only": True, "native_contact_extraction": native, "ramp_max_n": ramp_max, "stages": {}}

        anchored = _run_scene(config_path, config, "anchored_free_control", 300, work_dir)
        anchored_records = anchored["records"]
        anchored_stats = _stats(anchored_records, dt_s)
        anchored_stats["reaction_p99"] = percentile((record["reaction_force_proxy"] for record in anchored_records), 99)
        metrics["stages"]["anchored_free_control"] = anchored_stats
        if not reaction_proxy_clean([record["native_contact_count"] for record in anchored_records], [record["constraint_vector_size"] for record in anchored_records], [record["reaction_force_proxy"] for record in anchored_records]):
            _fail(metrics, "anchored_free_control shows native contact, nonempty LCP constraints, or nonzero reaction without a wall", engineering=False)
            return _finish(report_dir, work_dir, metrics)

        hold = _run_scene(config_path, config, "normal_contact_hold", 500, work_dir)
        hold_stats = _stats(hold["records"], dt_s)
        metrics["stages"]["normal_contact_hold"] = hold_stats
        episode = hold_stats["native_contact"]
        if not (episode["occupancy"] >= 0.90 and episode["longest_duration_s"] >= 0.300 and hold_stats["reaction_p10"] is not None and hold_stats["reaction_p10"] > anchored_stats["reaction_p99"]):
            _fail(metrics, "normal_contact_hold failed the native occupancy, duration, or reaction-separation gate", engineering=False)
            return _finish(report_dir, work_dir, metrics)

        ramp = _run_scene(config_path, config, "breakaway_ramp", 500, work_dir)
        ramp_records = ramp["records"]
        ramp_stats = _stats(ramp_records, dt_s)
        metrics["stages"]["breakaway_ramp"] = ramp_stats
        breakaway = find_breakaway([record["command_force_n"][0] for record in ramp_records], [record["native_contact_tangent_speed_median"] for record in ramp_records], [record["native_contact_count"] for record in ramp_records], [record["measurement_active"] for record in ramp_records], dt_s=dt_s, window_ms=int(config["simulation"]["breakaway_window_ms"]), radius_m=radius)
        metrics["breakaway"] = breakaway
        if breakaway is None or breakaway["force_n"] > 0.8 * ramp_max:
            _fail(metrics, "one monotonic breakaway ramp did not yield a valid breakaway force within the preregistered bound", engineering=False)
            return _finish(report_dir, work_dir, metrics)

        f_stick, f_slip = derived_excitation(breakaway["force_n"])
        metrics["f_stick_n"], metrics["f_slip_n"] = f_stick, f_slip
        stick = _stats(_run_scene(config_path, config, "fresh_stick", 300, work_dir, f_stick)["records"], dt_s)
        metrics["stages"]["fresh_stick"] = stick
        slip = _stats(_run_scene(config_path, config, "fresh_slip", 300, work_dir, f_slip)["records"], dt_s)
        metrics["stages"]["fresh_slip"] = slip
        stick_gate = stick["native_contact"]["occupancy"] >= 0.90 and stick["native_contact"]["longest_duration_s"] >= 0.200 and stick["native_tangent_travel_m"] <= 0.25 * radius
        slip_gate = slip["native_contact"]["occupancy"] >= 0.90 and slip["native_contact"]["longest_duration_s"] >= 0.200 and slip["native_tangent_travel_m"] >= radius
        speed_gate = stick["native_tangent_p95"] is not None and slip["native_tangent_p05"] is not None and stick["native_tangent_p95"] < slip["native_tangent_p05"]
        metrics["separation"] = {"reaction": True, "stick_slip_native_speed": bool(speed_gate)}
        if not (stick_gate and slip_gate and speed_gate):
            _fail(metrics, "fresh matched stick/slip failed native occupancy, travel, or native-speed separation", engineering=False)
            return _finish(report_dir, work_dir, metrics)

        free = _stats(_run_scene(config_path, config, "free_forward", 300, work_dir, f_slip)["records"], dt_s)
        jam = _stats(_run_scene(config_path, config, "blocked_jam", 300, work_dir, f_slip)["records"], dt_s)
        metrics["stages"]["free_forward"], metrics["stages"]["blocked_jam"] = free, jam
        jam_gate = jam["native_contact"]["occupancy"] >= 0.90 and jam["native_contact"]["longest_duration_s"] >= 0.200
        progress_gate = free.get("progress_p50_at_100ms_m", 0.0) > 0.0 and jam.get("progress_p90_at_100ms_m", float("inf")) < free.get("progress_p10_at_100ms_m", -float("inf"))
        metrics["separation"]["jam_free_progress"] = bool(progress_gate)
        if not (jam_gate and progress_gate):
            _fail(metrics, "shared-force blocked-jam failed native blocker contact or free/jam progress separation", engineering=False)
            return _finish(report_dir, work_dir, metrics)

        metrics.update({"verdict": "PHASE0S_SOFA_CONTACT_CONSTRUCTION_PASS", "fact": "All R1 construction gates passed with native contact identity and native contact-point tangent speed.", "inference": "The controlled construction is valid for rerunning, but not yet freezing, the SOFA Oracle.", "unknown": "Held-out capability and unified-passage behavior remain untested.", "next_action": "Rerun the one-shot seed-84000 SOFA Oracle calibration using the validated native contact signal, native contact-point tangent velocity, derived fresh stick/slip excitation, and validated free-forward/blocked-jam construction."})
        return _finish(report_dir, work_dir, metrics)
    except RuntimeError as error:
        metrics = {"seed": 84000, "cpu_only": True, "ramp_max_n": 2.0 * float(config["simulation"]["friction"]) * float(config["simulation"]["normal_preload_force_n"]), "stages": {}}
        _fail(metrics, str(error), engineering=True)
        return _finish(report_dir, work_dir, metrics)


if __name__ == "__main__":
    raise SystemExit(main())
