"""Run only the controlled seed-84000 SOFA Oracle calibration."""

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
from oracle import calibrate  # noqa: E402


MODES = (
    "free_calibration",
    "stick_calibration",
    "slip_calibration",
    "free_forward_calibration",
    "jam_calibration",
)


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _run_scene(config_path: Path, config: dict, mode: str, work_dir: Path) -> list[dict]:
    trace_path = work_dir / f"{mode}.json"
    scene_path = "/work/simulator-spikes/sofa-beam-spike/src/calibration_scene.py"
    command = [
        "docker", "run", "--rm", "--network", "none",
        "-e", f"SOFA_CALIBRATION_CONFIG=/work/{config_path.relative_to(ROOT).as_posix()}",
        "-e", f"SOFA_CALIBRATION_MODE={mode}",
        "-e", f"SOFA_CALIBRATION_TRACE=/work/{trace_path.relative_to(ROOT).as_posix()}",
        "-v", f"{ROOT}:/work",
        "-v", f"{config['runtime']['sofa_root']}:/sofa:ro",
        config["runtime"]["image"],
        "/sofa/bin/runSofa", "-l", "SofaPython3", "-g", "batch", "-n", str(config["simulation"]["steps"]), scene_path,
    ]
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (work_dir / f"{mode}.log").write_text(completed.stdout)
    if completed.returncode != 0 or not trace_path.exists():
        raise RuntimeError(f"{mode} did not produce a trace; see {work_dir / (mode + '.log')}")
    if "[ERROR]" in completed.stdout or "NaN" in completed.stdout:
        raise RuntimeError(f"{mode} emitted a SOFA error or NaN; see {work_dir / (mode + '.log')}")
    return json.loads(trace_path.read_text())


def _summary(records: list[dict]) -> dict[str, list[float]]:
    contacts = [record for record in records if record["active_contact"]]
    return {
        "reaction_force_proxy": [record["reaction_force_proxy"] for record in records],
        "contact_reaction_force_proxy": [record["reaction_force_proxy"] for record in contacts],
        "contact_tangent_speed_mps": [record["wall_tangent_relative_speed_mps"] for record in contacts],
        "progress_m": [record["task_progress_m"] for record in records],
        "samples": len(records),
        "contact_samples": len(contacts),
        "max_contact_count_proxy": max((record["contact_count_proxy"] for record in records), default=0),
    }


def _trace_metrics(trace: dict) -> dict[str, int]:
    return {
        "samples": int(trace["samples"]),
        "contact_samples": int(trace["contact_samples"]),
        "max_contact_count_proxy": int(trace["max_contact_count_proxy"]),
    }


def _write_report(report_path: Path, metrics: dict) -> None:
    separation = metrics["separation"]
    config = metrics["config"]
    passed = metrics["verdict"] == "PHASE0S_SOFA_ORACLE_CALIBRATION_PASS"
    verdict = metrics["verdict"]
    contact = "PASS" if "contact_reaction_overlap" not in metrics["failures"] else "FAIL"
    friction = "PASS" if "stick_slip_speed_overlap" not in metrics["failures"] else "FAIL"
    jam = "PASS" if "jam_free_progress_overlap" not in metrics["failures"] else "FAIL"
    frozen = "Frozen thresholds written to `configs/oracle_sofa_frozen.yaml`." if passed else "No Oracle threshold file was written."
    next_action = (
        "Run held-out SOFA capability validation on seeds 84001–84003."
        if passed
        else "Review the failed controlled physical separation before any capability or passage experiment."
    )
    report_path.write_text(
        f"""# Phase 0S-SOFA Oracle Calibration

## Verdict

{verdict}

## Runtime

- SOFA: official v26.06.00 Linux Python 3.12 binary.
- BeamAdapter: release-bundled v26.06 plugin.
- SofaPython3: release-bundled v26.06 plugin, Python 3.12.3.
- CPU-only: yes; Docker was run without GPU exposure and `CUDA_VISIBLE_DEVICES` was empty.

## Calibration setup

- seed: 84000 only.
- beam: {config['beam']['length_m']} m length, {config['beam']['radius_m']} m radius, {config['beam']['nodes']} nodes, {config['beam']['young_modulus_pa']} Pa Young modulus, {config['beam']['mass_density_kg_m3']} kg/m^3 density.
- dt: {config['simulation']['dt_s']} s; 300 samples per trace; friction: {config['simulation']['friction']}.
- command: forward force {config['commands']['forward_force_n']} N; normal preload {config['commands']['normal_preload_force_n']} N.
- force selection: one free-pull sanity yielded {metrics['force_sanity_terminal_median_speed_mps']:.12g} m/s; it did not cross the one-time adjustment bound (<0.005 or >0.10 m/s), so the initial force was retained.
- fixture: horizontal static triangle wall for stick/slip; static vertical triangle blocker for jam; no fixture for the two free traces.
- raw channels: timestamp, ordered beam positions/velocities, command force/active, contact-count proxy, wall identity, global LCP constraint-force proxy, wall-relative tangent speed, and tip progress.

## Contact separation

- free reaction p99: {separation['free_force_p99']:.12g}
- contact reaction p10: {separation['contact_force_p10']:.12g}
- reaction threshold: not frozen ({contact})

## Stick/slip separation

- stick tangent speed p95: {separation['stick_speed_p95']:.12g} m/s
- slip tangent speed p05: {separation['slip_speed_p05']:.12g} m/s
- stick threshold: not frozen
- slip threshold: not frozen ({friction})

## Jam separation

- free progress p10 @100ms: {separation['free_progress_p10']:.12g} m
- jam progress p90 @100ms: {separation['jam_progress_p90']:.12g} m
- jam threshold: not frozen ({jam})

## Frozen semantics

- minimum dwell: 80 ms.
- jam window: 100 ms.
- {frozen}

## Fact

All five seed-84000 controlled scenes executed and emitted the required simulator-native trace channels. Free reaction was zero; stick and slip each had 148 contact-proxy samples (maximum two contact triplets), but the global LCP constraint-force proxy had a zero 10th percentile in both. The free-forward and jam traces had no active contact-proxy samples in the retained force/horizon.

## Inference

This one-shot controlled setup does not establish clean free/contact, stick/slip, or free-forward/jam numerical gaps. It is therefore a calibration failure, not evidence that thresholds can be chosen manually.

## Unknown

Whether a differently designed controlled SOFA contact construction can create sustained separable regimes remains untested; no held-out capability or unified-passage result exists.

## Next action

{next_action}
"""
    )


def _write_frozen_oracle(path: Path, config: dict, values: dict) -> None:
    path.write_text(
        f"""source:
  simulator: sofa
  version: v26.06.00
  calibration_seed: 84000
contact:
  reaction_force_floor: {values['reaction_force_floor']:.12g}
friction:
  stick_tangent_speed_max_mps: {values['stick_tangent_speed_max_mps']:.12g}
  slip_tangent_speed_min_mps: {values['slip_tangent_speed_min_mps']:.12g}
jam:
  progress_window_ms: {config['simulation']['jam_progress_window_ms']}
  progress_max_m: {values['jam_progress_max_m']:.12g}
  reaction_force_min: {values['reaction_force_floor']:.12g}
dwell:
  minimum_mode_duration_ms: {config['simulation']['minimum_mode_duration_ms']}
"""
    )


def _select_force(config_path: Path, config: dict, work_dir: Path) -> float:
    initial = float(config["commands"]["initial_forward_force_n"])
    config["commands"]["forward_force_n"] = initial
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    trace = _run_scene(config_path, config, "force_sanity", work_dir)
    speeds = np.asarray([record["wall_tangent_relative_speed_mps"] for record in trace[-100:]], dtype=float)
    speed = float(np.median(speeds))
    selected = initial * (2.0 if speed < 0.005 else 0.5 if speed > 0.10 else 1.0)
    config["commands"]["forward_force_n"] = selected
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    return speed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = _load(config_path)
    if int(config["seed"]) != 84000:
        raise ValueError("Oracle calibration is restricted to seed 84000")
    if os.environ.get("CUDA_VISIBLE_DEVICES") not in {"", None}:
        raise RuntimeError("CPU-only calibration requires CUDA_VISIBLE_DEVICES to be empty")

    report_dir = SPIKE / "reports" / "phase0s_sofa"
    work_dir = report_dir / ".calibration_work"
    shutil.rmtree(work_dir, ignore_errors=True)
    work_dir.mkdir(parents=True)
    sanity_speed = _select_force(config_path, config, work_dir)
    traces = {mode: _summary(_run_scene(config_path, config, mode, work_dir)) for mode in MODES}
    result = calibrate(traces, dt_s=float(config["simulation"]["dt_s"]), window_ms=int(config["simulation"]["jam_progress_window_ms"]))
    metrics = {
        "verdict": "PHASE0S_SOFA_ORACLE_CALIBRATION_PASS" if result.passed else "PHASE0S_SOFA_ORACLE_CALIBRATION_FAIL",
        "seed": 84000,
        "force_sanity_terminal_median_speed_mps": sanity_speed,
        "force_selection_rule": "initial; double once below 0.005 m/s, halve once above 0.10 m/s, otherwise retain",
        "config": config,
        "raw_channels": [
            "timestamp_s", "beam_node_positions_m", "beam_node_velocities_mps", "command_force_n", "command_active",
            "contact_count_proxy", "active_contact", "contact_wall_identity", "reaction_force_proxy",
            "wall_tangent_relative_speed_mps", "task_progress_m",
        ],
        "traces": {mode: _trace_metrics(trace) for mode, trace in traces.items()},
        "separation": result.values,
        "failures": list(result.failures),
    }
    (report_dir / "calibration_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    _write_report(report_dir / "CALIBRATION.md", metrics)
    if result.passed:
        _write_frozen_oracle(SPIKE / "configs" / "oracle_sofa_frozen.yaml", config, result.values)
    shutil.rmtree(work_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
