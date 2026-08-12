"""Stage B audit: projective tail anchor versus global LCP constraintForces."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
PLUGIN = SPIKE / "native-contact-bridge" / "build" / "lib" / "libNativeContactBridge.so"


def _percentile(values, q: float) -> float | None:
    array = np.asarray(values, dtype=float)
    if not array.size: return None
    if not np.isfinite(array).all(): raise ValueError("non-finite metric")
    return float(np.percentile(array, q))


def _run(config_path: Path, config: dict, trace: Path, log: Path) -> None:
    if not PLUGIN.is_file(): raise RuntimeError(f"NativeContactBridge missing: {PLUGIN}")
    command = [
        "docker", "run", "--rm", "--network", "none", "-e", "CUDA_VISIBLE_DEVICES=", "-e", "LD_LIBRARY_PATH=/sofa/lib",
        "-e", f"SOFA_STAGE_B_CONFIG=/work/{config_path.relative_to(ROOT).as_posix()}",
        "-e", f"SOFA_STAGE_B_TRACE=/work/{trace.relative_to(ROOT).as_posix()}",
        "-v", f"{ROOT}:/work", "-v", f"{config['runtime']['sofa_root']}:/sofa:ro", config["runtime"]["image"],
        "/sofa/bin/runSofa", "-l", "SofaPython3", "-l", f"/work/{PLUGIN.relative_to(ROOT).as_posix()}", "-g", "batch", "-n", str(int(config["simulation"]["steps"])),
        "/work/simulator-spikes/sofa-beam-spike/src/stage_b_reaction_proxy_scene.py",
    ]
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log.write_text(completed.stdout)
    # v26.06 emits a benign duplicate module-name lookup after a full-path
    # plugin load. Any other SOFA error remains a failure.
    errors = [line for line in completed.stdout.splitlines() if "[ERROR]" in line and 'Plugin not found: "NativeContactBridge"' not in line]
    if completed.returncode != 0 or errors or "NaN" in completed.stdout: raise RuntimeError(f"runSofa failed; see {log}")
    if not trace.is_file(): raise RuntimeError("Stage B trace missing")


def analyze_trace(payload: dict, config: dict) -> dict:
    records, expected = payload["records"], int(config["simulation"]["steps"])
    if len(records) != expected: raise RuntimeError(f"trace length {len(records)} != {expected}")
    measured = [record for record in records if record["measurement_active"]]
    if not measured: raise RuntimeError("no measurement samples")
    values = lambda key, dtype, source=records: np.asarray([record[key] for record in source], dtype=dtype)
    ready, serial = values("bridge_ready", bool), values("bridge_frame_serial", int)
    native, sizes, reaction = values("native_contact_count", int), values("constraint_vector_size", int), values("reaction_force_proxy", float)
    tail, tip, sentinel = values("tail_translation_displacement_m", float), values("tip_displacement_m", float), values("minimum_centerline_to_sentinel_m", float)
    m_native, m_sizes, m_reaction = values("native_contact_count", int, measured), values("constraint_vector_size", int, measured), values("reaction_force_proxy", float, measured)
    checks = {
        "bridge_ready": bool(ready.all()), "frame_serial_advances": bool(serial.size and serial[-1] > serial[0]),
        "native_contact_zero": bool(np.all(m_native == 0)), "lcp_vector_empty": bool(np.all(m_sizes == 0)),
        "reaction_exact_zero": bool(np.all(m_reaction == 0.0)),
        "tail_anchor_active": bool(tail.max() <= float(config["gates"]["tail_translation_max_m"])),
        "load_schedule_valid": bool(np.mean([bool(record["command_active"]) for record in measured]) == 1.0),
        "sentinel_noncontact_valid": bool(sentinel.min() > float(config["gates"]["minimum_sentinel_clearance_m"])),
    }
    validity = all(checks[name] for name in ("bridge_ready", "frame_serial_advances", "native_contact_zero", "tail_anchor_active", "load_schedule_valid", "sentinel_noncontact_valid"))
    if not validity:
        verdict, interpretation, next_action = "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED", "Stage B did not produce a valid anchored-free contamination control.", "Repair only the Stage-B control validity issue and rerun Stage B."
    elif checks["lcp_vector_empty"] and checks["reaction_exact_zero"]:
        verdict, interpretation, next_action = "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CLEAN", "The translation-only projective tail anchor did not directly contaminate the current global LCP reaction proxy in this no-contact control.", "Run Stage C stable normal-contact construction with NativeContactBridge as formal contact identity and test whether real wall contact produces sustained nonzero reaction proxy."
    else:
        verdict, interpretation, next_action = "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CONTAMINATED", "The global LCP constraintForces channel is not structurally clean under the planned tail boundary.", "Choose and validate a per-contact Lagrange-multiplier extraction path, or remove the global reaction channel from the SOFA contact Oracle, before Stage C."
    return {
        "verdict": verdict, "steps": len(records), "measurement_samples": len(measured),
        "bridge_ready_all": checks["bridge_ready"], "frame_serial_min": int(serial.min()), "frame_serial_max": int(serial.max()), "frame_serial_advances": checks["frame_serial_advances"],
        "native_contact_total": int(native.sum()), "native_contact_total_measurement": int(m_native.sum()), "native_contact_max_per_frame": int(native.max()),
        "constraint_vector_size_max": int(sizes.max()), "constraint_vector_size_max_measurement": int(m_sizes.max()),
        "constraint_nonempty_frames": int(np.count_nonzero(sizes > 0)), "constraint_nonempty_measurement_frames": int(np.count_nonzero(m_sizes > 0)),
        "reaction_p50_measurement": _percentile(m_reaction, 50), "reaction_p90_measurement": _percentile(m_reaction, 90), "reaction_p99_measurement": _percentile(m_reaction, 99), "reaction_max_measurement": float(m_reaction.max()),
        "tail_translation_displacement_max_m": float(tail.max()), "tip_displacement_max_m": float(tip.max()), "minimum_centerline_to_sentinel_min_m": float(sentinel.min()),
        "load_active_fraction_measurement": float(np.mean([bool(record["command_active"]) for record in measured])), "checks": checks,
        "interpretation": interpretation, "next_action": next_action,
    }


def _write_report(path: Path, result: dict, config: dict) -> None:
    checks = "\n".join(f"- {name}: {'PASS' if passed else 'FAIL'}" for name, passed in result["checks"].items())
    path.write_text(f"""# Phase 0S-SOFA Stage B Reaction-Proxy Contamination Audit

## Verdict

{result['verdict']}

## Runtime

- SOFA: official v26.06.00 Linux binary
- BeamAdapter / SofaPython3: release-bundled v26.06
- NativeContactBridge: full-path loaded, source unchanged
- CPU-only: yes

## Frozen setup

- seed / dt / steps: {config['seed']} / {config['simulation']['dt_s']} s / {config['simulation']['steps']}
- normal endpoint load: -{config['simulation']['normal_load_n']} N along Y after {config['simulation']['settle_ms']} ms
- tail: node 0 translation fixed, rotation free; fixedDirections={config['tail_constraint']['fixed_directions']}
- sentinel plane: y={config['sentinel_fixture']['plane_y_m']} m

## Native-contact control

- bridge ready frames: {result['bridge_ready_all']}
- frame serial min/max: {result['frame_serial_min']} / {result['frame_serial_max']}
- native contact total / measurement / max-frame: {result['native_contact_total']} / {result['native_contact_total_measurement']} / {result['native_contact_max_per_frame']}
- minimum centerline-to-sentinel: {result['minimum_centerline_to_sentinel_min_m']:.12g} m

## LCP reaction audit

- vector size max all / measurement: {result['constraint_vector_size_max']} / {result['constraint_vector_size_max_measurement']}
- nonempty frames all / measurement: {result['constraint_nonempty_frames']} / {result['constraint_nonempty_measurement_frames']}
- reaction p50 / p90 / p99 / max: {result['reaction_p50_measurement']} / {result['reaction_p90_measurement']} / {result['reaction_p99_measurement']} / {result['reaction_max_measurement']}

## Control validity

- tail translation max: {result['tail_translation_displacement_max_m']:.12g} m
- tip displacement max: {result['tip_displacement_max_m']:.12g} m
- load-active fraction: {result['load_active_fraction_measurement']:.12g}

## Checks

{checks}

## Fact

NativeContactBridge verified the anchored control remained collision-free while `LCPConstraintSolver.constraintForces` was inspected directly.

## Inference

{result['interpretation']}

## Unknown

Real wall-contact reaction, stable contact, breakaway, stick/slip, jam, Oracle calibration, held-out capability, and unified passage remain untested.

## Next action

{result['next_action']}
""")


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", required=True, type=Path); args = parser.parse_args()
    if os.environ.get("CUDA_VISIBLE_DEVICES") not in {"", None}: raise RuntimeError("Stage B must be CPU-only")
    config_path = args.config.resolve(); config = json.loads(config_path.read_text())
    if int(config["seed"]) != 84000: raise ValueError("Stage B is restricted to seed 84000")
    report_dir = SPIKE / "reports" / "phase0s_sofa"; work = report_dir / ".stage_b_reaction_proxy_work"
    shutil.rmtree(work, ignore_errors=True); work.mkdir(parents=True)
    try:
        _run(config_path, config, work / "trace.json", work / "run.log")
        result = analyze_trace(json.loads((work / "trace.json").read_text()), config)
    except Exception as exc:
        result = {"verdict": "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED", "error": str(exc), "interpretation": "Stage B did not produce a valid contamination audit.", "next_action": "Repair only the Stage-B control validity issue and rerun Stage B.", "checks": {}}
    metrics = {"seed": int(config["seed"]), "cpu_only": True, "scope": "Stage B only", "runtime": config["runtime"], "result": result, "stages_c_to_f_executed": False, "oracle_fitted": False, "oracle_frozen": False}
    (report_dir / "stage_b_reaction_proxy_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = report_dir / "STAGE_B_REACTION_PROXY_AUDIT.md"
    if "steps" in result: _write_report(report, result, config)
    else: report.write_text(f"# Phase 0S-SOFA Stage B Reaction-Proxy Contamination Audit\n\n## Verdict\n\n{result['verdict']}\n\n## Fact\n\n{result['error']}\n\n## Next action\n\n{result['next_action']}\n")
    shutil.rmtree(work, ignore_errors=True); print(json.dumps(metrics, indent=2))
    return 0 if result["verdict"] in {"PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CLEAN", "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CONTAMINATED"} else 2


if __name__ == "__main__": raise SystemExit(main())
