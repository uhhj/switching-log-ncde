"""Stage-A-only audit for the C++ NativeContactBridge."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
sys.path.insert(0, str(SPIKE / "src"))
from contact_native import unpack_bridge_frame  # noqa: E402


def _library() -> Path:
    return SPIKE / "native-contact-bridge" / "build" / "lib" / "libNativeContactBridge.so"


def _run(config_path: Path, config: dict, trace: Path, log: Path) -> None:
    library = _library()
    if not library.is_file():
        raise RuntimeError(f"native bridge library missing: {library}")
    command = [
        "docker", "run", "--rm", "--network", "none",
        "-e", "CUDA_VISIBLE_DEVICES=", "-e", "LD_LIBRARY_PATH=/sofa/lib",
        "-e", f"SOFA_NATIVE_BRIDGE_CONFIG=/work/{config_path.relative_to(ROOT).as_posix()}",
        "-e", f"SOFA_NATIVE_BRIDGE_TRACE=/work/{trace.relative_to(ROOT).as_posix()}",
        "-v", f"{ROOT}:/work", "-v", f"{config['runtime']['sofa_root']}:/sofa:ro",
        config["runtime"]["image"], "/sofa/bin/runSofa", "-l", "SofaPython3", "-l",
        f"/work/{library.relative_to(ROOT).as_posix()}", "-g", "batch", "-n",
        str(int(config["simulation"]["steps"])),
        "/work/simulator-spikes/sofa-beam-spike/src/native_contact_bridge_smoke.py",
    ]
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log.write_text(completed.stdout)
    # An out-of-tree library is explicitly loaded by absolute path. v26.06
    # subsequently asks PluginManager to locate its module name in the global
    # repository and logs this harmless duplicate-lookup diagnostic. It is not
    # used as a contact signal; all other SOFA errors remain fatal.
    remaining_errors = [line for line in completed.stdout.splitlines() if "[ERROR]" in line and 'Plugin not found: "NativeContactBridge"' not in line]
    if completed.returncode != 0 or remaining_errors or "NaN" in completed.stdout:
        raise RuntimeError(f"runSofa native bridge smoke failed; see {log}")
    if not trace.is_file():
        raise RuntimeError("native bridge trace missing")


def _audit(records: list[dict], expected_steps: int) -> dict:
    failures: list[str] = []
    if len(records) != expected_steps:
        failures.append("trace_length")
    if not records:
        return {"pass": False, "failures": ["empty_trace"]}
    ready = np.asarray([bool(record["ready"]) for record in records], dtype=bool)
    serial = np.asarray([int(record["frame_serial"]) for record in records], dtype=int)
    contact_frames = [record for record in records if int(record["contact_count"]) > 0]
    if not ready.all(): failures.append("bridge_not_ready")
    if serial.max(initial=0) <= 0: failures.append("collision_event_not_captured")
    if not contact_frames: failures.append("no_native_contacts")
    beam_ids: list[int] = []; fixture_ids: list[int] = []; norms: list[float] = []; values: list[float] = []
    for record in contact_frames:
        try:
            frame = unpack_bridge_frame(record)
        except (KeyError, TypeError, ValueError):
            failures.append("bridge_array_shape")
            continue
        if not np.isfinite(frame["beam_points"]).all() or not np.isfinite(frame["fixture_points"]).all(): failures.append("nonfinite_contact_point")
        if not np.isfinite(frame["normals"]).all(): failures.append("nonfinite_normal")
        if not np.isfinite(frame["detection_values"]).all(): failures.append("nonfinite_detection_value")
        if np.any(frame["beam_ids"] < 0) or np.any(frame["beam_ids"] >= int(record["collision_point_count"])): failures.append("beam_primitive_id_range")
        if np.any(frame["fixture_ids"] < 0) or np.any(frame["fixture_ids"] >= int(record["fixture_triangle_count"])): failures.append("fixture_primitive_id_range")
        beam_ids.extend(frame["beam_ids"].tolist()); fixture_ids.extend(frame["fixture_ids"].tolist())
        norms.extend(np.linalg.norm(frame["normals"], axis=1).tolist()); values.extend(frame["detection_values"].tolist())
    if norms and (np.min(norms) < 0.9 or np.max(norms) > 1.1): failures.append("normal_not_unit")
    return {
        "pass": not failures, "failures": sorted(set(failures)), "frames": len(records),
        "ready_frames": int(ready.sum()), "serial_min": int(serial.min()), "serial_max": int(serial.max()),
        "contact_frames": len(contact_frames), "total_native_contacts": len(beam_ids),
        "max_contacts_per_frame": max((int(record["contact_count"]) for record in records), default=0),
        "beam_primitive_id_min": min(beam_ids) if beam_ids else None, "beam_primitive_id_max": max(beam_ids) if beam_ids else None,
        "fixture_primitive_id_min": min(fixture_ids) if fixture_ids else None, "fixture_primitive_id_max": max(fixture_ids) if fixture_ids else None,
        "normal_norm_min": float(np.min(norms)) if norms else None, "normal_norm_median": float(np.median(norms)) if norms else None,
        "normal_norm_max": float(np.max(norms)) if norms else None,
        "detection_value_min": float(np.min(values)) if values else None, "detection_value_max": float(np.max(values)) if values else None,
    }


def _report(path: Path, metrics: dict) -> None:
    audit = metrics.get("audit", {})
    verdict = metrics["verdict"]
    next_action = ("Resume Phase 0S-SOFA Controlled Contact Construction R1 Revised at Stage B: run the anchored-free reaction-contamination control using NativeContactBridge as the formal native contact identity, then stop for review before Stage C if the LCP reaction proxy is contaminated." if verdict.endswith("PASS") else "Repair only the minimal C++ plugin build/load/Data exposure path against official SOFA v26.06.00; do not modify controlled-contact physics or fall back to nearest-geometry proxies.")
    path.write_text(f"""# Phase 0S-SOFA Native Contact Bridge R1-B

## Verdict

{verdict}

## Runtime

- SOFA: official v26.06.00 Linux binary
- BeamAdapter / SofaPython3: release-bundled v26.06
- CPU-only: yes

## Plugin build

- CMake source: `native-contact-bridge/CMakeLists.txt`
- SOFA prefix: `/root/workspace/third_party/SOFA_v26.06.00_Linux`
- library: `{_library()}`
- full SOFA rebuild: no
- SofaPython3 rebuild: no

## Native source

- `NarrowPhaseDetection::getDetectionOutputs()`: yes
- exported: `DetectionOutput::elem`, `id`, `point[2]`, `normal`, and raw `value`
- `DetectionOutput::value` is reported only as a raw detection value.

## Stage-A smoke

- requested / completed steps: 20 / {audit.get('frames', 0)}
- bridge ready frames: {audit.get('ready_frames', 0)}
- frame serial min/max: {audit.get('serial_min')} / {audit.get('serial_max')}
- native contact frames / total / max per frame: {audit.get('contact_frames', 0)} / {audit.get('total_native_contacts', 0)} / {audit.get('max_contacts_per_frame', 0)}
- beam primitive ID range: {audit.get('beam_primitive_id_min')} / {audit.get('beam_primitive_id_max')}
- fixture primitive ID range: {audit.get('fixture_primitive_id_min')} / {audit.get('fixture_primitive_id_max')}
- normal norm min/median/max: {audit.get('normal_norm_min')} / {audit.get('normal_norm_median')} / {audit.get('normal_norm_max')}
- DetectionOutput value min/max: {audit.get('detection_value_min')} / {audit.get('detection_value_max')}
- failures: {audit.get('failures', [])}

## Proxy policy

- nearest-plane formal contact / velocity: no / no
- endpoint formal velocity: no
- LCP vector as formal contact identity: no
- stdout parsed for formal contact: no

## Fact

{metrics['fact']}

## Inference

{metrics['inference']}

## Unknown

{metrics['unknown']}

## Next action

{next_action}
""")


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", required=True, type=Path); args = parser.parse_args()
    config_path = args.config.resolve(); config = json.loads(config_path.read_text())
    report_dir = SPIKE / "reports" / "phase0s_sofa"; report_dir.mkdir(parents=True, exist_ok=True)
    work = report_dir / ".native_bridge_r1b_work"; shutil.rmtree(work, ignore_errors=True); work.mkdir()
    metrics = {"scope": "Stage A only", "cpu_only": True, "plugin": {"path": str(_library()), "source": "NarrowPhaseDetection::getDetectionOutputs", "formal_proxy_fallback": False}, "stages_b_to_f_executed": False, "oracle_fitted": False}
    try:
        trace, log = work / "trace.json", work / "run.log"; _run(config_path, config, trace, log)
        metrics["audit"] = _audit(json.loads(trace.read_text()), int(config["simulation"]["steps"]))
        passed = bool(metrics["audit"]["pass"])
        metrics["verdict"] = "PHASE0S_SOFA_NATIVE_CONTACT_BRIDGE_PASS" if passed else "PHASE0S_SOFA_NATIVE_CONTACT_BRIDGE_ENGINEERING_BLOCKED"
        metrics["fact"] = "The bridge read only the official v26.06 narrow-phase DetectionOutput map and exported ordinary SOFA Data." if passed else "The Stage-A bridge output did not satisfy the required native-data integrity gate."
        metrics["inference"] = "The native-contact observability path is validated; controlled contact mechanics remain untested." if passed else "This is an engineering signal-path result, not a physical-regime result."
        metrics["unknown"] = "Stages B-F, Oracle calibration, held-out capability, and unified passage remain untested."
    except RuntimeError as error:
        metrics.update({"verdict": "PHASE0S_SOFA_NATIVE_CONTACT_BRIDGE_ENGINEERING_BLOCKED", "audit": {"pass": False, "failures": [str(error)]}, "fact": str(error), "inference": "The Stage-A native-data path is not validated.", "unknown": "Stages B-F, Oracle calibration, held-out capability, and unified passage remain untested."})
    (report_dir / "native_contact_bridge_r1b_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    _report(report_dir / "NATIVE_CONTACT_BRIDGE_R1B.md", metrics)
    shutil.rmtree(work, ignore_errors=True)
    return 0 if metrics["verdict"].endswith("PASS") else 2


if __name__ == "__main__":
    raise SystemExit(main())
