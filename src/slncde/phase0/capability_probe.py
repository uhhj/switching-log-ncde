from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import numpy as np

from slncde.phase0b.labels import derive_labels, longest_true_run_samples, sustained_mask
from slncde.sim.config import load_config


PROBES = ("free_control", "stick_probe", "slip_probe", "jam_probe")
BEAD_HALF_WIDTH_M = 0.005
BEAD_MASS_KG = 0.1
WALL_HALF_THICKNESS_M = 0.005


def aggregate_contact_points(
    points: Sequence[Sequence[Any]], velocity_xyz: Sequence[float]
) -> Dict[str, Any]:
    """Aggregate PyBullet contacts with the same tuple semantics as T2."""
    velocity = np.asarray(velocity_xyz, dtype=np.float64)
    normal_forces = []
    friction_forces = []
    tangent_speeds = []
    normals = []
    for point in points:
        normal = np.asarray(point[7], dtype=np.float64)
        normal_force = float(point[9])
        friction = np.zeros(3, dtype=np.float64)
        if len(point) >= 14:
            friction += float(point[10]) * np.asarray(point[11], dtype=np.float64)
            friction += float(point[12]) * np.asarray(point[13], dtype=np.float64)
        tangent_velocity = velocity - np.dot(velocity, normal) * normal
        normals.append(normal)
        normal_forces.append(normal_force)
        friction_forces.append(float(np.linalg.norm(friction)))
        tangent_speeds.append(float(np.linalg.norm(tangent_velocity)))

    if normals:
        contact_normal = np.mean(np.asarray(normals), axis=0)
        norm = float(np.linalg.norm(contact_normal))
        if norm > 0.0:
            contact_normal = contact_normal / norm
    else:
        contact_normal = np.zeros(3, dtype=np.float64)
    return {
        "contact_count": int(len(points)),
        "normal_force_sum": float(np.sum(normal_forces)) if normal_forces else 0.0,
        "normal_force_max": float(np.max(normal_forces)) if normal_forces else 0.0,
        "friction_force_sum": float(np.sum(friction_forces)) if friction_forces else 0.0,
        "tangential_speed": float(np.mean(tangent_speeds)) if tangent_speeds else 0.0,
        "contact_normal": contact_normal,
    }


def aggregate_contact(physics: Any, body_a: int, body_b: int) -> Dict[str, Any]:
    velocity = physics.getBaseVelocity(int(body_a))[0]
    points = physics.getContactPoints(bodyA=int(body_a), bodyB=int(body_b))
    return aggregate_contact_points(points, velocity)


def progress_over_window(progress_m: Sequence[float], window_samples: int) -> np.ndarray:
    """Match the frozen T2 jam progress-window calculation exactly."""
    progress = np.asarray(progress_m, dtype=np.float64)
    window = int(window_samples)
    if window <= 0:
        raise ValueError("window_samples must be positive")
    delta = np.zeros(progress.shape, dtype=np.float64)
    if progress.size > window:
        delta[window:] = progress[window:] - progress[:-window]
    return delta


def capability_verdict(matrix: Mapping[str, Mapping[str, bool]]) -> str:
    if all(bool(matrix[name][layer]) for name in ("stick", "slip", "jam") for layer in ("raw", "label")):
        return "PHASE0C0_LABELS_CAPABLE_TASK_REPRESENTATION_SUSPECT"
    if (
        all(bool(matrix[name][layer]) for name in ("stick", "slip") for layer in ("raw", "label"))
        and bool(matrix["jam"]["raw"])
        and not bool(matrix["jam"]["label"])
    ):
        return "PHASE0C0_JAM_DEFINITION_FAIL"
    if any(bool(matrix[name]["raw"]) and not bool(matrix[name]["label"]) for name in ("stick", "slip", "jam")):
        return "PHASE0C0_ORACLE_DEFINITION_FAIL"
    if not bool(matrix["stick"]["raw"]) or not bool(matrix["slip"]["raw"]):
        return "PHASE0C0_SIM_CONTACT_CAPABILITY_FAIL"
    return "PHASE0C0_SIM_CONTACT_CAPABILITY_FAIL"


def _import_pybullet() -> Any:
    try:
        import pybullet as physics
    except ImportError as exc:  # pragma: no cover - exercised by runtime preflight
        raise RuntimeError("PyBullet is required for the capability probe") from exc
    return physics


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _load_label_source(config: Mapping[str, Any], repo_root: Path) -> Tuple[Dict[str, Any], Path]:
    source_path = repo_root / str(config["labels"]["source_config"])
    source = load_config(source_path)
    label_config = {
        "labels": dict(source["labels"]),
        "simulator": {
            "hz": int(config["simulation"]["hz"]),
            "trace_stride": 1,
        },
    }
    return label_config, source_path


def _create_world(physics: Any, config: Mapping[str, Any], probe: str) -> Dict[str, int]:
    physics.resetSimulation()
    hz = int(config["simulation"]["hz"])
    gravity = 0.0 if probe == "free_control" else float(config["simulation"]["gravity_mps2"])
    physics.setTimeStep(1.0 / hz)
    physics.setGravity(0.0, 0.0, gravity)
    physics.setPhysicsEngineParameter(deterministicOverlappingPairs=1)

    plane_shape = physics.createCollisionShape(physics.GEOM_PLANE)
    plane_id = physics.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=plane_shape)
    physics.changeDynamics(
        plane_id,
        -1,
        lateralFriction=float(config["probe"]["lateral_friction"]),
    )

    wall_id = -1
    if probe in ("jam_probe", "preflight"):
        wall_shape = physics.createCollisionShape(
            physics.GEOM_BOX, halfExtents=[WALL_HALF_THICKNESS_M, 0.05, 0.05]
        )
        wall_id = physics.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=wall_shape,
            basePosition=[float(config["probe"]["wall_x_m"]), 0.0, 0.05],
        )
        physics.changeDynamics(
            wall_id,
            -1,
            lateralFriction=float(config["probe"]["wall_lateral_friction"]),
        )

    bead_shape = physics.createCollisionShape(
        physics.GEOM_BOX, halfExtents=[BEAD_HALF_WIDTH_M] * 3
    )
    if probe == "free_control":
        bead_position = [0.0, 0.0, 0.05]
    elif probe == "preflight":
        bead_position = [
            float(config["probe"]["wall_x_m"]) - WALL_HALF_THICKNESS_M - BEAD_HALF_WIDTH_M,
            0.0,
            BEAD_HALF_WIDTH_M,
        ]
    elif probe == "jam_probe":
        bead_position = [
            float(config["probe"]["wall_x_m"])
            - WALL_HALF_THICKNESS_M
            - BEAD_HALF_WIDTH_M
            - float(config["probe"]["initial_wall_clearance_m"]),
            0.0,
            BEAD_HALF_WIDTH_M,
        ]
    else:
        bead_position = [0.0, 0.0, BEAD_HALF_WIDTH_M]
    bead_id = physics.createMultiBody(
        baseMass=BEAD_MASS_KG,
        baseCollisionShapeIndex=bead_shape,
        basePosition=bead_position,
    )
    physics.changeDynamics(
        bead_id,
        -1,
        lateralFriction=float(config["probe"]["lateral_friction"]),
    )
    return {"plane": int(plane_id), "wall": int(wall_id), "bead": int(bead_id)}


def _force_for_probe(config: Mapping[str, Any], probe: str) -> float:
    return {
        "free_control": 0.0,
        "stick_probe": float(config["probe"]["stick_force_n"]),
        "slip_probe": float(config["probe"]["slip_force_n"]),
        "jam_probe": float(config["probe"]["jam_push_force_n"]),
    }[probe]


def _run_episode(
    physics: Any,
    config: Mapping[str, Any],
    label_config: Mapping[str, Any],
    probe: str,
    repeat: int,
) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    ids = _create_world(physics, config, probe)
    hz = int(config["simulation"]["hz"])
    settle_steps = int(round(float(config["simulation"]["settle_s"]) * hz))
    for _ in range(settle_steps):
        physics.stepSimulation()

    origin_x = float(physics.getBasePositionAndOrientation(ids["bead"])[0][0])
    duration_s = 0.5 if probe == "free_control" else float(config["simulation"]["duration_s"])
    steps = int(round(duration_s * hz))
    force_n = _force_for_probe(config, probe)
    records: Dict[str, list] = {
        key: []
        for key in (
            "time_s", "physics_step", "position_xyz", "velocity_xyz",
            "command_force_xyz", "command_active", "floor_contact_count",
            "floor_normal_force", "floor_normal_force_max", "floor_tangent_speed",
            "floor_friction_force", "wall_contact_count", "wall_normal_force",
            "wall_normal_force_max", "wall_tangent_speed", "wall_friction_force",
            "contact_count", "normal_force_sum", "normal_force_max",
            "friction_force_sum", "tangential_speed", "progress_m",
        )
    }

    for step in range(steps):
        if force_n > 0.0:
            position = physics.getBasePositionAndOrientation(ids["bead"])[0]
            physics.applyExternalForce(
                ids["bead"], -1, [force_n, 0.0, 0.0], position, physics.WORLD_FRAME
            )
        physics.stepSimulation()
        position = np.asarray(physics.getBasePositionAndOrientation(ids["bead"])[0], dtype=np.float64)
        velocity = np.asarray(physics.getBaseVelocity(ids["bead"])[0], dtype=np.float64)
        floor = aggregate_contact(physics, ids["bead"], ids["plane"])
        wall = (
            aggregate_contact(physics, ids["bead"], ids["wall"])
            if ids["wall"] >= 0
            else aggregate_contact_points([], velocity)
        )
        selected = wall if probe == "jam_probe" else floor
        records["time_s"].append((step + 1) / hz)
        records["physics_step"].append(step + 1)
        records["position_xyz"].append(position)
        records["velocity_xyz"].append(velocity)
        records["command_force_xyz"].append([force_n, 0.0, 0.0])
        records["command_active"].append(force_n > 0.0)
        records["floor_contact_count"].append(floor["contact_count"])
        records["floor_normal_force"].append(floor["normal_force_sum"])
        records["floor_normal_force_max"].append(floor["normal_force_max"])
        records["floor_tangent_speed"].append(floor["tangential_speed"])
        records["floor_friction_force"].append(floor["friction_force_sum"])
        records["wall_contact_count"].append(wall["contact_count"])
        records["wall_normal_force"].append(wall["normal_force_sum"])
        records["wall_normal_force_max"].append(wall["normal_force_max"])
        records["wall_tangent_speed"].append(wall["tangential_speed"])
        records["wall_friction_force"].append(wall["friction_force_sum"])
        records["contact_count"].append(selected["contact_count"])
        records["normal_force_sum"].append(selected["normal_force_sum"])
        records["normal_force_max"].append(selected["normal_force_max"])
        records["friction_force_sum"].append(selected["friction_force_sum"])
        records["tangential_speed"].append(selected["tangential_speed"])
        records["progress_m"].append(float(position[0]) - origin_x)

    trace = {key: np.asarray(value) for key, value in records.items()}
    command_vector = np.zeros((steps, 3), dtype=np.float64)
    if force_n > 0.0:
        command_vector[:, 0] = 1.0
    label_trace = {
        "contact_active_beads": (trace["contact_count"] > 0).astype(np.int64),
        "normal_force_sum": trace["normal_force_sum"],
        "tangential_speed_mean": trace["tangential_speed"],
        "tangential_force_sum": trace["friction_force_sum"],
        "command_velocity_xyz": command_vector,
        "progress_m": trace["progress_m"],
        "phase": np.full(steps, "free" if probe == "free_control" else "insertion", dtype="U16"),
    }
    derived = derive_labels(label_trace, label_config)
    minimum_samples = int(np.asarray(derived["minimum_mode_samples"]).item())
    stick_sustained = sustained_mask(derived["stick"], minimum_samples)
    slip_sustained = sustained_mask(derived["slip"], minimum_samples)
    trace.update(
        {
            "contact_layer": derived["contact_state"],
            "friction_layer": derived["friction_regime"],
            "failure_layer": derived["jam_state"],
            "touch_event": derived["is_touch_event"],
            "release_event": derived["is_release_event"],
            "jam_onset_event": derived["is_jam_onset_event"],
            "jam_release_event": derived["is_jam_release_event"],
        }
    )

    labels = label_config["labels"]
    raw_contact = trace["contact_count"] > 0
    contact_dwell_ms = 1000.0 * longest_true_run_samples(raw_contact) / hz
    label_masks = {"stick_probe": stick_sustained, "slip_probe": slip_sustained, "jam_probe": derived["jam"]}
    final_window = min(steps - 1, int(round(0.3 * hz)))
    displacement_final_300 = abs(float(trace["position_xyz"][-1, 0] - trace["position_xyz"][-1 - final_window, 0]))
    displacement_driven = float(trace["position_xyz"][-1, 0] - trace["position_xyz"][0, 0])
    contact_tangent = trace["tangential_speed"][raw_contact]
    contact_friction = trace["friction_force_sum"][raw_contact]
    median_tangent = float(np.median(contact_tangent)) if contact_tangent.size else 0.0
    median_friction = float(np.median(contact_friction)) if contact_friction.size else 0.0
    median_normal = float(np.median(trace["normal_force_sum"][raw_contact])) if np.any(raw_contact) else 0.0
    window_samples = int(np.asarray(derived["jam_window_samples"]).item())
    progress_delta = progress_over_window(trace["progress_m"], window_samples)
    eligible_wall = raw_contact & (np.arange(steps) >= window_samples)
    median_jam_progress = float(np.median(progress_delta[eligible_wall])) if np.any(eligible_wall) else float("inf")

    if probe == "free_control":
        raw_pass = bool(np.mean(trace["contact_count"] == 0) >= 0.90)
        label_pass = bool(np.mean(derived["free"]) >= 0.90)
        label_dwell_ms = 1000.0 * longest_true_run_samples(derived["free"]) / hz
    elif probe == "stick_probe":
        raw_pass = bool(
            contact_dwell_ms >= 200.0
            and median_tangent <= float(labels["stick_tangent_speed_max_mps"])
            and displacement_final_300 <= 0.001
            and median_friction > 1e-9
        )
        label_dwell_ms = 1000.0 * longest_true_run_samples(stick_sustained) / hz
        label_pass = bool(label_dwell_ms >= float(labels["minimum_mode_duration_ms"]))
    elif probe == "slip_probe":
        raw_pass = bool(
            contact_dwell_ms >= 200.0
            and median_tangent >= float(labels["slip_tangent_speed_min_mps"])
            and displacement_driven >= 0.005
        )
        label_dwell_ms = 1000.0 * longest_true_run_samples(slip_sustained) / hz
        label_pass = bool(label_dwell_ms >= float(labels["minimum_mode_duration_ms"]))
    else:
        raw_pass = bool(
            contact_dwell_ms >= 200.0
            and median_normal >= float(labels["jam_normal_force_min_n"])
            and bool(np.all(trace["command_active"]))
            and median_jam_progress <= float(labels["jam_progress_max_m"])
        )
        label_dwell_ms = 1000.0 * longest_true_run_samples(derived["jam"]) / hz
        label_pass = bool(label_dwell_ms >= float(labels["minimum_mode_duration_ms"]))

    episode = {
        "probe": probe,
        "repeat": int(repeat),
        "raw_pass": raw_pass,
        "label_pass": label_pass,
        "contact_dwell_ms": contact_dwell_ms,
        "label_dwell_ms": label_dwell_ms,
        "median_tangential_speed_mps": median_tangent,
        "displacement_final_300ms_m": displacement_final_300,
        "displacement_driven_m": displacement_driven,
        "median_friction_force_n": median_friction,
        "median_normal_force_n": median_normal,
        "median_jam_window_progress_m": median_jam_progress,
        "free_raw_fraction": float(np.mean(trace["contact_count"] == 0)),
        "free_label_fraction": float(np.mean(derived["free"])),
    }
    return trace, episode


def _median(episodes: Sequence[Mapping[str, Any]], key: str) -> float:
    return float(np.median([float(item[key]) for item in episodes]))


def _aggregate_metrics(
    config: Mapping[str, Any], label_config: Mapping[str, Any], episodes: Sequence[Mapping[str, Any]]
) -> Dict[str, Any]:
    grouped = {probe: [item for item in episodes if item["probe"] == probe] for probe in PROBES}
    free = grouped["free_control"]
    results: Dict[str, Any] = {
        "free": {
            "raw_pass_count": sum(bool(item["raw_pass"]) for item in free),
            "label_pass_count": sum(bool(item["label_pass"]) for item in free),
            "raw_pass": sum(bool(item["raw_pass"]) for item in free) >= 2,
            "label_pass": sum(bool(item["label_pass"]) for item in free) >= 2,
        }
    }
    for name, probe in (("stick", "stick_probe"), ("slip", "slip_probe"), ("jam", "jam_probe")):
        items = grouped[probe]
        results[name] = {
            "raw": {
                "pass_count": sum(bool(item["raw_pass"]) for item in items),
                "pass": sum(bool(item["raw_pass"]) for item in items) >= 2,
            },
            "label": {
                "pass_count": sum(bool(item["label_pass"]) for item in items),
                "pass": sum(bool(item["label_pass"]) for item in items) >= 2,
            },
            "median_contact_dwell_ms": _median(items, "contact_dwell_ms"),
            "median_label_dwell_ms": _median(items, "label_dwell_ms"),
            "median_tangential_speed_mps": _median(items, "median_tangential_speed_mps"),
            "median_displacement_final_300ms_m": _median(items, "displacement_final_300ms_m"),
            "median_displacement_driven_m": _median(items, "displacement_driven_m"),
            "median_friction_force_n": _median(items, "median_friction_force_n"),
            "median_normal_force_n": _median(items, "median_normal_force_n"),
            "median_jam_window_progress_m": _median(items, "median_jam_window_progress_m"),
        }
    matrix = {
        name: {"raw": bool(results[name]["raw"]["pass"]), "label": bool(results[name]["label"]["pass"])}
        for name in ("stick", "slip", "jam")
    }
    results["capability_matrix"] = matrix
    results["verdict"] = capability_verdict(matrix)
    results["setup"] = {
        "hz": int(config["simulation"]["hz"]),
        "bead_mass_kg": BEAD_MASS_KG,
        "bead_collision_half_width_m": BEAD_HALF_WIDTH_M,
        "lateral_friction": float(config["probe"]["lateral_friction"]),
        "label_thresholds": dict(label_config["labels"]),
        "cpu_only": True,
    }
    results["episodes"] = list(episodes)
    return results


def _scientific_text(verdict: str) -> Tuple[str, str, str]:
    if verdict == "PHASE0C0_LABELS_CAPABLE_TASK_REPRESENTATION_SUSPECT":
        return (
            "The minimal PyBullet scenes formed stick, slip, and jam signals, and the frozen T2 Oracle recognized all three.",
            "The missing T2 regimes are therefore attributable primarily to the current cable/fixture/controller realization, not basic contact representation or label thresholds.",
            "Stop modifying DeformableRavens T2 and evaluate a simulator/representation better suited to contact-rich DLO dynamics.",
        )
    if verdict == "PHASE0C0_JAM_DEFINITION_FAIL":
        return (
            "Stick and slip passed in raw physics and labels; raw jam passed but the jam label failed.",
            "Friction contact representation is adequate, while the jam progress/force/dwell Oracle semantic is not.",
            "Revise only the jam Oracle semantic and rerun this same capability probe.",
        )
    if verdict == "PHASE0C0_ORACLE_DEFINITION_FAIL":
        return (
            "At least one raw physical regime formed but its frozen T2 label did not recognize it.",
            "The corresponding Oracle extraction, threshold, or dwell definition is the limiting layer.",
            "Revise only the failed Oracle definition and rerun this same capability probe.",
        )
    return (
        "At least one critical raw friction regime did not form in the minimal bead contact scene.",
        "The current PyBullet bead contact setup does not reliably provide the physical regimes required by the scientific task.",
        "Stop the DeformableRavens primary scientific simulator route and evaluate an alternative DLO simulator.",
    )


def _write_report(report_root: Path, metrics: Mapping[str, Any], pybullet_version: str) -> None:
    fact, inference, next_action = _scientific_text(str(metrics["verdict"]))
    setup = metrics["setup"]
    labels = setup["label_thresholds"]
    stick, slip, jam = metrics["stick"], metrics["slip"], metrics["jam"]
    yesno = lambda value: "PASS" if value else "FAIL"
    report = f"""# Phase 0C0 Contact-Regime Capability Probe

## Verdict
{metrics['verdict']}

## Setup
- PyBullet: {pybullet_version}
- hz: {setup['hz']}
- bead mass: {setup['bead_mass_kg']} kg
- bead collision half-width: {setup['bead_collision_half_width_m']} m
- friction: {setup['lateral_friction']}
- current T2 label thresholds: contact {labels['contact_normal_force_min_n']} N; stick {labels['stick_tangent_speed_max_mps']} m/s; slip {labels['slip_tangent_speed_min_mps']} m/s; command {labels['command_active_speed_min_mps']} m/s; jam {labels['jam_normal_force_min_n']} N, {labels['jam_window_ms']} ms, {labels['jam_progress_max_m']} m; dwell {labels['minimum_mode_duration_ms']} ms
- CPU-only: yes

## Free control
- raw free: {metrics['free']['raw_pass_count']}/3
- label free: {metrics['free']['label_pass_count']}/3

## Stick
- raw capability: {stick['raw']['pass_count']}/3
- label capability: {stick['label']['pass_count']}/3
- median contact dwell: {stick['median_contact_dwell_ms']:.3f} ms
- median tangential speed: {stick['median_tangential_speed_mps']:.9f} m/s
- median displacement: {stick['median_displacement_final_300ms_m']:.9f} m (final 300 ms)
- median friction force: {stick['median_friction_force_n']:.9f} N

## Slip
- raw capability: {slip['raw']['pass_count']}/3
- label capability: {slip['label']['pass_count']}/3
- median contact dwell: {slip['median_contact_dwell_ms']:.3f} ms
- median tangential speed: {slip['median_tangential_speed_mps']:.9f} m/s
- median displacement: {slip['median_displacement_driven_m']:.9f} m
- median friction force: {slip['median_friction_force_n']:.9f} N

## Jam
- raw capability: {jam['raw']['pass_count']}/3
- label capability: {jam['label']['pass_count']}/3
- median wall contact dwell: {jam['median_contact_dwell_ms']:.3f} ms
- median wall normal force: {jam['median_normal_force_n']:.9f} N
- median jam-window progress: {jam['median_jam_window_progress_m']:.9f} m

## Capability matrix
| Regime | Raw physics | Current label |
|---|---|---|
| stick | {yesno(stick['raw']['pass'])} | {yesno(stick['label']['pass'])} |
| slip | {yesno(slip['raw']['pass'])} | {yesno(slip['label']['pass'])} |
| jam | {yesno(jam['raw']['pass'])} | {yesno(jam['label']['pass'])} |

## Fact
{fact}

## Inference
{inference}

## Scientific interpretation
Raw physics capability and current Oracle-label capability were evaluated separately in single-bead scenes without a robot, cable constraints, or the T2 fixture.

## Next action
{next_action}
"""
    report_root.mkdir(parents=True, exist_ok=True)
    (report_root / "RESULT.md").write_text(report, encoding="utf-8")
    with (report_root / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")


def run_preflight(config_path: Path) -> Dict[str, Any]:
    config_path = Path(config_path).resolve()
    repo_root = _repo_root()
    config = load_config(config_path)
    label_config, source_path = _load_label_source(config, repo_root)
    physics = _import_pybullet()
    client = physics.connect(physics.DIRECT)
    try:
        ids = _create_world(physics, config, "preflight")
        for _ in range(int(round(float(config["simulation"]["settle_s"]) * int(config["simulation"]["hz"])))):
            position = physics.getBasePositionAndOrientation(ids["bead"])[0]
            physics.applyExternalForce(ids["bead"], -1, [float(config["probe"]["jam_push_force_n"]), 0.0, 0.0], position, physics.WORLD_FRAME)
            physics.stepSimulation()
        floor = aggregate_contact(physics, ids["bead"], ids["plane"])
        wall = aggregate_contact(physics, ids["bead"], ids["wall"])
        result = {
            "status": "PASS" if floor["contact_count"] > 0 and wall["contact_count"] > 0 else "FAIL",
            "floor_contact_count": floor["contact_count"],
            "wall_contact_count": wall["contact_count"],
            "bead_mass_kg": BEAD_MASS_KG,
            "bead_collision_half_width_m": BEAD_HALF_WIDTH_M,
            "friction": float(config["probe"]["lateral_friction"]),
            "label_source": str(source_path.relative_to(repo_root)),
            "label_thresholds": label_config["labels"],
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return result
    finally:
        physics.disconnect(client)


def run_capability_probe(config_path: Path) -> Dict[str, Any]:
    config_path = Path(config_path).resolve()
    repo_root = _repo_root()
    config = load_config(config_path)
    label_config, _ = _load_label_source(config, repo_root)
    physics = _import_pybullet()
    client = physics.connect(physics.DIRECT)
    report_root = repo_root / str(config["outputs"]["report_root"])
    data_root = report_root / "data"
    data_root.mkdir(parents=True, exist_ok=True)
    episodes = []
    try:
        for probe in PROBES:
            for repeat in range(1, int(config["experiment"]["repeats"]) + 1):
                trace, episode = _run_episode(physics, config, label_config, probe, repeat)
                np.savez_compressed(data_root / f"{probe}_repeat_{repeat}.npz", **trace)
                episodes.append(episode)
                print(f"{probe} repeat {repeat}: raw={'PASS' if episode['raw_pass'] else 'FAIL'} label={'PASS' if episode['label_pass'] else 'FAIL'}")
    finally:
        physics.disconnect(client)
    metrics = _aggregate_metrics(config, label_config, episodes)
    _write_report(report_root, metrics, str(getattr(physics, "__version__", physics.getAPIVersion())))
    print(json.dumps({"verdict": metrics["verdict"], "capability_matrix": metrics["capability_matrix"]}, indent=2))
    return metrics
