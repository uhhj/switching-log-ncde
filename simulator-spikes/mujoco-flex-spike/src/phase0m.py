from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import numpy as np
import yaml

from .controller import EndpointServo
from .labels import derive_labels
from .passage import PassageScene, reachability_clearance


BRANCHES = ("centered", "centered_repeat", "offset_medium", "offset_large")
NON_REPEAT_BRANCHES = ("centered", "offset_medium", "offset_large")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"expected mapping in {path}")
    return value


def resolve_phase0m(
    config_path: Path,
) -> Tuple[Path, Dict[str, Any], Dict[str, Any]]:
    config_path = Path(config_path).resolve()
    spike_root = config_path.parent.parent
    config = _load_yaml(config_path)
    oracle_path = (
        spike_root / str(config["oracle"]["source_config"])
    ).resolve()
    oracle_config = _load_yaml(oracle_path)
    return spike_root, config, dict(oracle_config["labels"])


def branch_y_target(
    branch: str, radius_m: float, config: Mapping[str, Any]
) -> float:
    key = {
        "centered": "centered_offset_r",
        "centered_repeat": "centered_offset_r",
        "offset_medium": "medium_offset_r",
        "offset_large": "large_offset_r",
    }[branch]
    return float(config["branches"][key]) * float(radius_m)


def _pad_rows(rows: Sequence[Sequence[int]]) -> np.ndarray:
    width = max((len(row) for row in rows), default=0)
    output = np.full((len(rows), width), -1, dtype=np.int64)
    for index, row in enumerate(rows):
        if row:
            output[index, : len(row)] = row
    return output


def prepare_common_state(
    scene: PassageScene,
    seed: int,
    config: Mapping[str, Any],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    radius = scene.geometry.radius_m
    rng = np.random.default_rng(int(seed))
    amplitude = float(config["seed_variation"]["initial_y_uniform_r"]) * radius
    initial_y = float(rng.uniform(-amplitude, amplitude))
    scene.reset_common(initial_y, float(config["simulation"]["gravity_mps2"]))
    timestep = float(config["simulation"]["timestep_s"])
    settle_steps = int(round(float(config["simulation"]["settle_s"]) / timestep))
    for _ in range(settle_steps):
        scene.simulator.step()
    positions = scene.simulator.vertex_positions()
    velocities = scene.simulator.vertex_velocities()
    _, head_velocity = scene.simulator.endpoint_state()
    fixture = scene.fixture_contact(head_velocity)
    head_before_entry = bool(
        positions[scene.head_index, 0] < scene.geometry.entry_x_m
    )
    contact_free = int(fixture["contact_count"]) == 0
    finite = bool(np.all(np.isfinite(scene.simulator.data.qpos))) and bool(
        np.all(np.isfinite(scene.simulator.data.qvel))
    )
    max_speed = float(np.max(np.linalg.norm(velocities, axis=1)))
    speed_pass = max_speed <= float(
        config["gates"]["preparation_max_node_speed_mps"]
    )
    preparation = {
        "seed": int(seed),
        "initial_y_m": initial_y,
        "head_before_entry": head_before_entry,
        "fixture_contact_free": contact_free,
        "finite_state": finite,
        "max_node_speed_mps": max_speed,
        "low_speed": speed_pass,
        "pass": bool(head_before_entry and contact_free and finite and speed_pass),
    }
    return scene.simulator.copy_state(), preparation


def _save_common_state(
    path: Path,
    state: Mapping[str, Any],
    preparation: Mapping[str, Any],
    scene: PassageScene,
) -> None:
    np.savez_compressed(
        path,
        qpos=state["qpos"],
        qvel=state["qvel"],
        act=state["act"],
        time=np.asarray(state["time"]),
        mocap_pos=state["mocap_pos"],
        mocap_quat=state["mocap_quat"],
        ordered_vertex_positions=scene.simulator.vertex_positions(),
        ordered_vertex_velocities=scene.simulator.vertex_velocities(),
        head_index=np.asarray(scene.head_index),
        leading_indices=scene.leading_indices,
        initial_y_m=np.asarray(preparation["initial_y_m"]),
        preparation_pass=np.asarray(preparation["pass"]),
        max_node_speed_mps=np.asarray(preparation["max_node_speed_mps"]),
        fixture_contact_free=np.asarray(preparation["fixture_contact_free"]),
    )


def run_branch(
    scene: PassageScene,
    state: Mapping[str, Any],
    branch: str,
    seed: int,
    config: Mapping[str, Any],
    thresholds: Mapping[str, Any],
) -> Dict[str, np.ndarray]:
    scene.simulator.restore_state(dict(state))
    restored_qpos = scene.simulator.data.qpos.copy()
    if not np.array_equal(restored_qpos, np.asarray(state["qpos"])):
        raise RuntimeError("common state restore failed")
    timestep = float(config["simulation"]["timestep_s"])
    hz = 1.0 / timestep
    active_steps = int(
        round(float(config["controller"]["branch_duration_s"]) / timestep)
    )
    hold_steps = int(
        round(float(config["controller"]["post_command_hold_s"]) / timestep)
    )
    initial_vertices = scene.simulator.vertex_positions()
    initial_leading_mean = np.mean(
        initial_vertices[scene.leading_indices], axis=0
    )
    head_position, _ = scene.simulator.endpoint_state()
    y_target = branch_y_target(
        branch, scene.geometry.radius_m, config
    )
    servo = EndpointServo.from_config(head_position, y_target, config)
    records: Dict[str, list] = {
        key: []
        for key in (
            "time",
            "ordered_vertex_positions",
            "ordered_vertex_velocities",
            "head_position",
            "leading4_mean_position",
            "command_forward",
            "command_lateral",
            "command_active",
            "applied_endpoint_force",
            "progress",
            "fixture_contact_count",
            "normal_force_sum",
            "normal_force_max",
            "tangential_force_sum",
            "tangential_force_max",
            "tangential_speed_mean",
            "tangential_speed_max",
            "floor_contact_count",
            "floor_normal_force_sum",
            "contact_element_ids",
            "contact_vertex_ids",
            "phase",
        )
    }
    success_step = -1
    total_steps = active_steps + hold_steps
    for step in range(total_steps):
        position, velocity = scene.simulator.endpoint_state()
        active = bool(step < active_steps and success_step < 0)
        force, command_forward, command_lateral, command_active = servo.command(
            min(step, active_steps) * timestep,
            position,
            velocity,
            active,
        )
        scene.simulator.step(force)
        vertices = scene.simulator.vertex_positions()
        vertex_velocities = scene.simulator.vertex_velocities()
        head = vertices[scene.head_index]
        head_velocity = vertex_velocities[scene.head_index]
        leading_mean = np.mean(vertices[scene.leading_indices], axis=0)
        fixture = scene.fixture_contact(head_velocity)
        floor = scene.floor_contact(head_velocity)
        records["time"].append((step + 1) * timestep)
        records["ordered_vertex_positions"].append(vertices)
        records["ordered_vertex_velocities"].append(vertex_velocities)
        records["head_position"].append(head)
        records["leading4_mean_position"].append(leading_mean)
        records["command_forward"].append(command_forward)
        records["command_lateral"].append(command_lateral)
        records["command_active"].append(command_active)
        records["applied_endpoint_force"].append(force)
        records["progress"].append(
            float(leading_mean[0] - initial_leading_mean[0])
        )
        records["fixture_contact_count"].append(fixture["contact_count"])
        records["normal_force_sum"].append(fixture["normal_force_sum"])
        records["normal_force_max"].append(fixture["normal_force_max"])
        records["tangential_force_sum"].append(fixture["tangent_force_sum"])
        records["tangential_force_max"].append(fixture["tangent_force_max"])
        records["tangential_speed_mean"].append(
            fixture["tangent_velocity_mean"]
        )
        records["tangential_speed_max"].append(
            fixture["tangent_velocity_max"]
        )
        records["floor_contact_count"].append(floor["contact_count"])
        records["floor_normal_force_sum"].append(floor["normal_force_sum"])
        records["contact_element_ids"].append(fixture["element_ids"])
        records["contact_vertex_ids"].append(fixture["vertex_ids"])
        records["phase"].append("insertion" if command_active else "hold")
        if success_step < 0 and scene.success():
            success_step = step
        if success_step >= 0 and step >= success_step + hold_steps:
            break

    element_rows = records.pop("contact_element_ids")
    vertex_rows = records.pop("contact_vertex_ids")
    trace = {key: np.asarray(value) for key, value in records.items()}
    trace["contact_element_ids"] = _pad_rows(element_rows)
    trace["contact_vertex_ids"] = _pad_rows(vertex_rows)
    command = np.column_stack(
        (
            trace["command_forward"],
            trace["command_lateral"],
            np.zeros(trace["command_forward"].shape),
        )
    )
    label_trace = {
        "contact_count": trace["fixture_contact_count"],
        "normal_force": trace["normal_force_sum"],
        "tangent_velocity": trace["tangential_speed_mean"],
        "command": command,
        "progress": trace["progress"],
        "phase": trace["phase"],
    }
    labels = derive_labels(label_trace, thresholds, hz)
    trace.update(
        {
            "contact_layer": labels["contact_label"],
            "friction_layer": labels["friction_label"],
            "failure_layer": labels["failure_label"],
            "events": labels["events"],
            "event_names": labels["event_names"],
            "progress_delta": labels["progress_delta"],
            "branch": np.asarray(branch),
            "seed": np.asarray(int(seed)),
            "success": np.asarray(success_step >= 0),
            "success_step": np.asarray(success_step),
            "y_target_m": np.asarray(y_target),
        }
    )
    return trace


def run_seeds(config_path: Path, seeds: Sequence[int]) -> int:
    spike_root, config, thresholds = resolve_phase0m(config_path)
    source_model = spike_root / str(config["simulation"]["source_model"])
    scene = PassageScene(source_model, config)
    timestep = float(config["simulation"]["timestep_s"])
    if not np.isclose(scene.simulator.timestep, timestep):
        raise ValueError("model timestep and config disagree")
    data_root = spike_root / str(config["paths"]["report_root"]) / "data"
    count = 0
    for seed in seeds:
        seed_root = data_root / f"seed_{int(seed)}"
        seed_root.mkdir(parents=True, exist_ok=True)
        state, preparation = prepare_common_state(scene, int(seed), config)
        _save_common_state(
            seed_root / "common_state.npz",
            state,
            preparation,
            scene,
        )
        metadata = {"preparation": preparation, "branches": {}}
        centered_commands = None
        for branch in BRANCHES:
            trace = run_branch(
                scene, state, branch, int(seed), config, thresholds
            )
            np.savez_compressed(seed_root / f"{branch}.npz", **trace)
            commands = np.column_stack(
                (
                    trace["command_forward"],
                    trace["command_lateral"],
                    trace["command_active"].astype(np.float64),
                )
            )
            if branch == "centered":
                centered_commands = commands
            if branch == "centered_repeat" and not np.array_equal(
                commands, centered_commands
            ):
                raise RuntimeError("centered repeat commands differ")
            metadata["branches"][branch] = {
                "samples": int(trace["time"].size),
                "success": bool(np.asarray(trace["success"]).item()),
                "final_progress_m": float(trace["progress"][-1]),
            }
            count += 1
        with (seed_root / "metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(
            f"seed {int(seed)}: preparation={'PASS' if preparation['pass'] else 'FAIL'}, "
            f"branches=4"
        )
    return count


def preflight(config_path: Path) -> Dict[str, Any]:
    spike_root, config, thresholds = resolve_phase0m(config_path)
    source_model = spike_root / str(config["simulation"]["source_model"])
    scene = PassageScene(source_model, config)
    state, preparation = prepare_common_state(
        scene, int(config["experiment"]["seeds"][0]), config
    )
    del state
    clearances = {
        branch: reachability_clearance(
            scene.geometry,
            branch_y_target(branch, scene.geometry.radius_m, config),
        )
        for branch in ("centered", "offset_medium", "offset_large")
    }
    required_oracle = {
        "contact_normal_force_min_n",
        "stick_tangent_speed_max_mps",
        "slip_tangent_speed_min_mps",
        "command_active_speed_min_mps",
        "jam_normal_force_min_n",
        "jam_window_ms",
        "jam_progress_max_m",
        "minimum_mode_duration_ms",
    }
    checks = {
        "model_loads": True,
        "vertex_ordering_resolved": bool(
            np.all(np.diff(scene.simulator.vertex_positions()[:, 0]) > 0)
        ),
        "head_index_resolved": scene.head_index
        == scene.simulator.vertex_body_ids.size - 1,
        "leading4_resolved": scene.leading_indices.size == 4,
        "fixture_geoms_exist": all(value >= 0 for value in scene.fixture_ids),
        "entry_exit_finite": bool(
            np.isfinite(scene.geometry.entry_x_m)
            and np.isfinite(scene.geometry.exit_x_m)
        ),
        "common_fixture_contact_free": bool(
            preparation["fixture_contact_free"]
        ),
        "oracle_import": required_oracle.issubset(thresholds),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "preparation": preparation,
        "cable_radius_m": scene.geometry.radius_m,
        "entry_x_m": scene.geometry.entry_x_m,
        "exit_x_m": scene.geometry.exit_x_m,
        "clearance_m": clearances,
        "head_index": scene.head_index,
        "leading_indices": scene.leading_indices.tolist(),
        "fixture_ids": list(scene.fixture_ids),
        "oracle_thresholds": thresholds,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return result
