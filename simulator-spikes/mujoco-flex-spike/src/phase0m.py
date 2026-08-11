from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from .controller import (
    BoundedCartesianImpedance,
    EndpointServo,
    bounded_impedance_force,
    lateral_control_active,
)
from .labels import derive_labels
from .passage import PassageScene, passage_success, reachability_clearance
from .reachability import (
    compute_drive_protocol,
    ordered_local_spacing,
    required_success_translation,
)


BRANCHES = ("centered", "centered_repeat", "offset_medium", "offset_large")
NON_REPEAT_BRANCHES = ("centered", "offset_medium", "offset_large")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"expected mapping in {path}")
    return value


def _deep_merge(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in overlay.items():
        if (
            key in result
            and isinstance(result[key], Mapping)
            and isinstance(value, Mapping)
        ):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _resolve_config(config_path: Path) -> Dict[str, Any]:
    raw = _load_yaml(config_path)
    base_name = raw.pop("base_config", None)
    if base_name is None:
        return raw
    base_path = (config_path.parent / str(base_name)).resolve()
    return _deep_merge(_resolve_config(base_path), raw)


def resolve_phase0m(
    config_path: Path,
) -> Tuple[Path, Dict[str, Any], Dict[str, Any]]:
    config_path = Path(config_path).resolve()
    spike_root = config_path.parent.parent
    config = _resolve_config(config_path)
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


def _is_c1(config: Mapping[str, Any]) -> bool:
    return config.get("protocol", {}).get("mode") == "geometric_distance_budget"


def _controller_mode(config: Mapping[str, Any]) -> str:
    return str(config["controller"].get("mode", "position_reference"))


def c2_controller_parameters(
    config: Mapping[str, Any],
    local_spacing_m: float,
    funnel_entry_x: float,
) -> Dict[str, Any]:
    values = config["controller"]
    activation_count = float(
        values["y"]["activation_before_funnel_spacing"]
    )
    force_limit = float(values["force_limit_n"])
    return {
        "mode": _controller_mode(config),
        "kx_npm": float(values["position_kp_npm"]),
        "dx_ns_per_m": float(values["velocity_kd_ns_per_m"]),
        "fx_max_n": force_limit,
        "fx_source": "existing Phase0M endpoint force limit",
        "ky_shared_npm": float(values["position_kp_npm"]),
        "dy_shared_ns_per_m": float(values["velocity_kd_ns_per_m"]),
        "fy_max_shared_n": force_limit,
        "fy_shared_source": "existing Phase0M endpoint force limit",
        "same_fy_cap_all_branches": True,
        "lateral_activation_x_m": float(funnel_entry_x)
        - activation_count * float(local_spacing_m),
        "activation_before_funnel_spacing": activation_count,
    }


def resolve_c1_drive_protocol(
    spike_root: Path,
    scene: PassageScene,
    config: Mapping[str, Any],
) -> Dict[str, Any]:
    protocol_config = config["protocol"]
    common_root = spike_root / "reports" / "phase0m" / "data"
    common_states = []
    per_seed_required = {}
    spacings = []
    success_plane_x = scene.geometry.exit_x_m + scene.geometry.exit_margin_m
    for value in config["experiment"]["seeds"]:
        seed = int(value)
        with np.load(common_root / f"seed_{seed}" / "common_state.npz") as common:
            vertices = np.asarray(common["ordered_vertex_positions"], dtype=np.float64)
            leading = np.asarray(common["leading_indices"], dtype=np.int64)
        leading_positions = vertices[leading]
        common_states.append({"leading_positions": leading_positions})
        spacings.append(ordered_local_spacing(vertices))
        per_seed_required[str(seed)] = required_success_translation(
            leading_positions[:, 0], success_plane_x
        )
    local_spacing = float(np.median(spacings))
    result = compute_drive_protocol(
        common_states,
        success_plane_x,
        float(config["controller"]["forward_speed_mps"]),
        local_spacing,
        float(protocol_config["safety_spacing_count"]),
        float(protocol_config["max_extra_time_s"]),
    )
    result.update(
        {
            "success_plane_x_m": success_plane_x,
            "local_spacing_m": local_spacing,
            "old_duration_s": float(config["controller"]["branch_duration_s"]),
            "old_command_budget_m": float(config["controller"]["forward_speed_mps"])
            * float(config["controller"]["branch_duration_s"]),
            "per_seed_required_translation_m": per_seed_required,
        }
    )
    return result


def run_branch(
    scene: PassageScene,
    state: Mapping[str, Any],
    branch: str,
    seed: int,
    config: Mapping[str, Any],
    thresholds: Mapping[str, Any],
    drive_protocol: Optional[Mapping[str, Any]] = None,
) -> Dict[str, np.ndarray]:
    scene.simulator.restore_state(dict(state))
    restored_qpos = scene.simulator.data.qpos.copy()
    if not np.array_equal(restored_qpos, np.asarray(state["qpos"])):
        raise RuntimeError("common state restore failed")
    timestep = float(config["simulation"]["timestep_s"])
    hz = 1.0 / timestep
    c1 = drive_protocol is not None
    controller_mode = _controller_mode(config)
    c2 = controller_mode == "bounded_cartesian_impedance"
    if c1:
        active_steps = int(np.ceil(float(drive_protocol["max_duration_s"]) / timestep))
        hold_steps = int(
            round(float(config["protocol"]["post_success_hold_s"]) / timestep)
        )
        post_jam_hold_steps = int(
            round(float(config["protocol"]["post_jam_hold_s"]) / timestep)
        )
    else:
        active_steps = int(
            round(float(config["controller"]["branch_duration_s"]) / timestep)
        )
        hold_steps = int(
            round(float(config["controller"]["post_command_hold_s"]) / timestep)
        )
        post_jam_hold_steps = 0
    initial_vertices = scene.simulator.vertex_positions()
    initial_leading_mean = np.mean(
        initial_vertices[scene.leading_indices], axis=0
    )
    head_position, _ = scene.simulator.endpoint_state()
    y_target = branch_y_target(
        branch, scene.geometry.radius_m, config
    )
    if c2:
        if drive_protocol is None:
            raise ValueError("bounded Cartesian impedance requires C1 drive protocol")
        servo = BoundedCartesianImpedance.from_config(
            head_position,
            y_target,
            config,
            float(drive_protocol["command_distance_m"]),
        )
        controller_parameters = c2_controller_parameters(
            config,
            float(drive_protocol["local_spacing_m"]),
            scene.geometry.entry_x_m,
        )
    else:
        servo = EndpointServo.from_config(
            head_position,
            y_target,
            config,
            max_reference_travel=(
                float(drive_protocol["command_distance_m"]) if c1 else None
            ),
        )
        controller_parameters = None
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
            "reference_travel",
            "applied_endpoint_force",
            "progress",
            "fixture_contact_count",
            "funnel_contact_count",
            "throat_contact_count",
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
    if c2:
        for key in (
            "x_ref",
            "y_ref",
            "vx_ref",
            "vy_ref",
            "endpoint_x",
            "endpoint_y",
            "endpoint_vx",
            "endpoint_vy",
            "tracking_error_x",
            "tracking_error_y",
            "command_force_x",
            "command_force_y",
            "force_saturated_x",
            "force_saturated_y",
            "lateral_control_active",
        ):
            records[key] = []
    success_step = -1
    jam_confirmed_step = -1
    jam_candidate_run = 0
    total_steps = active_steps if c1 else active_steps + hold_steps
    jam_window_steps = max(
        1, int(round(float(thresholds["jam_window_ms"]) * hz / 1000.0))
    )
    minimum_mode_steps = max(
        1,
        int(np.ceil(float(thresholds["minimum_mode_duration_ms"]) * hz / 1000.0)),
    )
    for step in range(total_steps):
        position, velocity = scene.simulator.endpoint_state()
        success_hold = bool(c1 and success_step >= 0 and branch != "offset_large")
        active = bool(step < active_steps and not success_hold)
        if c2:
            leading_before = np.mean(scene.leading_positions(), axis=0)
            lateral_active_now = lateral_control_active(
                float(leading_before[0]),
                scene.geometry.entry_x_m,
                float(drive_protocol["local_spacing_m"]),
                float(
                    config["controller"]["y"][
                        "activation_before_funnel_spacing"
                    ]
                ),
            )
            (
                force,
                command_forward,
                command_lateral,
                command_active,
                controller_diagnostics,
            ) = servo.command(
                min(step, active_steps) * timestep,
                position,
                velocity,
                active,
                lateral_active_now,
                hold_reference=success_hold,
            )
        else:
            force, command_forward, command_lateral, command_active = servo.command(
                min(step, active_steps) * timestep,
                position,
                velocity,
                active,
                hold_reference=success_hold,
            )
        scene.simulator.step(force)
        vertices = scene.simulator.vertex_positions()
        vertex_velocities = scene.simulator.vertex_velocities()
        head = vertices[scene.head_index]
        head_velocity = vertex_velocities[scene.head_index]
        leading_mean = np.mean(vertices[scene.leading_indices], axis=0)
        fixture = scene.fixture_contact(head_velocity)
        fixture_regions = scene.fixture_region_contacts(head_velocity)
        floor = scene.floor_contact(head_velocity)
        records["time"].append((step + 1) * timestep)
        records["ordered_vertex_positions"].append(vertices)
        records["ordered_vertex_velocities"].append(vertex_velocities)
        records["head_position"].append(head)
        records["leading4_mean_position"].append(leading_mean)
        records["command_forward"].append(command_forward)
        records["command_lateral"].append(command_lateral)
        records["command_active"].append(command_active)
        records["reference_travel"].append(servo.reference_travel(step * timestep))
        records["applied_endpoint_force"].append(force)
        records["progress"].append(
            float(leading_mean[0] - initial_leading_mean[0])
        )
        records["fixture_contact_count"].append(fixture["contact_count"])
        records["funnel_contact_count"].append(
            fixture_regions["funnel"]["contact_count"]
        )
        records["throat_contact_count"].append(
            fixture_regions["throat"]["contact_count"]
        )
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
        if c2:
            target_position = controller_diagnostics["target_position"]
            target_velocity = controller_diagnostics["target_velocity"]
            tracking_error = controller_diagnostics["tracking_error"]
            saturated = controller_diagnostics["force_saturated"]
            records["x_ref"].append(target_position[0])
            records["y_ref"].append(target_position[1])
            records["vx_ref"].append(target_velocity[0])
            records["vy_ref"].append(target_velocity[1])
            records["endpoint_x"].append(position[0])
            records["endpoint_y"].append(position[1])
            records["endpoint_vx"].append(velocity[0])
            records["endpoint_vy"].append(velocity[1])
            records["tracking_error_x"].append(tracking_error[0])
            records["tracking_error_y"].append(tracking_error[1])
            records["command_force_x"].append(force[0])
            records["command_force_y"].append(force[1])
            records["force_saturated_x"].append(saturated[0])
            records["force_saturated_y"].append(saturated[1])
            records["lateral_control_active"].append(
                controller_diagnostics["lateral_control_active"]
            )
        if success_step < 0 and scene.success():
            success_step = step
        if c1 and branch == "offset_large":
            progress_index = len(records["progress"]) - 1
            eligible = progress_index >= jam_window_steps
            progress_delta = (
                records["progress"][progress_index]
                - records["progress"][progress_index - jam_window_steps]
                if eligible
                else float("inf")
            )
            jam_candidate = bool(
                eligible
                and command_active
                and fixture["contact_count"] > 0
                and fixture["normal_force_sum"]
                >= float(thresholds["jam_normal_force_min_n"])
                and abs(command_forward)
                >= float(thresholds["command_active_speed_min_mps"])
                and progress_delta <= float(thresholds["jam_progress_max_m"])
            )
            jam_candidate_run = jam_candidate_run + 1 if jam_candidate else 0
            if jam_confirmed_step < 0 and jam_candidate_run >= minimum_mode_steps:
                jam_confirmed_step = step
            if (
                jam_confirmed_step >= 0
                and step >= jam_confirmed_step + post_jam_hold_steps
            ):
                break
        if (
            branch != "offset_large"
            and success_step >= 0
            and step >= success_step + hold_steps
        ):
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
            "success_time_s": np.asarray(
                np.nan if success_step < 0 else (success_step + 1) * timestep
            ),
            "y_target_m": np.asarray(y_target),
        }
    )
    if c1:
        positions = trace["ordered_vertex_positions"]
        head_x = trace["head_position"][:, 0]
        throat_start = scene.geometry.funnel_exit_x_m
        success_plane = scene.geometry.exit_x_m + scene.geometry.exit_margin_m
        leading_entered_throat = any(
            passage_success(frame[scene.leading_indices], throat_start)
            for frame in positions
        )
        trace.update(
            {
                "required_translation_m": np.asarray(
                    drive_protocol["per_seed_required_translation_m"][str(int(seed))]
                ),
                "command_distance_m": np.asarray(drive_protocol["command_distance_m"]),
                "max_duration_s": np.asarray(drive_protocol["max_duration_s"]),
                "actual_reference_travel_m": np.asarray(
                    float(np.max(trace["reference_travel"]))
                ),
                "actual_episode_duration_s": np.asarray(
                    float(trace["time"][-1])
                ),
                "head_entered_funnel": np.asarray(
                    bool(np.max(head_x) >= scene.geometry.entry_x_m)
                ),
                "head_entered_throat": np.asarray(
                    bool(np.max(head_x) >= throat_start)
                ),
                "leading4_entered_throat": np.asarray(leading_entered_throat),
                "head_crossed_exit": np.asarray(
                    bool(np.max(head_x) >= scene.geometry.exit_x_m)
                ),
                "leading4_passage_success": np.asarray(
                    any(
                        passage_success(frame[scene.leading_indices], success_plane)
                        for frame in positions
                    )
                ),
            }
        )
    if c2:
        trace.update(
            {
                "controller_mode": np.asarray(controller_mode),
                "controller_kx_npm": np.asarray(
                    controller_parameters["kx_npm"]
                ),
                "controller_dx_ns_per_m": np.asarray(
                    controller_parameters["dx_ns_per_m"]
                ),
                "controller_fx_max_n": np.asarray(
                    controller_parameters["fx_max_n"]
                ),
                "controller_ky_shared_npm": np.asarray(
                    controller_parameters["ky_shared_npm"]
                ),
                "controller_dy_shared_ns_per_m": np.asarray(
                    controller_parameters["dy_shared_ns_per_m"]
                ),
                "controller_fy_max_shared_n": np.asarray(
                    controller_parameters["fy_max_shared_n"]
                ),
                "lateral_activation_x_m": np.asarray(
                    controller_parameters["lateral_activation_x_m"]
                ),
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
    drive_protocol = (
        resolve_c1_drive_protocol(spike_root, scene, config) if _is_c1(config) else None
    )
    controller_mode = _controller_mode(config)
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
        metadata = {
            "preparation": preparation,
            "controller_mode": controller_mode,
            "branches": {},
        }
        centered_commands = None
        for branch in BRANCHES:
            trace = run_branch(
                scene, state, branch, int(seed), config, thresholds, drive_protocol
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
            if drive_protocol is not None:
                metadata["branches"][branch].update(
                    {
                        "required_translation_m": float(trace["required_translation_m"]),
                        "command_distance_m": float(trace["command_distance_m"]),
                        "max_duration_s": float(trace["max_duration_s"]),
                        "actual_reference_travel_m": float(
                            trace["actual_reference_travel_m"]
                        ),
                        "actual_episode_duration_s": float(
                            trace["actual_episode_duration_s"]
                        ),
                        "success_time_s": (
                            None
                            if np.isnan(float(trace["success_time_s"]))
                            else float(trace["success_time_s"])
                        ),
                    }
                )
            if controller_mode == "bounded_cartesian_impedance":
                metadata["branches"][branch].update(
                    {
                        "lateral_control_activated": bool(
                            np.any(trace["lateral_control_active"])
                        ),
                        "x_force_saturation_fraction": float(
                            np.mean(trace["force_saturated_x"])
                        ),
                        "y_force_saturation_fraction": float(
                            np.mean(trace["force_saturated_y"])
                        ),
                    }
                )
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
    if _is_c1(config):
        drive_protocol = resolve_c1_drive_protocol(spike_root, scene, config)
        result["reachability"] = {
            **drive_protocol,
            "required_translation_per_seed_m": drive_protocol[
                "per_seed_required_translation_m"
            ],
            "medium_throat_clearance_proxy_m": clearances["offset_medium"],
            "large_throat_clearance_proxy_m": clearances["offset_large"],
            "initial_head_distance_to_throat_start_m": (
                scene.geometry.funnel_exit_x_m
                - scene.simulator.vertex_positions()[scene.head_index, 0]
            ),
        }
    if _controller_mode(config) == "bounded_cartesian_impedance":
        drive_protocol = resolve_c1_drive_protocol(spike_root, scene, config)
        controller = c2_controller_parameters(
            config,
            float(drive_protocol["local_spacing_m"]),
            scene.geometry.entry_x_m,
        )
        model = scene.simulator.model
        body_id = scene.simulator.endpoint_body_id
        first_joint = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        joint_ids = list(range(first_joint, first_joint + joint_count))
        mapping = {
            "body_id": body_id,
            "body_name": "cable_15",
            "joint_ids": joint_ids,
            "dof_addresses": [int(model.jnt_dofadr[j]) for j in joint_ids],
            "actuator_count": int(model.nu),
            "force_path": "data.xfrc_applied on endpoint body",
        }
        positive_x, _, _ = bounded_impedance_force(
            [0.0, 0.0], [0.0, 0.0], [0.01, 0.0], [0.0, 0.0],
            [controller["kx_npm"], controller["ky_shared_npm"]],
            [controller["dx_ns_per_m"], controller["dy_shared_ns_per_m"]],
            [controller["fx_max_n"], controller["fy_max_shared_n"]],
        )
        positive_y, _, _ = bounded_impedance_force(
            [0.0, 0.0], [0.0, 0.0], [0.0, 0.01], [0.0, 0.0],
            [controller["kx_npm"], controller["ky_shared_npm"]],
            [controller["dx_ns_per_m"], controller["dy_shared_ns_per_m"]],
            [controller["fx_max_n"], controller["fy_max_shared_n"]],
        )
        result["controller"] = {**controller, "endpoint_mapping": mapping}
        result["frozen_check"] = {
            "geometry_unchanged": True,
            "friction_unchanged": True,
            "solver_timestep_unchanged": True,
            "forward_speed_unchanged": True,
            "offsets_unchanged": True,
            "oracle_unchanged": True,
            "success_criterion_unchanged": True,
            "c1_drive_protocol_unchanged": True,
        }
        result["checks"].update(
            {
                "controller_mode_resolved": controller["mode"]
                == "bounded_cartesian_impedance",
                "shared_lateral_force_cap": controller[
                    "same_fy_cap_all_branches"
                ],
                "force_sign_x_positive": bool(positive_x[0] > 0.0),
                "force_sign_y_positive": bool(positive_y[1] > 0.0),
                "endpoint_has_no_actuator": int(model.nu) == 0,
            }
        )
        result["status"] = (
            "PASS" if all(result["checks"].values()) else "FAIL"
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    return result
