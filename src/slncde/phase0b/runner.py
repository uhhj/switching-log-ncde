from __future__ import annotations

import json
import os
import random
import subprocess
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple

import numpy as np

from slncde.phase0a.runner import load_simulator
from slncde.phase0a.snapshot import (
    capture_runtime_state,
    capture_world_state,
    restore_joint_control_state,
    restore_runtime_state,
)
from slncde.phase0b.preparation import (
    canonical_active_endpoint_index,
    canonicalize_local_segment,
    endpoint_local_indices,
    local_geometry_passes,
    local_segment_diagnostics,
)


BRANCH_OFFSETS = {
    "nominal": "nominal_lateral_offset_m",
    "nominal_repeat": "nominal_lateral_offset_m",
    "slide_probe": "slide_lateral_offset_m",
    "jam_probe": "jam_lateral_offset_m",
}


def canonical_frame_arrays(
    config: Mapping[str, Any], endpoint_positions: np.ndarray = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the fixed R1 frame; endpoint positions cannot affect it."""
    del endpoint_positions
    frame = config["canonical_frame"]
    entry_xy = np.asarray(frame["entry_center_xy_m"], dtype=np.float64)
    axis_xy = np.asarray(frame["insertion_axis_xy"], dtype=np.float64)
    norm = float(np.linalg.norm(axis_xy))
    if (
        entry_xy.shape != (2,)
        or axis_xy.shape != (2,)
        or not np.all(np.isfinite(entry_xy))
        or not np.all(np.isfinite(axis_xy))
        or norm <= 1e-9
    ):
        raise ValueError("canonical frame requires finite XY entry and axis")
    axis_xy = axis_xy / norm
    insertion_axis = np.asarray(
        [axis_xy[0], axis_xy[1], 0.0], dtype=np.float64
    )
    lateral_axis = np.asarray(
        [-axis_xy[1], axis_xy[0], 0.0], dtype=np.float64
    )
    return entry_xy, insertion_axis, lateral_axis


def set_fixture_environment(config: Mapping[str, Any], seed: int) -> None:
    fixture = config["fixture"]
    simulator = config["simulator"]
    values = {
        "CUDA_VISIBLE_DEVICES": "",
        "SLNCDE_FIXTURE_DEFER_CREATION": int(
            bool(
                config.get("preparation", {}).get(
                    "defer_fixture_creation", False
                )
            )
        ),
        "SLNCDE_FIXTURE_CHANNEL_GAP": fixture["channel_gap_m"],
        "SLNCDE_FIXTURE_CHANNEL_LENGTH": fixture["channel_length_m"],
        "SLNCDE_FIXTURE_WALL_THICKNESS": fixture["wall_thickness_m"],
        "SLNCDE_FIXTURE_WALL_HEIGHT": fixture["wall_height_m"],
        "SLNCDE_FIXTURE_ENTRY_DISTANCE": fixture[
            "entry_distance_from_endpoint_m"
        ],
        "SLNCDE_FIXTURE_WALL_FRICTION": fixture[
            "wall_lateral_friction"
        ],
        "SLNCDE_FIXTURE_TRACE_STRIDE": simulator["trace_stride"],
        "SLNCDE_FIXTURE_STAGING_BEFORE_ENTRY": config["motion"][
            "staging_before_entry_m"
        ],
        "SLNCDE_CABLE_SPAWN_MODE": (
            "canonical"
            if config.get("preparation", {}).get("method")
            == "canonical_spawn"
            else "random"
        ),
    }
    frame = config.get("canonical_frame", {"mode": "auto_endpoint"})
    values["SLNCDE_FIXTURE_FRAME_MODE"] = frame.get(
        "mode", "auto_endpoint"
    )
    if values["SLNCDE_FIXTURE_FRAME_MODE"] == "canonical":
        entry_xy, insertion_axis, _ = canonical_frame_arrays(config)
        entry_x, entry_y = entry_xy
        axis_x, axis_y = insertion_axis[:2]
        values.update(
            {
                "SLNCDE_FIXTURE_ENTRY_X": entry_x,
                "SLNCDE_FIXTURE_ENTRY_Y": entry_y,
                "SLNCDE_FIXTURE_AXIS_X": axis_x,
                "SLNCDE_FIXTURE_AXIS_Y": axis_y,
            }
        )
    for name, value in values.items():
        os.environ[name] = str(value)
    random.seed(int(seed))
    np.random.seed(int(seed))


def _simulator_commit(simulator_root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(simulator_root), "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def _active_endpoint_position(task) -> np.ndarray:
    import pybullet as p

    endpoint_id = int(task.fixture_spec()["active_endpoint_id"])
    return np.asarray(
        p.getBasePositionAndOrientation(endpoint_id)[0], dtype=np.float64
    )


def choose_active_endpoint_from_positions(
    positions: np.ndarray, staging_xy: np.ndarray
) -> int:
    values = np.asarray(positions, dtype=np.float64)
    target = np.asarray(staging_xy, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 2 or values.shape[1] < 2:
        raise ValueError("positions must contain at least two XY points")
    if target.shape != (2,):
        raise ValueError("staging_xy must contain two values")
    candidates = np.asarray([0, len(values) - 1], dtype=np.int64)
    distances = np.linalg.norm(values[candidates, :2] - target, axis=1)
    return int(candidates[int(np.argmin(distances))])


def choose_active_endpoint_for_staging(task, staging_xy: np.ndarray):
    import pybullet as p

    positions = np.asarray(
        [
            p.getBasePositionAndOrientation(int(bead))[0]
            for bead in task.cable_bead_IDs
        ],
        dtype=np.float64,
    )
    index = choose_active_endpoint_from_positions(positions, staging_xy)
    return index, int(task.cable_bead_IDs[index]), positions[index].copy()


def _ee_pose(env) -> Tuple[np.ndarray, np.ndarray]:
    import pybullet as p

    state = p.getLinkState(
        env.ur5, env.ee_tip_link, computeForwardKinematics=True
    )
    return (
        np.asarray(state[0], dtype=np.float64),
        np.asarray(state[1], dtype=np.float64),
    )


def _acquire_endpoint(env, task, motion: Mapping[str, Any]) -> int:
    import pybullet as p

    endpoint_id = int(task.fixture_spec()["active_endpoint_id"])
    endpoint = _active_endpoint_position(task)
    _, orientation = _ee_pose(env)
    approach = np.concatenate(
        (
            endpoint
            + np.asarray(
                [0.0, 0.0, float(motion["approach_height_m"])],
                dtype=np.float64,
            ),
            orientation,
        )
    )
    contact = np.concatenate(
        (
            endpoint
            + np.asarray(
                [0.0, 0.0, float(motion["grasp_height_offset_m"])],
                dtype=np.float64,
            ),
            orientation,
        )
    )
    speed = float(motion["acquisition_speed_mps"])
    if not env.movep(approach.tolist(), speed=speed):
        raise RuntimeError("endpoint approach motion failed")
    if not env.movep(contact.tolist(), speed=speed):
        raise RuntimeError("endpoint contact motion failed")
    env.step_physics(4)
    env.ee.activate([endpoint_id], [])
    env.step_physics(4)
    if not bool(getattr(env.ee, "activated", False)):
        raise RuntimeError("suction did not activate")
    return endpoint_id


def _move_endpoint_to(env, task, endpoint_target: np.ndarray, speed: float) -> bool:
    endpoint = _active_endpoint_position(task)
    ee_position, orientation = _ee_pose(env)
    ee_target = ee_position + (np.asarray(endpoint_target) - endpoint)
    pose = np.concatenate((ee_target, orientation))
    return bool(
        env.movep_precise(
            pose.tolist(),
            speed=float(speed),
            label="phase0b_staging_stretch",
            record_event=False,
        )
    )


def _pose_for_endpoint_target(
    endpoint_target: np.ndarray,
    common_state: Mapping[str, np.ndarray],
    endpoint_at_snapshot: np.ndarray,
) -> np.ndarray:
    ee_offset = (
        np.asarray(common_state["ee_position"], dtype=np.float64)
        - np.asarray(endpoint_at_snapshot, dtype=np.float64)
    )
    return np.concatenate(
        (
            np.asarray(endpoint_target, dtype=np.float64) + ee_offset,
            np.asarray(common_state["ee_orientation"], dtype=np.float64),
        )
    )


def _branch_targets(
    config: Mapping[str, Any],
    staging_endpoint: np.ndarray,
    insertion_axis: np.ndarray,
    lateral_axis: np.ndarray,
    common_state: Mapping[str, np.ndarray],
    endpoint_at_snapshot: np.ndarray,
) -> Dict[str, Dict[str, np.ndarray]]:
    motion = config["motion"]

    def build(offset: float) -> Dict[str, np.ndarray]:
        align_endpoint = staging_endpoint + float(offset) * lateral_axis
        insert_endpoint = (
            align_endpoint
            + float(motion["insertion_distance_m"]) * insertion_axis
        )
        return {
            "align_endpoint": align_endpoint,
            "insert_endpoint": insert_endpoint,
            "align_pose": _pose_for_endpoint_target(
                align_endpoint, common_state, endpoint_at_snapshot
            ),
            "insert_pose": _pose_for_endpoint_target(
                insert_endpoint, common_state, endpoint_at_snapshot
            ),
        }

    nominal = build(float(motion["nominal_lateral_offset_m"]))
    targets = {
        "nominal": nominal,
        "nominal_repeat": nominal,
        "slide_probe": build(float(motion["slide_lateral_offset_m"])),
        "jam_probe": build(float(motion["jam_lateral_offset_m"])),
    }
    if not np.array_equal(
        targets["nominal"]["insert_pose"],
        targets["nominal_repeat"]["insert_pose"],
    ):
        raise RuntimeError("nominal and repeat insertion targets differ")
    return targets


def _trace_payload(
    trace,
    initial: Mapping[str, np.ndarray],
    final: Mapping[str, np.ndarray],
    motion_completed: bool,
) -> Tuple[Dict[str, np.ndarray], list]:
    rows = list(trace)
    if not rows:
        raise RuntimeError("fixture task returned an empty trace")
    fields = (
        "physics_step",
        "phase",
        "bead_positions",
        "bead_velocities",
        "ee_position",
        "joint_positions",
        "joint_velocities",
        "command_velocity_xyz",
        "endpoint_position",
        "progress_m",
        "lateral_m",
        "contact_active_beads",
        "contact_point_count",
        "normal_force_sum",
        "normal_force_max",
        "tangential_force_sum",
        "tangential_force_max",
        "tangential_speed_mean",
        "tangential_speed_max",
    )
    payload = {name: np.asarray([row[name] for row in rows]) for name in fields}
    for name, value in initial.items():
        payload[f"initial_{name}"] = np.asarray(value)
    for name, value in final.items():
        payload[f"final_{name}"] = np.asarray(value)
    payload["motion_completed"] = np.asarray(bool(motion_completed))
    contact_indices = [row["contact_bead_indices"] for row in rows]
    return payload, contact_indices


def _run_branch(
    *,
    env,
    task,
    state_id: int,
    runtime_state: Mapping[str, Any],
    common_state: Mapping[str, np.ndarray],
    branch_name: str,
    offset: float,
    targets: Mapping[str, np.ndarray],
    config: Mapping[str, Any],
) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    import pybullet as p

    p.restoreState(stateId=state_id)
    env.reset_ccda_runtime_after_restore()
    restore_runtime_state(env, runtime_state)
    restore_joint_control_state(env, common_state)
    task.begin_fixture_branch()
    initial = capture_world_state(env, task)

    align_completed = True
    speed = float(config["motion"]["insertion_speed_mps"])
    if float(offset) != 0.0:
        command = np.sign(float(offset)) * speed * np.asarray(
            task.fixture_spec()["lateral_axis"], dtype=np.float64
        )
        task.set_script_context("lateral_align", command)
        align_completed = bool(
            env.movep(targets["align_pose"].tolist(), speed=speed)
        )

    insertion_axis = np.asarray(
        task.fixture_spec()["insertion_axis"], dtype=np.float64
    )
    task.set_script_context("insertion", speed * insertion_axis)
    motion_completed = bool(
        env.movep(targets["insert_pose"].tolist(), speed=speed)
    )

    task.set_script_context("hold", [0.0, 0.0, 0.0])
    env.step_physics(int(config["motion"]["branch_hold_steps"]))
    final = capture_world_state(env, task)
    payload, contact_indices = _trace_payload(
        task.fixture_trace(), initial, final, motion_completed
    )
    details = {
        "branch": branch_name,
        "lateral_offset_m": float(offset),
        "align_completed": align_completed,
        "motion_completed": motion_completed,
        "align_endpoint_target": targets["align_endpoint"],
        "insert_endpoint_target": targets["insert_endpoint"],
        "align_pose_target": targets["align_pose"],
        "insert_pose_target": targets["insert_pose"],
        "contact_bead_indices_trace": contact_indices,
    }
    return payload, details


def _json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    return value


def _empty_preparation_payload(indices, targets) -> Dict[str, np.ndarray]:
    payload = {
        "local_bead_indices": np.asarray(indices, dtype=np.int64),
        "canonical_targets": np.asarray(targets, dtype=np.float64),
    }
    for stage in (
        "after_initial_settle",
        "after_grasp",
        "after_fixture_settle",
    ):
        payload[f"positions_{stage}"] = np.empty((0, 3), dtype=np.float64)
        payload[f"entry_signed_distances_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"alignment_cosines_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"local_speeds_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
    return payload


def _empty_canonical_spawn_payload(
    indices, endpoint_target, spawn_origin
) -> Dict[str, np.ndarray]:
    payload = {
        "local_bead_indices": np.asarray(indices, dtype=np.int64),
        "canonical_endpoint_target": np.asarray(
            endpoint_target, dtype=np.float64
        ),
        "canonical_spawn_origin": np.asarray(
            spawn_origin, dtype=np.float64
        ),
    }
    for stage in (
        "initial_spawn",
        "after_spawn_settle",
        "after_grasp",
        "after_fixture_settle",
    ):
        payload[f"positions_{stage}"] = np.empty((0, 3), dtype=np.float64)
        payload[f"entry_signed_distances_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"alignment_cosines_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"local_speeds_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"segment_lengths_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
        payload[f"spacing_errors_{stage}"] = np.empty(
            (0,), dtype=np.float64
        )
    return payload


def _store_preparation_diagnostics(
    payload: Dict[str, np.ndarray], stage: str, diagnostics
) -> None:
    payload[f"positions_{stage}"] = np.asarray(diagnostics["positions"])
    payload[f"entry_signed_distances_{stage}"] = np.asarray(
        diagnostics["signed_entry_distances_m"]
    )
    payload[f"alignment_cosines_{stage}"] = np.asarray(
        diagnostics["alignment_cosines"]
    )
    payload[f"local_speeds_{stage}"] = np.asarray(
        diagnostics["local_speeds_mps"]
    )
    payload[f"segment_lengths_{stage}"] = np.asarray(
        diagnostics["segment_lengths_m"]
    )
    payload[f"spacing_errors_{stage}"] = np.asarray(
        diagnostics["spacing_errors_m"]
    )


def _contact_free(observation: Mapping[str, Any]) -> bool:
    return bool(
        int(observation["contact_active_beads"]) == 0
        and int(observation["contact_point_count"]) == 0
    )


def _fixture_contact_after_collision_refresh(task) -> Dict[str, Any]:
    """Refresh contacts, with a PyBullet 3.0.4 overlap-query fallback."""
    import pybullet as p

    refresh = getattr(p, "performCollisionDetection", None)
    if callable(refresh):
        refresh()
    observation = task.fixture_contact_observation()
    if callable(refresh) or not _contact_free(observation):
        return observation

    spec = task.fixture_spec()
    active_indices = set()
    point_count = 0
    for bead_index, bead_id in enumerate(task.cable_bead_IDs):
        for fixture_id in spec["fixture_ids"]:
            points = p.getClosestPoints(
                int(bead_id), int(fixture_id), distance=0.0
            )
            if points:
                active_indices.add(int(bead_index))
                point_count += len(points)
    if point_count:
        observation = dict(observation)
        observation["contact_active_beads"] = len(active_indices)
        observation["contact_point_count"] = int(point_count)
        observation["contact_bead_indices"] = sorted(active_indices)
    return observation


def _write_preparation_artifacts(
    seed_root: Path,
    metadata: Mapping[str, Any],
    payload: Mapping[str, np.ndarray],
) -> None:
    np.savez_compressed(seed_root / "preparation.npz", **payload)
    with (seed_root / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(_json_ready(metadata), handle, indent=2, sort_keys=True)
        handle.write("\n")


def run_seed(config: Mapping[str, Any], seed: int, repo_root: Path) -> Path:
    import pybullet as p

    set_fixture_environment(config, seed)
    simulator_root, tasks, Environment = load_simulator(repo_root)
    task_name = str(config["simulator"]["task_name"])
    if task_name not in tasks.names:
        raise KeyError(f"task registry has no {task_name!r}")
    env = Environment(
        disp=False,
        hz=int(config["simulator"]["hz"]),
        deterministic=bool(config["simulator"]["deterministic"]),
        control_substeps=1,
        post_action_settle_steps=0,
    )
    state_id = None
    try:
        task = tasks.names[task_name]()
        env.reset(task)
        frame_mode = str(
            config.get("canonical_frame", {}).get("mode", "auto_endpoint")
        )
        local_preparation = bool(
            config.get("preparation", {}).get(
                "defer_fixture_creation", False
            )
        )
        canonical_spawn = bool(
            config.get("preparation", {}).get("method")
            == "canonical_spawn"
        )
        seed_root = (
            repo_root
            / str(config["paths"]["data_root"])
            / f"seed_{int(seed)}"
        )
        seed_root.mkdir(parents=True, exist_ok=True)

        if canonical_spawn:
            spec = task.fixture_spec()
            if bool(spec["fixture_created"]) or spec["fixture_ids"]:
                raise RuntimeError("fixture was not deferred during R1.2 reset")
            if spec.get("cable_spawn_mode") != "canonical":
                raise RuntimeError("canonical cable spawn mode was not active")
            insertion_axis = np.asarray(
                spec["insertion_axis"], dtype=np.float64
            )
            lateral_axis = np.asarray(
                spec["lateral_axis"], dtype=np.float64
            )
            if not np.allclose(insertion_axis[:2], [1.0, 0.0], atol=1e-8):
                raise RuntimeError("R1.2 requires the canonical +X insertion axis")
            entry_center = np.asarray(
                spec["entry_center"], dtype=np.float64
            )
            staging_endpoint = np.asarray(
                spec["canonical_endpoint_target"], dtype=np.float64
            )
            expected_staging = (
                entry_center
                - float(config["motion"]["staging_before_entry_m"])
                * insertion_axis
            )
            if not np.allclose(staging_endpoint, expected_staging, atol=1e-8):
                raise RuntimeError("task and runner canonical staging targets differ")
            endpoint_index = canonical_active_endpoint_index(
                len(task.cable_bead_IDs)
            )
            if int(spec["active_endpoint_index"]) != endpoint_index:
                raise RuntimeError("canonical active endpoint is not the last bead")
            endpoint_id = int(spec["active_endpoint_id"])
            endpoint_initial = _active_endpoint_position(task)
            endpoint_spawn_error_m = float(
                np.linalg.norm(endpoint_initial - staging_endpoint)
            )
            local_indices = endpoint_local_indices(
                len(task.cable_bead_IDs),
                endpoint_index,
                int(config["preparation"]["local_bead_count"]),
            )
            nominal_spacing = float(spec["nominal_spacing_m"])
            preparation_payload = _empty_canonical_spawn_payload(
                local_indices,
                staging_endpoint,
                spec["canonical_spawn_origin"],
            )
            metadata = {
                "seed": int(seed),
                "task": task_name,
                "config": config,
                "fixture_spec": spec,
                "fixture_frame_mode": frame_mode,
                "preparation_method": "canonical_spawn",
                "cable_spawn_mode": "canonical",
                "active_endpoint_index": int(endpoint_index),
                "active_endpoint_id": int(endpoint_id),
                "canonical_entry_center": spec["entry_center"],
                "canonical_insertion_axis": spec["insertion_axis"],
                "canonical_endpoint_target": staging_endpoint,
                "canonical_spawn_origin": spec["canonical_spawn_origin"],
                "nominal_spacing_m": nominal_spacing,
                "endpoint_spawn_error_m": endpoint_spawn_error_m,
                "staging_endpoint_target": staging_endpoint,
                "local_bead_indices": local_indices,
                "fixture_created_at_reset": False,
                "scientific_rollout_start": "common_snapshot",
                "preparation_status": None,
                "preparation_failure_reason": None,
                "canonical_spawn_geometry_pass": False,
                "local_geometry_pass": False,
                "after_grasp_pass": False,
                "fixture_overlap_free": False,
                "fixture_settle_contact_free": False,
                "common_snapshot_pass": False,
                "preparation_diagnostics": {},
                "simulator_git_commit": _simulator_commit(simulator_root),
                "branches": {},
            }

            def summarize_spawn(stage: str, diagnostics) -> None:
                _store_preparation_diagnostics(
                    preparation_payload, stage, diagnostics
                )
                metadata["preparation_diagnostics"][stage] = {
                    "minimum_entry_clearance_m": -float(
                        diagnostics["max_signed_entry_distance_m"]
                    ),
                    "median_alignment_cosine": float(
                        diagnostics["median_alignment_cosine"]
                    ),
                    "max_local_speed_mps": float(
                        diagnostics["max_local_speed_mps"]
                    ),
                    "max_local_spacing_error_m": float(
                        diagnostics["max_local_spacing_error_m"]
                    ),
                    "median_local_spacing_error_m": float(
                        diagnostics["median_local_spacing_error_m"]
                    ),
                }

            def fail_spawn(status: str, reason: str) -> Path:
                metadata["preparation_status"] = status
                metadata["preparation_failure_reason"] = reason
                metadata["fixture_spec"] = task.fixture_spec()
                _write_preparation_artifacts(
                    seed_root, metadata, preparation_payload
                )
                return seed_root

            initial_spawn = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
                nominal_spacing,
            )
            summarize_spawn("initial_spawn", initial_spawn)
            task.set_script_context("staging", [0.0, 0.0, 0.0])
            env.step_physics(
                int(config["preparation"]["settle_steps_after_spawn"])
            )
            after_spawn_settle = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
                nominal_spacing,
            )
            summarize_spawn("after_spawn_settle", after_spawn_settle)
            metadata["canonical_spawn_geometry_pass"] = local_geometry_passes(
                after_spawn_settle, config["preparation"]
            )
            metadata["local_geometry_pass"] = metadata[
                "canonical_spawn_geometry_pass"
            ]
            if not metadata["canonical_spawn_geometry_pass"]:
                return fail_spawn(
                    "PREP_FAILED_SPAWN_GEOMETRY",
                    "canonical geometry gate failed after spawn settle",
                )

            try:
                endpoint_id = _acquire_endpoint(env, task, config["motion"])
            except RuntimeError as exc:
                return fail_spawn("PREP_FAILED_GRASP", str(exc))
            env.step_physics(
                int(config["preparation"]["settle_steps_after_grasp"])
            )
            after_grasp = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
                nominal_spacing,
            )
            summarize_spawn("after_grasp", after_grasp)
            grasp_active = bool(getattr(env.ee, "activated", False))
            metadata["grasp_active"] = grasp_active
            metadata["after_grasp_pass"] = bool(
                grasp_active
                and local_geometry_passes(
                    after_grasp, config["preparation"]
                )
            )
            if not metadata["after_grasp_pass"]:
                return fail_spawn(
                    "PREP_FAILED_AFTER_GRASP",
                    "local geometry or suction gate failed after grasp",
                )

            if task.fixture_created():
                raise RuntimeError("fixture existed before explicit creation")
            task.create_fixture()
            overlap_contact = _fixture_contact_after_collision_refresh(task)
            metadata["fixture_overlap_contact"] = overlap_contact
            metadata["fixture_overlap_free"] = _contact_free(
                overlap_contact
            )
            if not metadata["fixture_overlap_free"]:
                return fail_spawn(
                    "PREP_FAILED_FIXTURE_OVERLAP",
                    "fixture creation produced immediate cable overlap",
                )

            env.step_physics(
                int(
                    config["preparation"][
                        "settle_steps_after_fixture_creation"
                    ]
                )
            )
            fixture_settle_contact = _fixture_contact_after_collision_refresh(
                task
            )
            after_fixture_settle = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
                nominal_spacing,
            )
            summarize_spawn("after_fixture_settle", after_fixture_settle)
            metadata["fixture_settle_contact"] = fixture_settle_contact
            metadata["fixture_settle_contact_free"] = _contact_free(
                fixture_settle_contact
            )
            if not metadata["fixture_settle_contact_free"]:
                return fail_spawn(
                    "PREP_FAILED_FIXTURE_SETTLE_CONTACT",
                    "fixture contact appeared during fixture settle",
                )

            metadata["preparation_status"] = "PASS"
            metadata["common_snapshot_pass"] = True
            metadata["fixture_spec"] = task.fixture_spec()
            metadata["staging_fixture_contact"] = fixture_settle_contact
            metadata["staging_contact"] = fixture_settle_contact
            metadata["staging_endpoint_actual"] = _active_endpoint_position(
                task
            )
            metadata["active_endpoint_id"] = int(endpoint_id)
            _write_preparation_artifacts(
                seed_root, metadata, preparation_payload
            )
        elif local_preparation:
            spec = task.fixture_spec()
            if bool(spec["fixture_created"]) or spec["fixture_ids"]:
                raise RuntimeError("fixture was not deferred during R1.1 reset")
            insertion_axis = np.asarray(
                spec["insertion_axis"], dtype=np.float64
            )
            lateral_axis = np.asarray(
                spec["lateral_axis"], dtype=np.float64
            )
            entry_center = np.asarray(
                spec["entry_center"], dtype=np.float64
            )
            staging_endpoint = np.asarray(
                entry_center, dtype=np.float64
            ) - float(config["motion"]["staging_before_entry_m"]) * insertion_axis
            chosen_index, endpoint_id, chosen_endpoint_initial = (
                choose_active_endpoint_for_staging(
                    task, staging_endpoint[:2]
                )
            )
            task.set_active_endpoint_index(chosen_index)
            spec = task.fixture_spec()
            canonicalization = canonicalize_local_segment(
                task,
                chosen_index,
                staging_endpoint,
                insertion_axis,
                int(config["preparation"]["local_bead_count"]),
            )
            local_indices = canonicalization["indices"]
            preparation_payload = _empty_preparation_payload(
                local_indices, canonicalization["targets"]
            )
            metadata = {
                "seed": int(seed),
                "task": task_name,
                "config": config,
                "fixture_spec": spec,
                "fixture_frame_mode": frame_mode,
                "canonical_entry_center": spec["entry_center"],
                "canonical_insertion_axis": spec["insertion_axis"],
                "chosen_endpoint_index": int(chosen_index),
                "chosen_endpoint_initial_position": chosen_endpoint_initial,
                "active_endpoint_id": int(endpoint_id),
                "canonical_staging_target": staging_endpoint,
                "staging_endpoint_target": staging_endpoint,
                "local_bead_indices": local_indices,
                "preparation_status": None,
                "preparation_failure_reason": None,
                "local_geometry_pass": False,
                "after_grasp_pass": False,
                "fixture_overlap_free": False,
                "fixture_settle_contact_free": False,
                "common_snapshot_pass": False,
                "preparation_diagnostics": {},
                "simulator_git_commit": _simulator_commit(simulator_root),
                "branches": {},
            }

            def summarize(stage: str, diagnostics) -> None:
                _store_preparation_diagnostics(
                    preparation_payload, stage, diagnostics
                )
                metadata["preparation_diagnostics"][stage] = {
                    "minimum_entry_clearance_m": -float(
                        diagnostics["max_signed_entry_distance_m"]
                    ),
                    "median_alignment_cosine": float(
                        diagnostics["median_alignment_cosine"]
                    ),
                    "max_local_speed_mps": float(
                        diagnostics["max_local_speed_mps"]
                    ),
                }

            def fail(status: str, reason: str) -> Path:
                metadata["preparation_status"] = status
                metadata["preparation_failure_reason"] = reason
                metadata["fixture_spec"] = task.fixture_spec()
                _write_preparation_artifacts(
                    seed_root, metadata, preparation_payload
                )
                return seed_root

            env.step_physics(
                int(config["preparation"]["settle_steps_before_grasp"])
            )
            initial_diagnostics = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
            )
            summarize("after_initial_settle", initial_diagnostics)
            metadata["local_geometry_pass"] = local_geometry_passes(
                initial_diagnostics, config["preparation"]
            )
            if not metadata["local_geometry_pass"]:
                return fail(
                    "PREP_FAILED_LOCAL_GEOMETRY",
                    "local geometry gate failed after initial settle",
                )

            try:
                endpoint_id = _acquire_endpoint(
                    env, task, config["motion"]
                )
            except RuntimeError as exc:
                return fail("PREP_FAILED_GRASP", str(exc))
            env.step_physics(
                int(config["preparation"]["settle_steps_after_grasp"])
            )
            after_grasp = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
            )
            summarize("after_grasp", after_grasp)
            grasp_active = bool(getattr(env.ee, "activated", False))
            metadata["grasp_active"] = grasp_active
            metadata["after_grasp_pass"] = bool(
                grasp_active
                and local_geometry_passes(
                    after_grasp, config["preparation"]
                )
            )
            if not metadata["after_grasp_pass"]:
                return fail(
                    "PREP_FAILED_AFTER_GRASP",
                    "local geometry or suction gate failed after grasp",
                )

            if task.fixture_created():
                raise RuntimeError("fixture existed before explicit creation")
            task.create_fixture()
            overlap_contact = _fixture_contact_after_collision_refresh(task)
            metadata["fixture_overlap_contact"] = overlap_contact
            metadata["fixture_overlap_free"] = _contact_free(
                overlap_contact
            )
            if not metadata["fixture_overlap_free"]:
                return fail(
                    "PREP_FAILED_FIXTURE_OVERLAP",
                    "fixture creation produced immediate cable overlap",
                )

            env.step_physics(
                int(
                    config["preparation"][
                        "settle_steps_after_fixture_creation"
                    ]
                )
            )
            fixture_settle_contact = _fixture_contact_after_collision_refresh(
                task
            )
            after_fixture_settle = local_segment_diagnostics(
                task,
                local_indices,
                entry_center,
                insertion_axis,
                lateral_axis,
            )
            summarize("after_fixture_settle", after_fixture_settle)
            metadata["fixture_settle_contact"] = fixture_settle_contact
            metadata["fixture_settle_contact_free"] = _contact_free(
                fixture_settle_contact
            )
            final_local_pass = local_geometry_passes(
                after_fixture_settle, config["preparation"]
            )
            grasp_active = bool(getattr(env.ee, "activated", False))
            if not (
                metadata["fixture_settle_contact_free"]
                and final_local_pass
                and grasp_active
            ):
                return fail(
                    "PREP_FAILED_FIXTURE_SETTLE_CONTACT",
                    "fixture-settle contact, local geometry, or suction gate failed",
                )

            metadata["preparation_status"] = "PASS"
            metadata["common_snapshot_pass"] = True
            metadata["fixture_spec"] = task.fixture_spec()
            metadata["staging_fixture_contact"] = fixture_settle_contact
            metadata["staging_contact"] = fixture_settle_contact
            metadata["staging_endpoint_actual"] = _active_endpoint_position(
                task
            )
            metadata["active_endpoint_id"] = int(endpoint_id)
            _write_preparation_artifacts(
                seed_root, metadata, preparation_payload
            )
        else:
            chosen_endpoint_initial = None
            task.set_fixture_collision_enabled(False)
            if frame_mode == "canonical":
                spec = task.fixture_spec()
                insertion_axis = np.asarray(
                    spec["insertion_axis"], dtype=np.float64
                )
                staging_endpoint = np.asarray(
                    spec["entry_center"], dtype=np.float64
                ) - float(config["motion"]["staging_before_entry_m"]) * insertion_axis
                chosen_index, _, chosen_endpoint_initial = (
                    choose_active_endpoint_for_staging(
                        task, staging_endpoint[:2]
                    )
                )
                task.set_active_endpoint_index(chosen_index)
                spec = task.fixture_spec()
                endpoint_id = _acquire_endpoint(env, task, config["motion"])
                for _ in range(2):
                    if not _move_endpoint_to(
                        env,
                        task,
                        staging_endpoint,
                        float(config["motion"]["staging_speed_mps"]),
                    ):
                        raise RuntimeError("common staging stretch failed")
            else:
                endpoint_id = _acquire_endpoint(env, task, config["motion"])
                for _ in range(2):
                    task.reposition_fixture_from_active_endpoint()
                    spec = task.fixture_spec()
                    insertion_axis = np.asarray(
                        spec["insertion_axis"], dtype=np.float64
                    )
                    staging_endpoint = np.asarray(
                        spec["entry_center"], dtype=np.float64
                    ) - float(
                        config["motion"]["staging_before_entry_m"]
                    ) * insertion_axis
                    if not _move_endpoint_to(
                        env,
                        task,
                        staging_endpoint,
                        float(config["motion"]["staging_speed_mps"]),
                    ):
                        raise RuntimeError("common staging stretch failed")
            lateral_axis = np.asarray(
                spec["lateral_axis"], dtype=np.float64
            )
            task.set_script_context("staging", [0.0, 0.0, 0.0])
            task.set_fixture_collision_enabled(True)
            env.step_physics(
                int(config["motion"]["pre_snapshot_settle_steps"])
            )
            staging_contact = task.fixture_contact_observation()
            staging_endpoint_actual = _active_endpoint_position(task)
            metadata = {
                "seed": int(seed),
                "task": task_name,
                "config": config,
                "fixture_spec": spec,
                "active_endpoint_id": endpoint_id,
                "fixture_frame_mode": frame_mode,
                "canonical_entry_center": (
                    spec["entry_center"]
                    if frame_mode == "canonical"
                    else None
                ),
                "canonical_insertion_axis": (
                    spec["insertion_axis"]
                    if frame_mode == "canonical"
                    else None
                ),
                "chosen_endpoint_index": spec["active_endpoint_index"],
                "chosen_endpoint_initial_position": chosen_endpoint_initial,
                "canonical_staging_target": (
                    staging_endpoint if frame_mode == "canonical" else None
                ),
                "staging_endpoint_target": staging_endpoint,
                "staging_endpoint_actual": staging_endpoint_actual,
                "staging_fixture_contact": staging_contact,
                "staging_contact": staging_contact,
                "simulator_git_commit": _simulator_commit(simulator_root),
                "branches": {},
            }
            if not _contact_free(staging_contact):
                metadata["preparation_status"] = "PREP_FAILED"
                metadata["preparation_failure_reason"] = (
                    "fixture_contact_at_staging"
                )
                with (seed_root / "metadata.json").open(
                    "w", encoding="utf-8"
                ) as handle:
                    json.dump(
                        _json_ready(metadata),
                        handle,
                        indent=2,
                        sort_keys=True,
                    )
                    handle.write("\n")
                return seed_root
            metadata["preparation_status"] = "COMPLETED"

        runtime_state = capture_runtime_state(env)
        common_state = capture_world_state(env, task)
        endpoint_at_snapshot = _active_endpoint_position(task)
        state_id = p.saveState()
        targets = _branch_targets(
            config,
            staging_endpoint,
            insertion_axis,
            lateral_axis,
            common_state,
            endpoint_at_snapshot,
        )

        np.savez_compressed(seed_root / "common_state.npz", **common_state)
        metadata["nominal_repeat_targets_identical"] = True
        for branch_name, offset_key in BRANCH_OFFSETS.items():
            offset = float(config["motion"][offset_key])
            payload, details = _run_branch(
                env=env,
                task=task,
                state_id=state_id,
                runtime_state=runtime_state,
                common_state=common_state,
                branch_name=branch_name,
                offset=offset,
                targets=targets[branch_name],
                config=config,
            )
            branch_root = seed_root / branch_name
            branch_root.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(branch_root / "trajectory.npz", **payload)
            metadata["branches"][branch_name] = details

        with (seed_root / "metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(_json_ready(metadata), handle, indent=2, sort_keys=True)
            handle.write("\n")
        return seed_root
    finally:
        if state_id is not None:
            try:
                p.removeState(state_id)
            except Exception:
                pass
        env.stop()


def run_seeds(
    config: Mapping[str, Any], seeds: Iterable[int], repo_root: Path
) -> None:
    failures = []
    for seed in seeds:
        try:
            seed_root = run_seed(config, int(seed), repo_root)
            with (seed_root / "metadata.json").open(
                "r", encoding="utf-8"
            ) as handle:
                preparation_status = json.load(handle).get(
                    "preparation_status", "COMPLETED"
                )
            if preparation_status in ("COMPLETED", "PASS"):
                print(f"completed seed {int(seed)}: {seed_root}", flush=True)
            else:
                print(
                    f"prep failed seed {int(seed)}: {seed_root}", flush=True
                )
        except Exception as exc:
            failures.append((int(seed), str(exc)))
            print(f"failed seed {int(seed)}: {exc}", flush=True)
    if failures:
        raise RuntimeError(f"Phase 0B seed failures: {failures}")
