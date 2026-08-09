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


BRANCH_OFFSETS = {
    "nominal": "nominal_lateral_offset_m",
    "nominal_repeat": "nominal_lateral_offset_m",
    "slide_probe": "slide_lateral_offset_m",
    "jam_probe": "jam_lateral_offset_m",
}


def set_fixture_environment(config: Mapping[str, Any], seed: int) -> None:
    fixture = config["fixture"]
    simulator = config["simulator"]
    values = {
        "CUDA_VISIBLE_DEVICES": "",
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
    }
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
        task.set_fixture_collision_enabled(False)
        endpoint_id = _acquire_endpoint(env, task, config["motion"])
        for _ in range(2):
            task.reposition_fixture_from_active_endpoint()
            spec = task.fixture_spec()
            insertion_axis = np.asarray(
                spec["insertion_axis"], dtype=np.float64
            )
            staging_endpoint = np.asarray(
                spec["entry_center"], dtype=np.float64
            ) - float(config["motion"]["staging_before_entry_m"]) * insertion_axis
            if not _move_endpoint_to(
                env,
                task,
                staging_endpoint,
                float(config["motion"]["staging_speed_mps"]),
            ):
                raise RuntimeError("common staging stretch failed")
        lateral_axis = np.asarray(spec["lateral_axis"], dtype=np.float64)
        task.set_script_context("staging", [0.0, 0.0, 0.0])
        task.set_fixture_collision_enabled(True)
        env.step_physics(int(config["motion"]["pre_snapshot_settle_steps"]))
        staging_contact = task.fixture_contact_observation()
        if int(staging_contact["contact_point_count"]) != 0:
            raise RuntimeError("common staging pose has fixture contact")

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

        seed_root = (
            repo_root
            / str(config["paths"]["data_root"])
            / f"seed_{int(seed)}"
        )
        seed_root.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(seed_root / "common_state.npz", **common_state)
        metadata = {
            "seed": int(seed),
            "task": task_name,
            "config": config,
            "fixture_spec": spec,
            "active_endpoint_id": endpoint_id,
            "staging_endpoint_target": staging_endpoint,
            "staging_endpoint_actual": endpoint_at_snapshot,
            "staging_contact": staging_contact,
            "simulator_git_commit": _simulator_commit(simulator_root),
            "branches": {},
        }
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
            print(f"completed seed {int(seed)}: {seed_root}", flush=True)
        except Exception as exc:
            failures.append((int(seed), str(exc)))
            print(f"failed seed {int(seed)}: {exc}", flush=True)
    if failures:
        raise RuntimeError(f"Phase 0B seed failures: {failures}")
