from __future__ import annotations

import json
import importlib
import os
import random
import subprocess
import sys
import types
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple

import numpy as np

from .snapshot import (
    capture_runtime_state,
    capture_world_state,
    restore_joint_control_state,
    restore_runtime_state,
)


BRANCH_CONDITIONS = (
    ("free", "free"),
    ("high_friction", "hidden_high_friction"),
    ("free_repeat", "free"),
)


def _set_experiment_environment(config: Mapping[str, Any], seed: int) -> None:
    simulator = config["simulator"]
    friction = config["friction"]
    values = {
        "CUDA_VISIBLE_DEVICES": "",
        "CCDA_DEFER_HIDDEN_FACTOR_ARMING": "1",
        "CCDA_HIDDEN_CONDITION": "free",
        "CCDA_VISIBLE_SEED": str(seed),
        "CCDA_PAIR_GROUP": f"phase0a_{seed}",
        "CCDA_FRICTION_MECHANISM": simulator["friction_mechanism"],
        "CCDA_TRACE_STRIDE": simulator["trace_stride"],
        "CCDA_NATIVE_BASE_LATERAL_FRICTION": friction["base_lateral_friction"],
        "CCDA_NATIVE_HIDDEN_LATERAL_FRICTION": friction[
            "hidden_lateral_friction"
        ],
        "CCDA_FRICTION_SELECTED_COUNT": friction["selected_count"],
        "CCDA_FRICTION_CENTER_RATIO": friction["center_ratio"],
    }
    for name, value in values.items():
        os.environ[name] = str(value)
    random.seed(seed)
    np.random.seed(seed)


def load_simulator(repo_root: Path):
    """Load only the simulator modules needed by the CPU-only smoke test."""
    simulator_root = repo_root / "external" / "deformable-ravens"
    if str(simulator_root) not in sys.path:
        sys.path.insert(0, str(simulator_root))
    if "ravens" not in sys.modules:
        package = types.ModuleType("ravens")
        package.__path__ = [str(simulator_root / "ravens")]
        package.__package__ = "ravens"
        sys.modules["ravens"] = package
    tasks = importlib.import_module("ravens.tasks")
    Environment = importlib.import_module("ravens.environment").Environment

    return simulator_root, tasks, Environment


def _simulator_commit(simulator_root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(simulator_root), "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def _acquire_endpoint(env, task, motion: Mapping[str, Any]) -> int:
    import pybullet as p

    active_id = int(task.cable_bead_IDs[-1])
    endpoint = np.asarray(
        p.getBasePositionAndOrientation(active_id)[0], dtype=np.float64
    )
    orientation = np.asarray(
        p.getLinkState(
            env.ur5, env.ee_tip_link, computeForwardKinematics=True
        )[1],
        dtype=np.float64,
    )
    approach = np.concatenate((endpoint + np.asarray([0.0, 0.0, 0.05]), orientation))
    contact = np.concatenate(
        (
            endpoint
            + np.asarray(
                [0.0, 0.0, float(motion["grasp_height_offset_m"])]
            ),
            orientation,
        )
    )
    speed = float(motion["acquisition_speed"])
    if not env.movep(approach.tolist(), speed=speed):
        raise RuntimeError("endpoint approach motion failed")
    if not env.movep(contact.tolist(), speed=speed):
        raise RuntimeError("endpoint contact motion failed")
    env.step_physics(4)
    env.ee.activate([active_id], [])
    env.step_physics(4)
    if not bool(getattr(env.ee, "activated", False)):
        raise RuntimeError("suction did not activate")
    return active_id


def _build_pull_target(
    common_state: Mapping[str, np.ndarray], pull_distance_m: float
) -> np.ndarray:
    beads = np.asarray(common_state["bead_positions"], dtype=np.float64)
    tangent = beads[-1, :2] - beads[-2, :2]
    norm = float(np.linalg.norm(tangent))
    if norm <= 1e-12:
        raise RuntimeError("endpoint cable tangent is degenerate")
    tangent /= norm
    target = np.concatenate(
        (
            np.asarray(common_state["ee_position"], dtype=np.float64),
            np.asarray(common_state["ee_orientation"], dtype=np.float64),
        )
    )
    target[:2] += float(pull_distance_m) * tangent
    return target


def _trace_payload(
    trace: Iterable[Mapping[str, Any]],
    initial: Mapping[str, np.ndarray],
    final: Mapping[str, np.ndarray],
) -> Dict[str, np.ndarray]:
    rows = list(trace)
    if not rows:
        raise RuntimeError("simulator returned an empty CCDA trace")
    fields = (
        "physics_step",
        "phase",
        "bead_positions",
        "bead_velocities",
        "ee_position",
        "contact_force_norm",
        "contact_max_force_norm",
        "contact_active_beads",
        "contact_mean_speed",
    )
    payload = {name: np.asarray([row[name] for row in rows]) for name in fields}
    for name, value in initial.items():
        payload[f"initial_{name}"] = np.asarray(value)
    for name, value in final.items():
        payload[f"final_{name}"] = np.asarray(value)
    return payload


def _run_branch(
    *,
    env,
    task,
    state_id: int,
    runtime_state: Mapping[str, Any],
    common_state: Mapping[str, np.ndarray],
    condition: str,
    pull_target: np.ndarray,
    motion: Mapping[str, Any],
) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    import pybullet as p

    p.restoreState(stateId=state_id)
    env.reset_ccda_runtime_after_restore()
    restore_runtime_state(env, runtime_state)
    restore_joint_control_state(env, common_state)

    task.reset_ccda_branch(condition)
    arm_info = task.arm_ccda_hidden_factor_after_settle()
    initial = capture_world_state(env, task)

    task.set_ccda_phase("no_action")
    env.step_physics(int(motion["no_action_steps"]))

    task.set_ccda_phase("main_pull")
    if not env.movep(pull_target.tolist(), speed=float(motion["pull_speed"])):
        raise RuntimeError("pull motion failed")

    task.set_ccda_phase("post_main")
    env.step_physics(int(motion["post_pull_steps"]))

    final = capture_world_state(env, task)
    payload = _trace_payload(task.ccda_trace(), initial, final)
    privileged = task.ccda_privileged_state()
    return payload, {"arm_info": arm_info, "privileged_state": privileged}


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


def run_pair(config: Mapping[str, Any], seed: int, repo_root: Path) -> Path:
    """Run and immediately persist one three-branch causal pair."""
    import pybullet as p

    _set_experiment_environment(config, seed)
    simulator_root, tasks, Environment = load_simulator(repo_root)
    simulator = config["simulator"]
    motion = config["motion"]
    task_name = str(simulator["task_name"])
    if task_name not in tasks.names:
        raise KeyError(f"task registry has no {task_name!r}")

    env = Environment(
        disp=False,
        hz=int(simulator["hz"]),
        deterministic=bool(simulator["deterministic"]),
        control_substeps=1,
        post_action_settle_steps=0,
    )
    state_id = None
    try:
        task = tasks.names[task_name]()
        env.reset(task)
        active_id = _acquire_endpoint(env, task, motion)
        env.step_physics(int(motion["pre_snapshot_settle_steps"]))
        runtime_state = capture_runtime_state(env)
        common_state = capture_world_state(env, task)
        pull_target = _build_pull_target(
            common_state, float(motion["pull_distance_m"])
        )
        state_id = p.saveState()

        pair_root = repo_root / str(config["paths"]["data_root"]) / f"pair_{seed}"
        branch_metadata: Dict[str, Any] = {}
        for branch_name, condition in BRANCH_CONDITIONS:
            payload, details = _run_branch(
                env=env,
                task=task,
                state_id=state_id,
                runtime_state=runtime_state,
                common_state=common_state,
                condition=condition,
                pull_target=pull_target,
                motion=motion,
            )
            branch_root = pair_root / branch_name
            branch_root.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(branch_root / "trajectory.npz", **payload)
            branch_metadata[branch_name] = {
                "condition": condition,
                **details,
            }

        selected = branch_metadata["high_friction"]["privileged_state"]
        metadata = {
            "seed": int(seed),
            "task": task_name,
            "config": config,
            "active_endpoint_bead_id": active_id,
            "pull_target": pull_target,
            "arm_info": {
                name: info["arm_info"] for name, info in branch_metadata.items()
            },
            "selected_bead_indices": selected["selected_local_indices"],
            "simulator_git_commit": _simulator_commit(simulator_root),
        }
        with (pair_root / "metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(_json_ready(metadata), handle, indent=2, sort_keys=True)
            handle.write("\n")
        return pair_root
    finally:
        if state_id is not None:
            try:
                p.removeState(state_id)
            except Exception:
                pass
        env.stop()


def run_pairs(
    config: Mapping[str, Any], seeds: Iterable[int], repo_root: Path
) -> None:
    for seed in seeds:
        pair_root = run_pair(config, int(seed), repo_root)
        print(f"completed seed {int(seed)}: {pair_root}", flush=True)
