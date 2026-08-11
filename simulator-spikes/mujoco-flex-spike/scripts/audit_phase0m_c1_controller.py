#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.passage import PassageScene  # noqa: E402
from src.phase0m import _load_yaml, resolve_phase0m  # noqa: E402


def _git(*args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(ROOT), *args], text=True
    ).strip()


def _episode_durations(mask: np.ndarray, timestep_s: float) -> np.ndarray:
    values = np.asarray(mask, dtype=bool)
    starts = np.flatnonzero(values & np.r_[True, ~values[:-1]])
    ends = np.flatnonzero(values & np.r_[~values[1:], True])
    return (ends - starts + 1).astype(np.float64) * float(timestep_s)


def _event_counts(trace: Dict[str, np.ndarray]) -> Dict[str, int]:
    names = np.asarray(trace["event_names"]).astype(str)
    events = np.asarray(trace["events"], dtype=bool)
    return {
        name: int(np.count_nonzero(events[:, index]))
        for index, name in enumerate(names)
    }


def audit(config_path: Path) -> Dict[str, Any]:
    spike_root, config, thresholds = resolve_phase0m(config_path)
    scene = PassageScene(
        spike_root / str(config["simulation"]["source_model"]), config
    )
    model = scene.simulator.model
    endpoint_body = scene.simulator.endpoint_body_id
    first_joint = int(model.body_jntadr[endpoint_body])
    joint_ids = list(
        range(first_joint, first_joint + int(model.body_jntnum[endpoint_body]))
    )
    mapping = {
        "body_id": endpoint_body,
        "body_name": mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, endpoint_body
        ),
        "joint_ids": joint_ids,
        "dof_addresses": [int(model.jnt_dofadr[j]) for j in joint_ids],
        "joint_axes": [model.jnt_axis[j].tolist() for j in joint_ids],
        "actuator_count": int(model.nu),
        "force_path": "FlexSimulator.step -> data.xfrc_applied[endpoint_body, :3]",
    }
    phase0s = _load_yaml(spike_root / "configs" / "capability.yaml")
    timestep = float(config["simulation"]["timestep_s"])
    minimum_dwell_s = float(thresholds["minimum_mode_duration_ms"]) / 1000.0
    data_root = spike_root / str(config["paths"]["report_root"]) / "data"
    centered_rows = []
    branch_rows: Dict[str, List[Dict[str, Any]]] = {
        "offset_medium": [],
        "offset_large": [],
    }
    all_contact_episodes: List[float] = []
    event_totals = {"touch": 0, "release": 0}
    for seed_value in config["experiment"]["seeds"]:
        seed = int(seed_value)
        seed_root = data_root / f"seed_{seed}"
        with np.load(seed_root / "common_state.npz") as common:
            vertices0 = np.asarray(
                common["ordered_vertex_positions"], dtype=np.float64
            )
            head_index = int(np.asarray(common["head_index"]).item())
            leading = np.asarray(common["leading_indices"], dtype=np.int64)
        start_head_x = float(vertices0[head_index, 0])
        start_leading_mean_x = float(np.mean(vertices0[leading, 0]))
        start_leading_min_x = float(np.min(vertices0[leading, 0]))
        for branch in ("centered", "offset_medium", "offset_large"):
            with np.load(seed_root / f"{branch}.npz") as loaded:
                trace = {key: loaded[key] for key in loaded.files}
            reference_x = start_head_x + np.asarray(
                trace["reference_travel"], dtype=np.float64
            )
            head_x = np.asarray(trace["head_position"], dtype=np.float64)[:, 0]
            leading_positions = np.asarray(
                trace["ordered_vertex_positions"], dtype=np.float64
            )[:, leading, 0]
            if branch == "centered":
                centered_rows.append(
                    {
                        "seed": seed,
                        "reference_x_travel_m": float(
                            reference_x[-1] - reference_x[0]
                        ),
                        "head_x_travel_m": float(head_x[-1] - start_head_x),
                        "leading4_mean_x_travel_m": float(
                            np.mean(leading_positions[-1])
                            - start_leading_mean_x
                        ),
                        "leading4_min_x_travel_m": float(
                            np.min(leading_positions[-1]) - start_leading_min_x
                        ),
                        "final_reference_head_error_m": float(
                            reference_x[-1] - head_x[-1]
                        ),
                        "max_reference_head_error_m": float(
                            np.max(reference_x - head_x)
                        ),
                    }
                )
                continue
            raw_contact = np.asarray(trace["fixture_contact_count"]) > 0
            durations = _episode_durations(raw_contact, timestep)
            all_contact_episodes.extend(durations.tolist())
            events = _event_counts(trace)
            event_totals["touch"] += events.get("touch", 0)
            event_totals["release"] += events.get("release", 0)
            contact_y_error = (
                float(np.asarray(trace["y_target_m"]).item())
                - np.asarray(trace["head_position"], dtype=np.float64)[:, 1]
            )[raw_contact]
            endpoint_vy = np.asarray(
                trace["ordered_vertex_velocities"], dtype=np.float64
            )[:, head_index, 1][raw_contact]
            normal = np.asarray(trace["normal_force_sum"], dtype=np.float64)[
                raw_contact
            ]
            branch_rows[branch].append(
                {
                    "seed": seed,
                    "median_reference_y_minus_endpoint_y_m": float(
                        np.median(contact_y_error)
                    )
                    if contact_y_error.size
                    else 0.0,
                    "median_endpoint_vy_mps": float(np.median(endpoint_vy))
                    if endpoint_vy.size
                    else 0.0,
                    "median_normal_force_n": float(np.median(normal))
                    if normal.size
                    else 0.0,
                    "max_contact_dwell_ms": float(
                        1000.0 * np.max(durations)
                    )
                    if durations.size
                    else 0.0,
                    "contact_episode_count": int(durations.size),
                    "touch": events.get("touch", 0),
                    "release": events.get("release", 0),
                }
            )
    durations = np.asarray(all_contact_episodes, dtype=np.float64)
    short_count = int(np.count_nonzero(durations < minimum_dwell_s))
    result = {
        "source": {
            "branch": _git("branch", "--show-current"),
            "commit": _git("rev-parse", "HEAD"),
            "config": str(Path(config_path).resolve().relative_to(spike_root)),
        },
        "endpoint_mapping": mapping,
        "old_controller": {
            "type": "moving Cartesian position-reference PD force servo",
            "kp_npm": float(config["controller"]["position_kp_npm"]),
            "kd_ns_per_m": float(
                config["controller"]["velocity_kd_ns_per_m"]
            ),
            "vector_norm_force_limit_n": float(
                config["controller"]["force_limit_n"]
            ),
            "force_application": mapping["force_path"],
            "telemetry": (
                "applied_endpoint_force and reference_travel are saved; x_ref is "
                "reconstructable and y_target is saved; no actuator force exists"
            ),
        },
        "phase0s": {
            "force_application": mapping["force_path"],
            "stick_command_force_n": float(phase0s["probe"]["stick_force_n"]),
            "slip_command_force_n": float(phase0s["probe"]["slip_force_n"]),
            "jam_command_force_n": float(phase0s["probe"]["jam_force_n"]),
        },
        "centered": centered_rows,
        "contact_branches": branch_rows,
        "chatter": {
            "contact_episode_count": int(durations.size),
            "median_contact_episode_duration_ms": float(
                1000.0 * np.median(durations)
            )
            if durations.size
            else 0.0,
            "frozen_minimum_dwell_ms": 1000.0 * minimum_dwell_s,
            "episodes_shorter_than_minimum": short_count,
            "short_contact_fraction": short_count / max(int(durations.size), 1),
            "touch": event_totals["touch"],
            "release": event_totals["release"],
        },
    }
    report_root = spike_root / "reports" / "phase0m_c2"
    report_root.mkdir(parents=True, exist_ok=True)
    centered_table = "\n".join(
        "| {seed} | {reference_x_travel_m:.9f} | {head_x_travel_m:.9f} | "
        "{leading4_mean_x_travel_m:.9f} | {leading4_min_x_travel_m:.9f} | "
        "{final_reference_head_error_m:.9f} | {max_reference_head_error_m:.9f} |".format(
            **row
        )
        for row in centered_rows
    )
    branch_lines = []
    for branch, rows in branch_rows.items():
        branch_lines.append(
            f"- {branch}: median y error while in raw contact "
            f"{np.median([r['median_reference_y_minus_endpoint_y_m'] for r in rows]):.9f} m; "
            f"median endpoint vy {np.median([r['median_endpoint_vy_mps'] for r in rows]):.9f} m/s; "
            f"median normal force {np.median([r['median_normal_force_n'] for r in rows]):.9f} N; "
            f"contact episodes {sum(r['contact_episode_count'] for r in rows)}; "
            f"touch/release {sum(r['touch'] for r in rows)}/{sum(r['release'] for r in rows)}"
        )
    text = f"""# Phase 0M-C1 Controller Audit

## Source
- branch: {result['source']['branch']}
- commit: {result['source']['commit']}
- config: {result['source']['config']}

## Endpoint mapping
- body: {mapping['body_name']} (id {mapping['body_id']})
- joint ids: {mapping['joint_ids']}
- DOF addresses: {mapping['dof_addresses']}
- joint axes: {mapping['joint_axes']}
- actuators: {mapping['actuator_count']}
- force path: {mapping['force_path']}

## Old Phase 0M/C1 controller
- implementation: moving Cartesian position-reference PD force servo
- Kp/Kd: {result['old_controller']['kp_npm']} N/m / {result['old_controller']['kd_ns_per_m']} N s/m
- force limit: {result['old_controller']['vector_norm_force_limit_n']} N vector norm
- telemetry: {result['old_controller']['telemetry']}

## Phase 0S force source
- application: {result['phase0s']['force_application']}
- stick/slip/jam command force: {result['phase0s']['stick_command_force_n']} / {result['phase0s']['slip_command_force_n']} / {result['phase0s']['jam_command_force_n']} N

## Centered tracking

| seed | reference x travel | head x travel | leading4 mean travel | leading4 min travel | final ref-head error | max ref-head error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
{centered_table}

## Medium / large contact
{chr(10).join(branch_lines)}

## Chatter
- raw contact episode count: {result['chatter']['contact_episode_count']}
- median raw contact episode duration: {result['chatter']['median_contact_episode_duration_ms']:.3f} ms
- frozen minimum dwell: {result['chatter']['frozen_minimum_dwell_ms']:.3f} ms
- episodes shorter than minimum: {result['chatter']['episodes_shorter_than_minimum']}
- short-contact fraction: {result['chatter']['short_contact_fraction']:.9f}
- Oracle touch/release: {result['chatter']['touch']} / {result['chatter']['release']}

## Fact
C1 used a real endpoint force path rather than qpos teleportation, but coupled x/y PD tracking under one vector-norm cap and activated the lateral target from the start. Almost every raw fixture-contact episode was shorter than the frozen dwell requirement.

## Inference
C2 can cleanly test per-axis bounded impedance with geometry-triggered lateral activation and one shared lateral cap, while preserving the C1 references and all task/contact variables.

## Action
Run the single frozen C2 controller correction without parameter tuning.
"""
    (report_root / "CONTROLLER_AUDIT.md").write_text(text, encoding="utf-8")
    print(
        f"controller audit: episodes={result['chatter']['contact_episode_count']}, "
        f"median={result['chatter']['median_contact_episode_duration_ms']:.3f} ms, "
        f"short_fraction={result['chatter']['short_contact_fraction']:.9f}"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    audit(args.config)


if __name__ == "__main__":
    main()
