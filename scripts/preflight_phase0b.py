#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from slncde.phase0a.runner import load_simulator
from slncde.phase0b.config import load_config
from slncde.phase0b.runner import set_fixture_environment


def main() -> None:
    parser = argparse.ArgumentParser(description="Preflight the Phase 0B fixture")
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    seed = int(config["experiment"]["seeds"][0])
    set_fixture_environment(config, seed)
    print(f"CUDA_VISIBLE_DEVICES={os.environ['CUDA_VISIBLE_DEVICES']!r}")

    import pybullet as p

    _, tasks, Environment = load_simulator(REPO_ROOT)
    task_name = str(config["simulator"]["task_name"])
    if task_name not in tasks.names:
        raise RuntimeError(f"task registry is missing {task_name}")
    env = Environment(
        disp=False,
        hz=int(config["simulator"]["hz"]),
        deterministic=True,
        control_substeps=1,
        post_action_settle_steps=0,
    )
    try:
        task = tasks.names[task_name]()
        env.reset(task)
        spec = task.fixture_spec()
        if len(spec["fixture_ids"]) != 2:
            raise RuntimeError("fixture does not contain two wall bodies")
        if spec["active_endpoint_id"] is None:
            raise RuntimeError("fixture task has no active endpoint")
        for name in ("entry_center", "channel_exit"):
            x, y = spec[name][:2]
            if not (task.X_MIN <= x <= task.X_MAX and task.Y_MIN <= y <= task.Y_MAX):
                raise RuntimeError(f"{name} lies outside workspace")
        contact = task.fixture_contact_observation()
        if int(contact["contact_point_count"]) != 0:
            raise RuntimeError("initial fixture contact is not zero")
        wall_aabbs = [p.getAABB(int(body)) for body in spec["fixture_ids"]]
        for minimum, maximum in wall_aabbs:
            if not (
                task.X_MIN <= minimum[0] <= maximum[0] <= task.X_MAX
                and task.Y_MIN <= minimum[1] <= maximum[1] <= task.Y_MAX
            ):
                raise RuntimeError("canonical fixture wall lies outside workspace")
        entry = np.asarray(spec["entry_center"], dtype=np.float64)
        axis = np.asarray(spec["insertion_axis"], dtype=np.float64)
        staging = entry - float(
            config["motion"]["staging_before_entry_m"]
        ) * axis
        print(f"cable_bead_count={len(task.cable_bead_IDs)}")
        print(f"active_endpoint={spec['active_endpoint_id']}")
        print(f"fixture frame mode: {spec['fixture_frame_mode']}")
        print(f"entry_center={spec['entry_center']}")
        print(f"insertion_axis={spec['insertion_axis']}")
        print(f"lateral_axis={spec['lateral_axis']}")
        print(f"left_wall_AABB={wall_aabbs[0]}")
        print(f"right_wall_AABB={wall_aabbs[1]}")
        print(f"channel_gap_m={spec['channel_gap_m']}")
        print(f"staging_target_xy={staging[:2].tolist()}")
        print(f"fixture_ids={spec['fixture_ids']}")
        print("preflight=PASS")
    finally:
        env.stop()


if __name__ == "__main__":
    main()
