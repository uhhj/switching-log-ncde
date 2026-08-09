#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from slncde.phase0a.runner import load_simulator


def main() -> None:
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    os.environ["CCDA_DEFER_HIDDEN_FACTOR_ARMING"] = "1"
    os.environ["CCDA_HIDDEN_CONDITION"] = "free"
    os.environ["CCDA_FRICTION_MECHANISM"] = "native_segment"
    os.environ["CCDA_TRACE_STRIDE"] = "4"
    print(f"CUDA_VISIBLE_DEVICES={os.environ['CUDA_VISIBLE_DEVICES']!r}")

    import pybullet as p  # noqa: F401
    _, tasks, Environment = load_simulator(REPO_ROOT)

    task_name = "ccda-hidden-friction-cable"
    if task_name not in tasks.names:
        raise RuntimeError(f"task registry is missing {task_name}")
    env = Environment(disp=False, deterministic=True, hz=240)
    try:
        task = tasks.names[task_name]()
        env.reset(task)
        print(f"task={task_name}")
        print(f"bead_count={len(task.cable_bead_IDs)}")
        print(f"conditions={list(task.CONDITIONS)}")
        print(f"friction_mechanism={os.environ['CCDA_FRICTION_MECHANISM']}")
        print("preflight=ok")
    finally:
        env.stop()


if __name__ == "__main__":
    main()
