#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from slncde.phase0b.config import load_config
from slncde.phase0b.runner import run_seeds


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Phase 0B fixture branches")
    parser.add_argument("--config", required=True)
    parser.add_argument("--seeds", nargs="+", type=int)
    args = parser.parse_args()
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    config = load_config(args.config)
    seeds = args.seeds if args.seeds is not None else config["experiment"]["seeds"]
    run_seeds(config, seeds, REPO_ROOT)


if __name__ == "__main__":
    main()
