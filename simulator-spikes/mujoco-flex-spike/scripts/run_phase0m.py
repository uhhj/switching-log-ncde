#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.phase0m import preflight, resolve_phase0m, run_seeds


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="*")
    parser.add_argument("--preflight", action="store_true")
    args = parser.parse_args()
    if args.preflight:
        result = preflight(args.config)
        if result["status"] != "PASS":
            raise SystemExit(1)
        return
    _, config, _ = resolve_phase0m(args.config)
    seeds = args.seeds or [int(value) for value in config["experiment"]["seeds"]]
    count = run_seeds(args.config, seeds)
    print(f"trajectories completed: {count}")


if __name__ == "__main__":
    main()
