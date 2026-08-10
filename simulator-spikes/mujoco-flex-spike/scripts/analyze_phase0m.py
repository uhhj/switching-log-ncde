#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.metrics import analyze_phase0m


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    metrics = analyze_phase0m(args.config)
    print(
        json.dumps(
            {"verdict": metrics["verdict"], "gates": metrics["gates"]},
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
