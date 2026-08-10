#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.metrics import analyze


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=ROOT / "configs" / "capability.yaml"
    )
    args = parser.parse_args()
    metrics = analyze(args.config)
    print(
        json.dumps(
            {
                "verdict": metrics["verdict"],
                "capability_matrix": metrics["capability_matrix"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
