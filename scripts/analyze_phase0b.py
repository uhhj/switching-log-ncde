#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from slncde.phase0b.config import load_config
from slncde.phase0b.metrics import analyze_experiment
from slncde.phase0b.report import write_reports


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze existing Phase 0B data")
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    result = analyze_experiment(REPO_ROOT, config)
    result_path = write_reports(result, config, REPO_ROOT)
    print(result["verdict"])
    print(f"report={result_path}")


if __name__ == "__main__":
    main()
