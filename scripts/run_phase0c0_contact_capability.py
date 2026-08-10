#!/usr/bin/env python3
import argparse
from pathlib import Path

from slncde.phase0.capability_probe import run_capability_probe, run_preflight


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Phase 0C0 contact capability probe")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--preflight", action="store_true")
    args = parser.parse_args()
    if args.preflight:
        result = run_preflight(args.config)
        if result["status"] != "PASS":
            raise SystemExit(1)
    else:
        run_capability_probe(args.config)


if __name__ == "__main__":
    main()
