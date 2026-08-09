# Switching Log-NCDE

This repository currently contains only the Phase 0A causal-pair smoke test. It asks whether the same PyBullet snapshot and the same future robot motion produce reproducibly different cable futures when only a hidden contact-friction condition changes.

The experiment runs three branches per seed: `free`, `high_friction` (the simulator condition `hidden_high_friction`), and `free_repeat`. The same-condition repeat estimates the deterministic noise floor. No NCDE, mode classifier, MPC, or RL code belongs here until this gate passes.

## CPU-only smoke

```bash
export CUDA_VISIBLE_DEVICES=""
python3 -m pip install -e ".[dev]"
pytest -q tests/phase0a
python3 scripts/preflight_cpu.py
python3 scripts/run_phase0a.py --config configs/phase0a_smoke.yaml --seeds 81001
python3 scripts/analyze_phase0a.py --config configs/phase0a_smoke.yaml
python3 scripts/run_phase0a.py --config configs/phase0a_smoke.yaml --seeds 81002 81003 81004 81005
python3 scripts/analyze_phase0a.py --config configs/phase0a_smoke.yaml
```

The simulator is supplied by the `external/deformable-ravens` submodule. Its tested PyBullet requirement applies; the main package intentionally does not pin PyBullet.

## Scientific phase status

- Phase 0A: NO-GO for artificial hidden-friction mechanism.
- Phase 0B: real fixture contact and low repeat noise on completed seeds.
- Phase 0B-R1: canonical fixture placement solved workspace placement, but
  endpoint-only staging failed to control adjacent cable geometry.
- Phase 0B-R1.1: local bead teleport was incompatible with the cable constraints.
- Current: Phase 0B-R1.2 constraint-consistent canonical cable spawn.

Local-segment canonicalization is benchmark initial-state generation. It is
applied before the common snapshot and identically for all future branches.
No cable teleportation or state reset is allowed after the common snapshot.
R1.2 creates the whole bead chain and its point-to-point constraints directly
in the canonical pre-insertion geometry; it does not rearrange an existing cable.
