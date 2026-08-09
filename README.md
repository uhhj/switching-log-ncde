# Switching Log-NCDE

This repository is currently qualifying a CPU-only cable constrained-passage
benchmark before any predictive model is trained.

## Scientific status

- Phase0A hidden friction: NO-GO.
- Phase0B open channel: NO-GO.
- Current: Phase0B-T2 constriction passage qualification.

The active task uses constraint-consistent canonical cable spawn, a converging
funnel and a narrow throat. Four deterministic branches start from one common
PyBullet snapshot and are labeled offline from native contact, force,
tangential velocity and progress signals.

```bash
export CUDA_VISIBLE_DEVICES=""
pytest -q tests/phase0b
python scripts/preflight_phase0b.py --config configs/phase0b_t2_constriction.yaml
python scripts/run_phase0b.py --config configs/phase0b_t2_constriction.yaml --seeds 83001 83002 83003 83004 83005
python scripts/analyze_phase0b.py --config configs/phase0b_t2_constriction.yaml
```
