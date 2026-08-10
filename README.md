# Switching Log-NCDE

This repository is currently qualifying a CPU-only cable constrained-passage
benchmark before any predictive model is trained.

## Scientific status

- Phase0A hidden friction: NO-GO.
- Phase0B open channel: NO-GO.
- Phase0B-T2: NO-GO. Contact is real and repeatable, but stick/slip/jam were
  not established.
- Current: Phase0C0 contact-regime capability probe, separating task
  representation failure, Oracle definition failure, and simulator contact
  capability failure.

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
python scripts/run_phase0c0_contact_capability.py --config configs/phase0c0_contact_capability.yaml --preflight
python scripts/run_phase0c0_contact_capability.py --config configs/phase0c0_contact_capability.yaml
```

## Current simulator path

Phase 0C0 established that PyBullet contact extraction and Oracle labels can
recognize stick/slip/jam, while the DeformableRavens bead-chain task could not
realize them reliably. Phase 0S-MJ then demonstrated free/stick/slip/jam
capability with MuJoCo 1D flex.

Current phase: Phase 0M, a single MuJoCo flex constrained-passage task. Do not
expand DeformableRavens as a parallel primary simulator. Do not train dynamics
models until Phase 0M and the subsequent matched-state/matched-action audit
pass.

## Phase 0M-C1

The original Phase 0M fixed-duration rollout had insufficient geometric
reachability for the leading-4 passage criterion. C1 changed only the
longitudinal drive-distance / horizon protocol; geometry, controller speed,
branch offsets, friction and Oracle definitions remained unchanged. The
corrected batch reached the constrained region but remained a task NO-GO.
