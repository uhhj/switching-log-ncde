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

## Current phase: Phase 0M-C2

C1 removed the fixed-horizon reachability confound, but the unified MuJoCo
flex passage task still showed short contact chatter and no sustained
stick/slip/jam.

C2 is the final MuJoCo task correction. Geometry, friction, forward
reference speed, offsets, solver and Oracle definitions stay frozen.
Only endpoint actuation changes from position-reference control to bounded
contact-compatible Cartesian impedance.

If C2 fails the original Phase 0M gates, no further MuJoCo passage tuning
is allowed; the next simulator path is SOFA BeamAdapter.

## Current simulator path: Phase 0S-SOFA

MuJoCo 1D flex demonstrated controlled free/stick/slip/jam capability,
but the unified constrained-passage task remained a final NO-GO after
reachability correction and the final controller comparison.

No further MuJoCo passage tuning is allowed.

The current Phase 0 task evaluates SOFA BeamAdapter. It first calibrates
simulator-specific numerical Oracle thresholds in controlled scenes,
validates held-out free/stick/slip/jam capability, and only then runs one
unified constrained-passage smoke.

Do not train dynamics models before this P0 task and the subsequent
matched-state/matched-action audit pass.

## Phase 0S-SOFA Native Contact Bridge R1-B

SofaPython3 v26.06 does not expose the C++ `ContactListener` getters used by
the revised controlled-contact audit. A minimal out-of-tree
`NativeContactBridge` exports existing narrow-phase `DetectionOutput` fields
through ordinary SOFA Data: native beam/fixture primitive IDs, contact points,
beam-outward normal, and raw detection value. It performs no collision
detection and uses no nearest-point or endpoint proxy. Only Stage A is
validated; controlled contact stages B-F remain unexecuted.

## Phase 0S-SOFA Stage B

The anchored-free control used the translation-only tail projective constraint
planned for controlled stick/slip, with the same 0.001 N normal endpoint load
and no native collision contact. `LCPConstraintSolver.constraintForces`
remained structurally empty and zero, so the tail does not directly contaminate
the current global LCP reaction proxy. This does not validate the proxy as a
physically correct wall-contact force; Stage C remains unexecuted.

## Phase 0S-SOFA Stage C V2

The earlier Stage-B configuration requested a 1 MPa Young modulus but did not
wire it into `BeamInterpolation`. Stage C V2 explicitly binds and reads back
`defaultYoungModulus` before its C0 material-matched baseline and C1 contact
trace. It also requires fresh bridge frames, direct `ConstantForceField` Data
readback, and timestamp-span dwell. C0 passed, while C1 was blocked because
its zero-load geometry already produced native contact; no contact-mechanics
gate was evaluated.

## Phase 0S-SOFA Stage C V2.1

Offline reconstruction of the frozen SOFA v26.06.00 Point/Triangle
`LocalMinDistance` value corrected the V2 interpretation: the 50 zero-load
native-detection frames were all proximity-only (+1 mm gap) and reaction-free.
The immutable C1 measurement window still has 0% native, geometric, and
load-bearing contact, below the frozen 90% occupancy and 300 ms dwell gates.
Stage C is therefore a contact-construction FAIL; no reaction, geometry, or
later capability stage was evaluated.
