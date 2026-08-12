# Phase 0S-SOFA Stage B Reaction-Proxy Contamination Audit

## Verdict

PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CLEAN

## Runtime

- SOFA: official v26.06.00 Linux binary
- BeamAdapter / SofaPython3: release-bundled v26.06
- NativeContactBridge: full-path loaded; source unchanged
- CPU-only: yes

## Frozen setup

- Seed / dt / steps: 84000 / 0.002 s / 300
- Normal endpoint load: -0.001 N along Y after 100 ms
- Tail: node 0 translation fixed, rotation free; `fixedDirections=[1,1,1,0,0,0]`
- Sentinel plane: y=-1.0 m

## Native-contact control

- Bridge-ready frames: 300 / 300
- Frame serial min/max: 1 / 300
- Native contact total / measurement / max-frame: 0 / 0 / 0
- Minimum centerline-to-sentinel distance: 0.879911765387 m

## LCP reaction audit

- Vector size max all / measurement: 0 / 0
- Nonempty frames all / measurement: 0 / 0
- Reaction p50 / p90 / p99 / max: 0.0 / 0.0 / 0.0 / 0.0

## Control validity

- Tail translation max: 0 m
- Tip displacement max: 0.183654946577 m
- Load-active fraction during measurement: 1.0
- All bridge, serial, native-contact, sentinel, tail, load, vector-empty, and exact-zero checks: PASS

## Fact

NativeContactBridge verified the anchored control remained collision-free while
`LCPConstraintSolver.constraintForces` was inspected directly.

## Inference

The translation-only projective tail anchor did not directly contaminate the
current global LCP reaction proxy in this no-contact control.

## Unknown

Real wall-contact reaction, stable contact, breakaway, stick/slip, jam, Oracle
calibration, held-out capability, and unified passage remain untested.

## Next action

Run Stage C stable normal-contact construction with NativeContactBridge as
formal contact identity and test whether real wall contact produces sustained
nonzero reaction proxy.
