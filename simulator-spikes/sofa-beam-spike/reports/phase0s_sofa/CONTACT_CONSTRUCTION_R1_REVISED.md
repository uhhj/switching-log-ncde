# Phase 0S-SOFA Controlled Contact Construction R1 Revised

## Verdict

PHASE0S_SOFA_ENGINEERING_BLOCKED

## Runtime

- SOFA: official v26.06.00 Linux binary
- BeamAdapter: official bundled plugin
- SofaPython3: official bundled plugin
- CPU-only: yes (`CUDA_VISIBLE_DEVICES=""`)

## Frozen setup

- Seed: 84000
- Beam: length 0.12 m, radius 0.002 m, 7 nodes, 0.02 m spacing, E=1e6 Pa, density=1000 kg/m3
- dt: 0.002 s; friction: 0.8; normal preload: 0.001 N
- Contact distance / initial clearance: 0.002 m / 0.001 m
- Planned dwell / progress window: 80 ms / 100 ms

## Native contact extraction

- Component instantiated: `ContactListener`
- Collision primitive: `PointCollisionModel`
- 20-step planar wall-contact Stage A audit: executed
- Python-bound listener getters: none (`dir(listener)` exposed no `get*` methods)
- Native pair/primitive identity: unavailable
- Native contact point: unavailable
- Native contact normal: unavailable. The static wall's triangle winding is known, but it cannot replace a simulator-native contact record for the R1 formal gate.
- Formal contact-point velocity: unavailable because no native beam primitive ID was readable.
- Nearest-plane, endpoint, and nearest-node signals were not substituted into the formal gate.

## Stage results

- A native contact extraction: BLOCKED — binding does not expose contact IDs, points, or normals.
- B anchored-free contamination: not run.
- C stable normal contact: not run.
- D monotonic breakaway: not run.
- E fresh stick/slip: not run.
- F free-forward / blocked-jam: not run.

## Oracle status

- No Oracle threshold was fitted or frozen.
- `configs/oracle_sofa_frozen.yaml` was not created.
- Existing calibration inputs and reports were not changed.

## Fact

The local official v26.06.00 sources implement `ContactListener` C++ methods such as contact-count, element, and point access. The verified official SofaPython3 runtime can instantiate the component, but does not bind those getters into Python. After correcting the scene's component-retrieval path, the listener still exposed zero getter methods during the audited contact scene.

## Inference

This is an engineering/API limitation, not a physical separation result. The R1 requirements prohibit replacing native identity with endpoint, nearest-node, or nearest-plane proxies.

## Unknown

Whether a supported C++ adapter or another officially exposed SOFA native-contact API can produce the required per-contact identity remains unknown. Held-out capability and unified-passage behavior remain untested.

## Next action

Review the failed controlled physical separation before any capability or passage experiment.
