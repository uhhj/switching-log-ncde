# Phase 0M-C2 Contact-Compatible Endpoint Control

## Verdict
PHASE0M_C2_FINAL_MUJOCO_NO_GO

## Frozen task
- MuJoCo: 3.2.3 (CPU only)
- geometry: C1 unchanged
- friction: C1 unchanged
- forward speed: 0.030000000 m/s, C1 unchanged
- C1 drive protocol: unchanged
- Oracle: C1 unchanged
- seeds: [83001, 83002, 83003, 83004, 83005]

## Controller change
- old controller: legacy moving position-reference Cartesian PD force with a vector-norm cap
- new controller: bounded contact-compatible Cartesian impedance with per-axis force caps
- endpoint mapping: body 16 (cable_15), joints [45, 46, 47], DOFs [45, 46, 47], force via data.xfrc_applied on endpoint body
- Kx/Dx: 40.000000 N/m / 1.000000 N s/m
- Fx max + source: 2.000000 N / existing Phase0M endpoint force limit
- Ky/Dy: 40.000000 N/m / 1.000000 N s/m
- shared Fy max + source: 2.000000 N / existing Phase0M endpoint force limit
- shared across centered/medium/large: yes
- lateral activation rule: leading-4 mean x >= -0.019999971 m

## Preparation
- PASS: 5/5

## Repeat
- RMSE @100 ms: 0.000000000 m
- RMSE @250 ms: 0.000000000 m
- RMSE @500 ms: 0.000000000 m
- seeds <=1.5 mm: 5/5

## Passage
- centered success: 0/5
- median centered progress: 0.182331460 m
- median distance remaining on failures: 0.004818249 m

## Contact
- centered / medium / large: 0/5 / 5/5 / 5/5
- funnel / throat dwell: 13.000 ms / 44.000 ms
- contact episode count: 5157
- median dwell: 32.000 ms
- median raw episode duration: 6.000 ms
- short-contact fraction (<80.0 ms): 0.999612178

## Friction
- sustained stick: 0/15 non-repeat
- medium sustained slip: 0/5
- stick / slip dwell: 2.000 ms / 15.000 ms

## Failure
- large sustained jam: 0/5
- dwell: 0.000 ms
- normal force: 0.000000000 N
- jam-window progress: 0.000000000 m

## Controller diagnostics
- median x tracking error: 0.023544300 m
- p95 x tracking error: 0.026809440 m
- median y tracking error while active: 0.000061911 m
- x force saturation fraction: 0.000000000
- y force saturation fraction: 0.000000000

## Events
- touch: 4912
- release: 4907
- stick->slip: 0
- slip->stick: 0
- jam onset: 0
- jam release: 0

## C1 vs C2
- touch/release: 4979/4975 -> 4912/4907
- contact dwell: 33.000 -> 32.000 ms
- centered success: 0 -> 0
- stick: 0 -> 0
- slip: 0 -> 0
- jam: 0 -> 0

## Fact
C2 completed normally with geometry, friction, solver, forward reference, offsets, Oracle and C1 drive protocol frozen; failed gates: centered_passage, medium_slip, large_jam, dataset_stick, meaningful_transitions.

## Inference
Contact-compatible endpoint control did not make the unified MuJoCo passage naturally realize every required sustained regime.

## Scientific interpretation
MuJoCo basic regime capability and corrected reachability are already established, so the constrained-passage realization now has sufficient negative evidence and receives no further tuning.

## Next action
Start SOFA BeamAdapter DLO spike.
