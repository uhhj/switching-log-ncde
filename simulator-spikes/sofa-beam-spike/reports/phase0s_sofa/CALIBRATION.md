# Phase 0S-SOFA Oracle Calibration

## Verdict

PHASE0S_SOFA_ORACLE_CALIBRATION_FAIL

## Runtime

- SOFA: official v26.06.00 Linux Python 3.12 binary.
- BeamAdapter: release-bundled v26.06 plugin.
- SofaPython3: release-bundled v26.06 plugin with Python 3.12.3.
- CPU-only: yes; Docker had no GPU exposure and `CUDA_VISIBLE_DEVICES` was empty.

## Calibration setup

- seed: 84000 only.
- beam: 0.12 m length, 0.002 m radius, 7 nodes, 1,000,000 Pa Young modulus, and 1000 kg/m^3 density.
- dt: 0.002 s; 300 samples per trace; friction: 0.8.
- command: forward force 0.00005 N; normal preload 0.001 N.
- force-selection source: the one permitted free-pull sanity measured a terminal median speed of 0.0166101378318 m/s. It did not cross the specified one-time adjustment bound (<0.005 or >0.10 m/s), so the initial forward force was retained.
- fixture: a horizontal static triangle wall for stick/slip, a static vertical triangle blocker for jam, and no fixture in the free traces.
- raw channels: timestamp; ordered beam positions and velocities; command force and activity; contact-count proxy; wall identity; global LCP constraint-force proxy; wall-relative tangent speed; and tip progress.

## Contact separation

- free reaction p99: 0
- contact reaction p10: 0
- reaction threshold: not frozen (FAIL)

## Stick/slip separation

- stick tangent speed p95: 0.381683085609 m/s
- slip tangent speed p05: 0.0411734112803 m/s
- stick threshold: not frozen
- slip threshold: not frozen (FAIL)

## Jam separation

- free progress p10 @100ms: 0.000341315181385 m
- jam progress p90 @100ms: 0.00166167688981 m
- jam threshold: not frozen (FAIL)

## Frozen semantics

- minimum dwell: 80 ms.
- jam window: 100 ms.
- No Oracle threshold file was written.

## Fact

All five seed-84000 controlled scenes executed through `runSofa` and emitted the required simulator-native raw channels. Free reaction was zero. Stick and slip each had 148 contact-proxy samples with a maximum of two constraint triplets, but the global LCP constraint-force proxy had a zero 10th percentile in both traces. Free-forward and jam had zero active contact-proxy samples in the retained force/horizon.

## Inference

This one-shot controlled setup does not establish clean free/contact, stick/slip, or free-forward/jam numerical gaps. The result is a calibration failure; no numerical threshold has been manually selected.

## Unknown

Whether a differently designed controlled SOFA contact construction can create sustained separable regimes remains untested. No held-out capability or unified-passage result exists.

## Next action

Review the failed controlled physical separation before any capability or passage experiment.
