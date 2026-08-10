# Phase 0M Reachability Audit

## Source
- branch: phase0m-c1-reachable-horizon
- commit: 06f8eef9c80a4be7c05c88d040737058ba1836fc
- config: configs/phase0m_passage_smoke.yaml
- previous result: PHASE0M_NO_GO

## Actual geometry
- cable radius: 0.005000000 m
- local spacing: 0.009999986 m
- funnel entry x: 0.000000000 m
- throat start x: 0.050000000 m
- throat exit x: 0.100000000 m
- success plane x: 0.115000000 m
- entry half-gap: 2.4 r = 0.012000000 m
- throat half-gap: 1.18 r = 0.005900000 m
- exit margin: 3.0 r = 0.015000000 m

## Controller
- actual controller type: moving endpoint position-reference servo
- forward speed/reference semantics: target_x = start_x + 0.030000 m/s * elapsed_s
- old duration: 3.000000 s
- old planned travel: 0.090000000 m

## Per-seed required travel

| seed | leading4 min x | required success translation | old budget sufficient |
| --- | ---: | ---: | :---: |
| 83001 | -0.070004526 | 0.185004526 | no |
| 83002 | -0.070004526 | 0.185004526 | no |
| 83003 | -0.070004526 | 0.185004526 | no |
| 83004 | -0.070004526 | 0.185004526 | no |
| 83005 | -0.070004526 | 0.185004526 | no |

Required translation min/median/max: 0.185004526 / 0.185004526 / 0.185004526 m.

## Throat reachability
- median initial head distance to throat start: 0.090000219 m
- medium target: 0.65 r = 0.003250000 m
- large target: 1.55 r = 0.007750000 m
- medium clearance proxy: -0.002350000 m
- large clearance proxy: -0.006850000 m
- medium old traces leading-4 entered throat: 0/5
- large old traces leading-4 entered throat: 0/5
- large first-contact region (x proxy; trace has no fixture geom id): funnel_x_proxy

## Verdict
REACHABILITY_CONFOUNDED

## Fact
The 0.090000 m planned endpoint-reference travel is below the 0.185004526 m leading-4 success lower bound; all centered traces stopped before the success plane and medium never entered the throat.

## Inference
The fixed 3.0 s horizon confounded the previous Phase 0M task-level NO-GO, so one distance-derived horizon correction is applicable.

## Action
Run one fresh C1 batch with only the longitudinal drive-distance and termination protocol corrected.
