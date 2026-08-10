# Phase 0M MuJoCo Flex Constrained-Passage Qualification

## Verdict
PHASE0M_NO_GO

## Setup
- MuJoCo version: 3.2.3
- CPU-only: yes
- cable radius: 0.005 m
- fixture dimensions in meters: {'entry_full_gap': 0.024, 'throat_full_gap': 0.0118, 'funnel_length': 0.05, 'throat_length': 0.05}
- frozen Oracle source: ../../configs/phase0b_t2_constriction.yaml
- seeds: [83001, 83002, 83003, 83004, 83005]

## Preparation
- PASS 5/5

## Repeat
- RMSE @100 ms: 0.000000000 m
- RMSE @250 ms: 0.000000000 m
- RMSE @500 ms: 0.000000000 m
- per-seed threshold pass: 5/5

## Passage
- centered success: 0/5
- median progress: 0.059682374 m
- leading-4 exit success: 0/5

## Contact
- centered contact: 0/5
- medium contact: 0/5
- large contact: 5/5
- contact dwell: 12.000 ms

## Friction regimes
- sustained stick episodes: 0/15 non-repeat
- medium sustained slip: 0/5
- stick dwell: 0.000 ms
- slip dwell: 12.000 ms
- medium slip tangent speed: 0.000000000 m/s

## Failure regime
- large sustained jam: 0/5
- jam dwell: 0.000 ms
- normal force: 0.000000000 N
- jam-window progress: 0.000000000 m

## Events
- touch: 578
- release: 578
- stick_to_slip: 0
- slip_to_stick: 0
- jam_onset: 0
- jam_release: 0

## Representative sequences
- medium contact: free
- medium friction: none
- medium failure: normal
- large contact: free -> contact -> free -> contact -> free -> contact -> free -> contact -> free -> contact -> ... (229 transitions total) ... -> contact -> free -> contact -> free -> contact -> free -> contact -> free -> contact -> free
- large friction: none -> slip -> none -> slip -> none -> slip -> none -> slip -> none -> slip -> ... (229 transitions total) ... -> slip -> none -> slip -> none -> slip -> none -> slip -> none -> slip -> none
- large failure: normal

## Mode coverage
- occupancy: {'contact': 0.04342424242424243, 'stick': 0.0, 'slip': 0.04342424242424243, 'jam': 0.0}
- episode coverage: {'contact': 5, 'stick': 0, 'slip': 0, 'jam': 0}
- dwell: {'contact': 12.0, 'stick': 0.0, 'slip': 12.0, 'jam': 0.0}

## Fact
Preparation completed, but the following qualification gates failed: centered_passage, medium_contact, medium_slip, large_jam, dataset_stick, meaningful_transitions.

## Inference
The first unified MuJoCo flex passage task does not yet provide the required natural and repeatable regime coverage.

## Scientific interpretation
This is a task-level NO-GO; no geometry, controller, friction, or Oracle parameter was tuned after the formal batch began.

## Next action
Review the Phase 0M contact, progress, and mode traces to choose between one minimal task correction and SOFA BeamAdapter.
