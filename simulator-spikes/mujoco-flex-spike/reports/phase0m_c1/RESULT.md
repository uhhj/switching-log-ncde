# Phase 0M-C1 Reachable-Horizon Correction

## Verdict
PHASE0M_C1_TASK_NO_GO

## Protocol correction
- old command budget: 0.090000000 m
- required geometric translation: 0.185004526 m
- new command distance: 0.205004497 m
- new nominal drive time: 6.833483235 s
- new max duration: 7.583483235 s
- only changed variable: longitudinal command distance / drive horizon and its termination logic

## Preparation
- PASS: 5/5

## Protocol reachability
- centered sufficient travel: 5/5
- medium entered throat: 5/5
- large entered throat: 5/5

## Repeat
- RMSE @100 ms: 0.000000000 m
- RMSE @250 ms: 0.000000000 m
- RMSE @500 ms: 0.000000000 m
- seeds <=1.5 mm: 5/5

## Passage
- centered success: 0/5
- median centered progress: 0.182330822 m

## Contact
- centered contact: 0/5
- medium contact: 5/5
- large contact: 5/5
- median contact dwell: 33.000 ms

## Friction
- sustained stick episodes: 0/15 non-repeat
- medium sustained slip: 0/5
- median stick dwell: 2.000 ms
- median slip dwell: 13.000 ms

## Failure
- large sustained jam: 0/5
- median jam dwell: 0.000 ms
- median jam normal force: 0.000000000 N
- median jam-window progress: 0.000000000 m

## Events
- touch: 4979
- release: 4975
- stick->slip: 0
- slip->stick: 0
- jam onset: 0
- jam release: 0

## Region dwell
- funnel contact dwell: 12.000 ms
- throat contact dwell: 44.000 ms

## Mode coverage
- occupancy: {'contact': 0.20052742616033756, 'stick': 2.6371308016877634e-05, 'slip': 0.18051160337552744, 'jam': 0.0}
- episode coverage: {'contact': 10, 'stick': 0, 'slip': 0, 'jam': 0}
- dwell: {'contact': 33.0, 'stick': 2.0, 'slip': 13.0, 'jam': 0.0}

## Phase0M vs C1
- centered success: 0 -> 0
- medium contact: 0 -> 5
- large contact: 5 -> 5
- stick: 0 -> 0
- slip: 0 -> 0
- jam: 0 -> 0

## Fact
The old horizon was confounded; under the corrected distance-derived protocol, all branches reached the intended region but these scientific gates failed: centered_passage, medium_slip, large_jam, dataset_stick, meaningful_transitions.

## Inference
The horizon confound is removed, so the remaining failure belongs to the frozen task, controller coupling, or contact realization.

## Scientific interpretation
The current passage does not naturally supply all required sustained regimes; contact chatter after throat entry is not merely a truncated approach.

## Next action
Choose one minimal task correction versus switching to SOFA BeamAdapter.
