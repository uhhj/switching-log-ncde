# Phase 0B-T2 Constriction Passage Qualification

## Verdict
PHASE0B_T2_NO_GO

## Preparation
- common snapshot PASS: 5 / 5

## Repeat
- RMSE @100ms: 0 m
- RMSE @250ms: 0 m
- RMSE @500ms: 0 m
- seeds <=2mm: 5 / 5

## Passage
- nominal success: 0 / 5
- median final progress: 0.0620256 m

## Contact layer
- nominal contact: 5 / 5
- slide contact: 5 / 5
- jam contact: 5 / 5
- median contact dwell: 158.333 ms

## Friction layer
- sustained stick seeds: 0 / 5
- sustained slip seeds: 0 / 5
- median stick dwell: 0 ms
- median slip dwell: 0 ms

## Failure layer
- sustained jam seeds: 0 / 5
- median jam dwell: 0 ms
- jam onset count: 0

## Events
- touch: 40
- release: 25
- stick->slip: 0
- slip->stick: 0
- jam onset: 0
- jam release: 0

## Fact
5 / 5 paired rollouts completed; nominal passage succeeded in 0, slide contact in 5, and jam in 0.

## Inference
The fixed constriction geometry does not jointly support nominal passage and reproducible contact regimes.

## Scientific interpretation
The current T2 realization is not qualified for Phase 0C.

## Next action
Reevaluate the simulator representation or mode hypothesis.
