# Phase 0B-R1.2 Constraint-Consistent Canonical Cable Spawn

## Verdict
PHASE0B_R1_2_NO_GO

## Preparation
- attempted: 5 / 5
- canonical spawn geometry pass: 5 / 5
- after-grasp geometry pass: 5 / 5
- fixture overlap-free: 5 / 5
- fixture-settle contact-free: 5 / 5
- common snapshot PASS: 5 / 5

## Spawn diagnostics
- median endpoint spawn error: 1.08957e-07 m
- median alignment cosine after spawn settle: 1
- median max spacing error: 5.87067e-07 m
- median minimum entry clearance: 0.0169998 m
- median max local speed: 1.5182e-05 m/s

## Repeat stability
- RMSE @100ms: 0 m
- RMSE @250ms: 0 m
- RMSE @500ms: 0 m
- seeds <=2mm: 5 / 5

## Nominal
- executable: 5 / 5
- progress >=40mm: 5 / 5
- median final progress: 0.0512969 m
- sustained jam: 0 / 5

## Fixture contact
- slide contact: 0 / 5
- jam contact: 0 / 5

## Modes
- sustained slip: 0 / 5
- sustained stick: 0 / 5
- sustained jam: 0 / 5

## Representative transitions
- slide: free
- jam: free

## Benchmark boundary
The cable and its point-to-point constraints are created directly in the deterministic canonical pre-insertion geometry. The fixed seeds remain for execution consistency. No cable teleportation, fixture reconstruction, or constraint reconstruction occurs after the common snapshot.

## Fact
All 5 fixed seeds were attempted; canonical spawn geometry passed in 5, after-grasp geometry passed in 5, and 5 reached a contact-free common snapshot. 5 paired rollouts completed.

## Inference
The prepared task does not reliably realize the required stable fixture contact and slip regimes.

## Scientific interpretation
The current channel benchmark is not qualified for Phase 0C.

## Next action
Reassess the fixture-task realization before further patching.
