# Phase 0B-R1.1 Local-Segment Canonicalization

## Verdict
PHASE0B_R1_1_PREPARATION_FAIL

## Preparation
- attempted: 5 / 5
- local geometry pass: 0 / 5
- after-grasp pass: 0 / 5
- fixture overlap-free: 0 / 5
- fixture-settle contact-free: 0 / 5
- common snapshot PASS: 0 / 5

## Local geometry
- median minimum entry clearance: -0.0463966 m
- median alignment cosine: 0.15745
- median max local speed: 0.000239977 m/s

## Repeat stability
- median RMSE @100 ms: n/a m
- median RMSE @250 ms: n/a m
- median RMSE @500 ms: n/a m
- seeds <= 2 mm: 0 / 5

## Nominal
- executable: 0 / 5
- progress >= 0.040 m: 0 / 5
- median final progress: n/a m
- sustained jam: 0 / 5

## Fixture contact
- slide contact: 0 / 5
- jam contact: 0 / 5

## Modes
- sustained slip: 0 / 5
- sustained stick: 0 / 5
- sustained jam: 0 / 5

## Representative transitions
- slide: n/a
- jam: n/a

## Benchmark boundary
Local-segment canonicalization is benchmark initial-state generation. It is applied before the common snapshot and identically for all future branches. No cable teleportation or state reset is allowed after the common snapshot.

## Fact
All 5 fixed seeds were attempted; local geometry passed in 0, after-grasp geometry passed in 0, and 0 reached a contact-free common snapshot. 0 paired rollouts completed.

## Inference
The fixed one-pass initialization did not yield five valid common snapshots, so downstream rollout counts are not a task verdict.

## Scientific interpretation
Initial-state generation remains unqualified.

## Next action
Inspect compatibility between the cable constraint frames and the five-bead reset geometry.
