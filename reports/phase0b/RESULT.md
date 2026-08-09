# Phase 0B Fixture Task Qualification

## Verdict
PHASE0B_ENGINEERING_BLOCKED

## Setup
- task: slncde-fixture-channel-cable
- fixture: two-wall channel, gap 0.014 m, length 0.05 m
- seeds: [82001, 82002, 82003, 82004, 82005]
- CPU-only: yes

## Repeat stability
- median repeat RMSE @100ms: 0.00042709 m
- median repeat RMSE @250ms: 0.000750093 m
- median repeat RMSE @500ms: 0.000741054 m
- seeds <= 2mm: 3 / 5

## Nominal
- successful nominal branches: 3 / 5
- median final progress: 0.0630934 m
- nominal jam count: 0 / 5

## Contact
- slide contact branches: 3 / 5
- jam contact branches: 3 / 5
- median slide contact duration: 483.333 ms
- median jam contact duration: 216.667 ms

## Mode realization
- sustained slip branches: 2 / 5
- sustained stick seeds: 0 / 5
- sustained jam branches: 0 / 5
- representative slide sequence: free -> slip -> contact_transition
- representative jam sequence: free -> contact_transition -> free -> contact_transition -> free -> contact_transition -> slip -> contact_transition -> slip

## Fact
3 fixed seeds completed; nominal succeeded in 3, slide contact occurred in 3, jam contact occurred in 3, sustained slip occurred in 2, and sustained jam occurred in 0. Prescribed results are missing for seeds [82003, 82004].

## Inference
The simulator pipeline did not produce all prescribed scientific data.

## Scientific interpretation
No fixture-task qualification conclusion is available.

## Next action
Restore simulator execution and complete all five fixed seeds.
