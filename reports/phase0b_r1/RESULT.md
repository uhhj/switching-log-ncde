# Phase 0B-R1 Canonical Fixture Qualification

## Verdict
PHASE0B_R1_ENGINEERING_BLOCKED

## Preparation
- completed seeds: 1 / 5
- canonical staging contact-free: 1 / 5
- fixture placement failures: 0 / 5
- prep failures: 4 / 5

## Repeat stability
- median RMSE @100 ms: 0 m
- median RMSE @250 ms: 0 m
- median RMSE @500 ms: 0 m
- seeds <= 2 mm: 1 / 5

## Nominal
- executable: 1 / 5
- median final progress: 0.0155543 m
- sustained jam: 0 / 5

## Fixture contact
- slide contact: 1 / 5
- jam contact: 1 / 5

## Modes
- sustained slip in slide_probe: 1 / 5
- sustained stick: 0 / 5
- sustained jam in jam_probe: 0 / 5

## Representative transitions
- slide: slip -> free -> slip -> contact_transition -> slip -> contact_transition -> slip -> contact_transition -> slip
- jam: slip -> free -> slip -> contact_transition -> slip -> contact_transition -> slip -> contact_transition -> slip -> contact_transition -> slip -> contact_transition

## Comparison with Phase 0B
- previous completed: 3 / 5
- previous repeat @500ms: 0.000741054 m
- previous slip: 2 / 3 completed
- previous jam: 0 / 3 completed
- previous preparation failures: 82003, 82004

## Fact
Canonical preparation completed for 1 / 5 seeds; 1 / 5 staging states were fixture-contact-free. Repeat, nominal, contact, and mode counts are reported from 1 complete paired rollouts.

## Inference
The fixed frame was placeable, but adjacent cable beads contacted the walls during staging in four seeds, so the frame alone did not produce five comparable common states.

## Scientific interpretation
No R1 task-qualification conclusion is available.

## Next action
Redesign canonical staging preparation so the trailing cable remains outside the channel, then rerun all five fixed seeds.
