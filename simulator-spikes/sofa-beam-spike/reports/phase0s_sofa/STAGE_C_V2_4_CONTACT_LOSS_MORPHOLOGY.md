# Phase 0S-SOFA Stage C V2.4-L Stable-Contact Loss Morphology Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_4L_ALL_INTERNAL_GAPS_PROXIMITY_ONLY`

## Execution

Offline-only audit of the frozen V2.4 coupled-correction trace. No SOFA, Docker, GPU, trace regeneration, physics change or threshold change.

## Frozen failed gate

- occupancy: 0.4257142857142857
- episodes: 13
- longest timestamp-span dwell: 0.09400000000000008 s
- internal gaps: 12

## Gap classes

```json
{
  "PROXIMITY_ONLY": 12
}
```

## Per-gap summary

| gap | class | start s | end s | native frac | LCP frac | reaction frac | pre nodes | post nodes | overlap |
|---:|---|---:|---:|---:|---:|---:|---|---|---|
| 1 | PROXIMITY_ONLY | 0.472000 | 0.568000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 2 | PROXIMITY_ONLY | 0.666000 | 0.796000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 3 | PROXIMITY_ONLY | 0.800000 | 0.806000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 4 | PROXIMITY_ONLY | 0.812000 | 0.836000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 5 | PROXIMITY_ONLY | 0.840000 | 0.860000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 6 | PROXIMITY_ONLY | 0.868000 | 0.878000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 7 | PROXIMITY_ONLY | 0.882000 | 0.888000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 8 | PROXIMITY_ONLY | 0.896000 | 0.898000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 9 | PROXIMITY_ONLY | 0.904000 | 0.904000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 10 | PROXIMITY_ONLY | 0.922000 | 0.922000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 11 | PROXIMITY_ONLY | 0.958000 | 0.958000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |
| 12 | PROXIMITY_ONLY | 0.986000 | 0.986000 | 1.000 | 1.000 | 1.000 | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] | [1, 2, 3, 4, 5, 6] |

## Fact

Every internal contact gap preserves mobile native proximity detection while formal geometric contact is absent.

## Inference

The repeated contact losses are not explained by alarm-envelope exit, native-detection dropout, or temporary footprint loss.

## Unknown

Node migration and global LCP/reaction are diagnostic only; no node-specific constraint-row attribution is available.

## Next action

Review frozen coupled-correction and damping semantics, then authorize exactly one bounded contact-stabilization intervention.
