# Phase 0S-SOFA Stage C V2.3-L Crossing-Local / Global-Response Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_3L_LOCAL_CONTACT_PERSISTS_GLOBAL_RESPONSE_ACTIVE_AT_FIRST_CROSSING`

## Scope

Offline-only analysis of the committed V2.3-R2 trace. No SOFA, Docker, GPU, trace regeneration or physics change.

The formal verdict uses the earliest crossing only. Later crossings are propagation diagnostics.

Node-local DetectionOutput/contact state is available. Constraint-vector and reaction signals are global and are not node-mapped.

## Per-crossing state

| node | crossing s | from y | to y | local geo | normal_y from | normal_y to | flip | global LCP | global reaction | min DetectionOutput | last local geo s | gap s |
|---:|---:|---:|---:|:---:|---:|---:|:---:|:---:|:---:|---:|---:|---:|
| 6 | 0.144000 | 1.23012783e-05 | -0.000209000807 | True | -1.000000 | -1.000000 | False | True | True | -0.00198769872 | 0.144000 | 0.000000 |
| 5 | 0.156000 | 0.000160951924 | -2.42765365e-05 | True | -1.000000 | -1.000000 | False | True | True | -0.00183904808 | 0.156000 | 0.000000 |
| 4 | 0.174000 | 0.000131363781 | -3.90174653e-05 | True | -1.000000 | -1.000000 | False | True | True | -0.00186863622 | 0.174000 | 0.000000 |
| 3 | 0.194000 | 0.000138896547 | -1.84951751e-05 | True | -1.000000 | -1.000000 | False | True | True | -0.00186110345 | 0.194000 | 0.000000 |
| 2 | 0.218000 | 7.79845923e-05 | -5.91335991e-05 | True | -1.000000 | -1.000000 | False | True | True | -0.00192201541 | 0.218000 | 0.000000 |
| 1 | 0.258000 | 7.69107105e-06 | -8.82578764e-05 | True | -1.000000 | -1.000000 | False | True | True | -0.00199230893 | 0.258000 | 0.000000 |

## Fact

Node-local formal geometric contact persists while global LCP rows and global reaction response are active at first crossing completion.

## Inference

Detection geometry alone is no longer the leading explanation: node-local formal contact still exists while global response remains active at the first barrier violation.

## normal_y diagnostic

beam-outward normal_y does not change sign across the first crossing. Inspect the local multi-contact geometry and normal records; this diagnostic does not alter the formal verdict or choose the next physics intervention.

## Unknown

Global constraint rows and reaction are not node-mapped; the trace cannot prove the individual crossing node's constraint/reaction contribution.

## Next action

Review the frozen first-crossing local-contact/global-response evidence together with the official SOFA barrier/contact-response semantics, then authorize exactly one bounded barrier-topology or response-formulation intervention while keeping Sphere radius, effective envelope, load, material and dt frozen.
