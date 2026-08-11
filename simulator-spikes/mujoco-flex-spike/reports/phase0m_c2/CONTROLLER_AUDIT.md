# Phase 0M-C1 Controller Audit

## Source
- branch: phase0m-c2-contact-control
- commit: a2545da75e5b52f96fd7d713739974a6cb7eff03
- config: configs/phase0m_c1_reachable_horizon.yaml

## Endpoint mapping
- body: cable_15 (id 16)
- joint ids: [45, 46, 47]
- DOF addresses: [45, 46, 47]
- joint axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
- actuators: 0
- force path: FlexSimulator.step -> data.xfrc_applied[endpoint_body, :3]

## Old Phase 0M/C1 controller
- implementation: moving Cartesian position-reference PD force servo
- Kp/Kd: 40.0 N/m / 1.0 N s/m
- force limit: 2.0 N vector norm
- telemetry: applied_endpoint_force and reference_travel are saved; x_ref is reconstructable and y_target is saved; no actuator force exists

## Phase 0S force source
- application: FlexSimulator.step -> data.xfrc_applied[endpoint_body, :3]
- stick/slip/jam command force: 0.2 / 1.5 / 2.0 N

## Centered tracking

| seed | reference x travel | head x travel | leading4 mean travel | leading4 min travel | final ref-head error | max ref-head error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 83001 | 0.205004497 | 0.184558334 | 0.182324702 | 0.180178627 | 0.020446163 | 0.024236029 |
| 83002 | 0.205004497 | 0.184565799 | 0.182333133 | 0.180188049 | 0.020438698 | 0.024238855 |
| 83003 | 0.205004497 | 0.184551788 | 0.182317472 | 0.180170759 | 0.020452709 | 0.024234088 |
| 83004 | 0.205004497 | 0.184564742 | 0.182331844 | 0.180186483 | 0.020439755 | 0.024242156 |
| 83005 | 0.205004497 | 0.184563858 | 0.182330822 | 0.180185309 | 0.020440639 | 0.024238198 |

## Medium / large contact
- offset_medium: median y error while in raw contact 0.002063350 m; median endpoint vy 0.000343723 m/s; median normal force 0.095386551 N; contact episodes 2482; touch/release 2236/2236
- offset_large: median y error while in raw contact 0.004700221 m; median endpoint vy 0.000000018 m/s; median normal force 0.191984545 N; contact episodes 2741; touch/release 2743/2739

## Chatter
- raw contact episode count: 5223
- median raw contact episode duration: 6.000 ms
- frozen minimum dwell: 80.000 ms
- episodes shorter than minimum: 5219
- short-contact fraction: 0.999234157
- Oracle touch/release: 4979 / 4975

## Fact
C1 used a real endpoint force path rather than qpos teleportation, but coupled x/y PD tracking under one vector-norm cap and activated the lateral target from the start. Almost every raw fixture-contact episode was shorter than the frozen dwell requirement.

## Inference
C2 can cleanly test per-axis bounded impedance with geometry-triggered lateral activation and one shared lateral cap, while preserving the C1 references and all task/contact variables.

## Action
Run the single frozen C2 controller correction without parameter tuning.
