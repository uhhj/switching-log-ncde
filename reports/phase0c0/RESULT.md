# Phase 0C0 Contact-Regime Capability Probe

## Verdict
PHASE0C0_LABELS_CAPABLE_TASK_REPRESENTATION_SUSPECT

## Setup
- PyBullet: 202007060
- hz: 240
- bead mass: 0.1 kg
- bead collision half-width: 0.005 m
- friction: 0.6
- current T2 label thresholds: contact 0.05 N; stick 0.002 m/s; slip 0.005 m/s; command 0.005 m/s; jam 0.5 N, 100 ms, 0.0005 m; dwell 80 ms
- CPU-only: yes

## Free control
- raw free: 3/3
- label free: 3/3

## Stick
- raw capability: 3/3
- label capability: 3/3
- median contact dwell: 1000.000 ms
- median tangential speed: 0.000004867 m/s
- median displacement: 0.000000142 m (final 300 ms)
- median friction force: 0.245079216 N

## Slip
- raw capability: 3/3
- label capability: 3/3
- median contact dwell: 1000.000 ms
- median tangential speed: 5.502246500 m/s
- median displacement: 5.305288197 m
- median friction force: 0.320589613 N

## Jam
- raw capability: 3/3
- label capability: 3/3
- median wall contact dwell: 958.333 ms
- median wall normal force: 1.990831863 N
- median jam-window progress: -0.000003013 m

## Capability matrix
| Regime | Raw physics | Current label |
|---|---|---|
| stick | PASS | PASS |
| slip | PASS | PASS |
| jam | PASS | PASS |

## Fact
The minimal PyBullet scenes formed stick, slip, and jam signals, and the frozen T2 Oracle recognized all three.

## Inference
The missing T2 regimes are therefore attributable primarily to the current cable/fixture/controller realization, not basic contact representation or label thresholds.

## Scientific interpretation
Raw physics capability and current Oracle-label capability were evaluated separately in single-bead scenes without a robot, cable constraints, or the T2 fixture.

## Next action
Stop modifying DeformableRavens T2 and evaluate a simulator/representation better suited to contact-rich DLO dynamics.
