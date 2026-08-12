# Phase 0S-SOFA Stage C Stable Contact V2

## Verdict

PHASE0S_SOFA_STAGE_C_INSTRUMENTATION_BLOCKED

## Runtime

- SOFA: official v26.06.00 Linux binary; BeamAdapter and SofaPython3: v26.06.
- NativeContactBridge: full-path loaded; source unchanged.
- CPU-only: yes.

## Material source audit

The local v26.06 BeamAdapter material interface is `BeamInterpolation.radius`,
`defaultYoungModulus`, and `defaultPoissonRatio`; mass density is the
`AdaptiveBeamForceFieldAndMass.massDensity` Data. Stage C V2 binds and reads
those Data after initialization.

## Material requested vs effective readback

- Requested E / effective six-edge values: 1000000 Pa / `[1000000]*6`.
- Requested nu / effective values: 0.4 / `[0.4]*6`.
- Requested radius / effective values: 0.002 m / `[0.002]*6`.
- Requested rho / effective values: 1000 kg/m3 / `[1000]*6`.
- Material binding: PASS for C0 and C1.

## Historical Stage-B qualification

The Stage-B config requested E=1e6 but did not bind it to `BeamInterpolation`.
Its no-contact anchor-to-LCP structural conclusion is retained; its deformation
is not interpreted as a 1 MPa response.

## C0 instrumentation

- Steps / bridge serial expected and actual: 300 / 1–300 / 1–300.
- Duplicate / gap / regression / source mismatch: 0 / 0 / 0 / 0.
- Force target DOF / mismatch frames / max error: 6 / none / 0 N.
- Timing start / end / interval max error: `4.44e-16` / `4.44e-16` / `5.38e-17` s.
- Tail translation max: 0 m. All C0 instrumentation gates: PASS.

## C0 reaction baseline

- Native contacts: 0.
- Sentinel minimum centerline clearance: 0.879789305355 m.
- LCP vector size max: 0.
- Reaction p99 / max: 0 / 0.
- C0 baseline: PASS.

## C1 instrumentation

- Steps / bridge serial expected and actual: 500 / 1–500 / 1–500.
- Duplicate / gap / regression / source mismatch: 0 / 0 / 0 / 0.
- Force target DOF / mismatch frames / max error: 6 / none / 0 N.
- Timing start / end / interval max error: `7.77e-16` / `7.77e-16` / `1.09e-16` s.
- Material readback and tail anchor: PASS.

## C1 zero-load baseline

- Samples: 50.
- Native contacts: 100 (two contacts per frame).
- LCP size max: 6.
- Reaction max: 0.
- Result: FAIL — `zero_load_initial_geometry_not_contact_free`.

## C1 contact dwell, reaction coupling, and geometry

Not evaluated for a scientific verdict because the zero-load instrumentation
gate failed. Measurement contact samples do not contribute after this block.

## Native contact ↔ LCP size relation

Measurement diagnostic only: `(0 native contacts, 0 unique beam primitives,
0 LCP entries) -> 350 frames`. The zero-load samples are excluded from this
scientific diagnostic after the instrumentation block.

## Fact

Both C0 and C1 explicitly bound and read back the requested material, one fresh
bridge `CollisionEndEvent` arrived for every recorded step, and force Data
matched the cached step-start schedule exactly. C1 nevertheless had native
wall contact before the scheduled load began.

## Inference

No stable-contact or reaction-coupling claim is permitted. The C1 initial
geometry, under the fixed official point/triangle collision semantics, was not
contact-free at zero load.

## Unknown

Sustained contact, contact reaction coupling, local contact geometry,
breakaway, stick/slip, jam, Oracle calibration, held-out capability, and
unified passage remain untested.

## Next action

Repair only the failing instrumentation/provenance gate and rerun the unchanged C0/C1 sequence as appropriate.
