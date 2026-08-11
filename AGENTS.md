# Repository Constraints

- Scientific goal: determine whether explicit contact regimes add predictive information about future cable dynamics.
- CPU time only during Phase 0 task qualification.
- Keep `external/deformable-ravens` as a Git submodule.
- Report status, key metrics, Fact/Inference, and exactly one next action.

## Current phase: Phase 0B-T2

The previous open two-wall channel realization is closed as NO-GO: canonical
preparation, repeat stability and nominal execution passed, but slide/jam
probes produced zero fixture contact.

Current work replaces it with a guide/funnel constriction passage task that
must naturally realize hierarchical contact, stick/slip and jam states.

Do not restore the deleted channel/local-teleport runtime paths.
Do not train dynamics models until Phase 0 task qualification passes.

## Current phase: Phase 0C0

Phase 0B-T2 established reliable preparation, repeatability and real fixture
contact, but no sustained stick/slip/jam.

Before changing task geometry again, Phase 0C0 isolates simulator/contact
representation from DLO task mechanics using minimal single-bead contact
probes.

The key decision is raw-physics capability versus Oracle-label capability.
Do not tune the T2 fixture or train models during Phase 0C0.

## Current simulator path

Phase 0C0 established that PyBullet contact extraction and Oracle labels
can recognize stick/slip/jam, while the DeformableRavens bead-chain task
could not realize them reliably.

Phase 0S-MJ then demonstrated free/stick/slip/jam capability with MuJoCo
1D flex.

Current phase: Phase 0M, a single MuJoCo flex constrained-passage task.
Do not expand DeformableRavens as a parallel primary simulator.
Do not train dynamics models until Phase 0M and the subsequent
matched-state/matched-action audit pass.

## Phase 0M-C1

The original Phase 0M fixed-duration rollout had insufficient geometric
reachability for the leading-4 passage criterion. C1 changed only the
longitudinal drive-distance / horizon protocol; geometry, controller speed,
branch offsets, friction and Oracle definitions remained unchanged. The
corrected batch reached the constrained region but remained a task NO-GO.

## Current phase: Phase 0M-C2

C1 removed the fixed-horizon reachability confound, but the unified MuJoCo
flex passage task still showed short contact chatter and no sustained
stick/slip/jam.

C2 is the final MuJoCo task correction. Geometry, friction, forward
reference speed, offsets, solver and Oracle definitions stay frozen.
Only endpoint actuation changes from position-reference control to bounded
contact-compatible Cartesian impedance.

If C2 fails the original Phase 0M gates, no further MuJoCo passage tuning
is allowed; the next simulator path is SOFA BeamAdapter.
