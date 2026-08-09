# Repository Constraints

- Scientific goal: determine whether an explicit contact regime contains additional predictive information about future cable dynamics.
- Current phase: Phase 0A only.
- Do not implement complex models until Phase 0A passes.
- Prioritize an executable simulator-to-paired-rollout-to-analysis path.
- Do not add excessive audit, contract, defensive-coding, or SHA256 machinery.
- GPU time is expensive; Phase 0A is strictly CPU-only.
- Keep the simulator in `external/deformable-ravens` as a Git submodule.
- Each round reports only status, key metrics, Fact/Inference, and exactly one next action.

## Current scientific phase

Phase 0A hidden-friction smoke is closed as NO-GO for that mechanism.

Current work is Phase 0B: qualify a fixture-based cable insertion task that
naturally realizes free/contact/slip/jam with low same-condition repeat noise.

Do not implement NCDE, Log-NCDE, mode inference, MPC or RL until Phase 0B
passes and Phase 0C establishes Oracle-mode necessity.

## Current phase: Phase 0B-R1

Phase 0A hidden-friction mechanism: NO-GO.

Phase 0B established low repeat drift and real fixture contact on completed
seeds, but random fixture placement failed for two seeds and the current jam
probe produced no sustained jam.

Phase 0B-R1 changes only the fixture frame to a fixed canonical workspace
frame and reruns all five seeds.

Do not modify jam geometry in R1.
Do not enter Phase 0C until task qualification is complete.

## Current phase: Phase 0B-R1.1

R1 fixed workspace placement but endpoint-only staging did not control the
adjacent cable geometry. Four of five seeds contacted fixture walls before
the common snapshot.

R1.1 uses benchmark initialization only: fixture creation is deferred, the
endpoint-adjacent five-bead segment is canonicalized outside the entry plane,
the endpoint is grasped, then the fixture is created and a contact-free common
snapshot is saved.

No cable reset or teleportation is permitted after that snapshot.
Do not tune jam geometry in R1.1.
