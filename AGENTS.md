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
