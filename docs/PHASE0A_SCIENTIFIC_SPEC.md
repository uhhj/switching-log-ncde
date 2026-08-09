# Phase 0A Scientific Specification

## Question

Starting from a matched PyBullet snapshot, does changing only the hidden cable-to-plane friction regime under an identical future robot action create a repeatable future cable divergence that is materially larger than a same-condition repeat?

## Causal pair

For each of five fixed seeds, the environment is reset once, the final cable bead is grasped, the world settles, and one common PyBullet state is saved. The state is restored for `free`, `hidden_high_friction`, and a second `free` rollout. The hidden factor is armed only after each restore. One pull target, computed from the final two cable beads at the common snapshot, is reused by all branches.

## Observed regime

Branch names are interventions, not observed mode labels. During the main pull, a sample is stick-like only when at least one selected bead has active plane friction and mean selected-bead speed is at most 0.003 m/s. It is slip-like only when contact is active and the speed is at least 0.008 m/s. A regime is present only after at least 80 ms of sustained samples.

The alternate continuous-response check requires the two branches' median active-contact speeds to differ by at least the entire configured stick-to-slip gap (0.005 m/s). This uses an existing physical threshold and avoids inventing a force cutoff. Contact force remains a required reported continuous quantity.

## Gate

A valid pair must have an initial cross-condition cable RMSE no greater than 10 micrometres, a 500 ms future RMSE of at least 1 mm, a cross/repeat ratio of at least 3 using a 10 micrometre repeat floor, and either sustained observed-regime contrast or the continuous speed response above. At least three of all five fixed pairs are required for `PHASE0A_SMOKE_GO`.

These are internal smoke-test thresholds, not literature standards. Phase 0A can establish only that the explicit contact regime carries predictive information in this controlled simulator experiment; it cannot establish that Switching Log-NCDE is the appropriate model.

## Execution revision

The initial five-pair pilot used hidden lateral friction 1.2. All initial states matched, but none of the five pairs showed observed regime contrast. The protocol's single allowed correction was therefore applied: hidden lateral friction was changed to 2.0 with seeds, motion, direction, and gates unchanged. The committed configuration and result describe that final run.
