# Phase 0A Smoke Result

## Verdict
PHASE0A_SMOKE_NO_GO

## Key metrics
- valid pairs: 0 / 5
- median initial cross RMSE: 0 m
- median RMSE @100 ms: 0.00368291 m
- median RMSE @250 ms: 0.00544783 m
- median RMSE @500 ms: 0.00550342 m
- median repeat RMSE @500 ms: 0.00524511 m
- median cross/repeat ratio @500 ms: 4.07835
- free median contact speed: 0 m/s
- high-friction median contact speed: 0 m/s
- free median contact force: 0 N
- high-friction median contact force: 0 N
- regime-contrast pairs: 0

## Fact
5 fixed-seed pairs completed; 0 passed every pair gate and 0 showed observed friction-response contrast. This is the single permitted correction run: hidden lateral friction changed from 1.2 to 2.0 after the initial five-pair pilot had matched initial states but no observed regime contrast.

## Inference
The configured hidden-friction intervention did not produce future divergence that was both regime-grounded and clearly above repeat noise.

## Scientific interpretation
Do not proceed to NCDE, mode-classifier, MPC, or RL work.

## Next action
Stop complex-model development for this contact-friction mechanism.
