import numpy as np


def select_horizon_step(physics_steps, onset_step, horizon_ms, hz):
    steps = np.asarray(physics_steps, dtype=np.int64)
    steps = steps[steps >= int(onset_step)]
    if steps.size == 0:
        raise ValueError("no aligned sample at or after insertion onset")
    target = float(onset_step) + float(horizon_ms) * float(hz) / 1000.0
    return int(steps[np.argmin(np.abs(steps.astype(np.float64) - target))])
