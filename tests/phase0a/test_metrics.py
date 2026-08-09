import numpy as np
import pytest

from slncde.phase0a.metrics import (
    experiment_gate,
    pair_gate,
    repeat_ratio,
    select_horizon_step,
    sustained_mask,
)


CONFIG = {
    "analysis": {
        "initial_state_rmse_max_m": 1e-5,
        "future_divergence_min_m": 1e-3,
        "future_vs_repeat_multiplier": 3.0,
    },
    "gate": {"minimum_valid_pairs": 3, "minimum_total_pairs": 5},
}


def test_horizon_selection_and_repeat_ratio():
    steps = np.asarray([24, 28, 32, 36, 40, 44, 48])
    assert select_horizon_step(steps, 24, 50, 240) == 36
    assert repeat_ratio(0.0012, 0.0, 1e-5) == pytest.approx(120.0)


def test_sustained_mask_requires_configured_duration():
    steps = np.arange(4, 44, 4)
    mask = np.asarray([False, True, True, True, True, True, False, True, True, False])
    result = sustained_mask(mask, steps, minimum_duration_ms=80, hz=240)
    assert result.tolist() == [False, True, True, True, True, True, False, False, False, False]


def test_pair_and_experiment_gates():
    passing = {
        "initial_cross_rmse": 0.0,
        "future_cross_rmse_500ms": 0.0012,
        "cross_to_repeat_ratio_500ms": 12.0,
        "friction_response_contrast": True,
    }
    assert pair_gate(passing, CONFIG)
    pairs = []
    for index in range(5):
        pairs.append(
            {
                **passing,
                "valid_signal_pair": index < 3,
            }
        )
    assert experiment_gate(pairs, CONFIG) == "PHASE0A_SMOKE_GO"
