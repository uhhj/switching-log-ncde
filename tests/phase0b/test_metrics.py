import numpy as np

from slncde.phase0a.metrics import select_horizon_step
from slncde.phase0a.snapshot import cable_rmse
from slncde.phase0b.metrics import experiment_verdict, repeat_gate


CONFIG = {
    "experiment": {"seeds": [1, 2, 3, 4, 5]},
    "analysis": {"repeat_rmse_500ms_max_m": 0.0015},
}


def test_rmse_horizon_and_repeat_gate():
    assert cable_rmse(np.zeros((2, 3)), np.ones((2, 3)) * 0.002) == 0.002
    assert select_horizon_step([4, 8, 12, 16, 20], 4, 50, 240) == 16
    assert repeat_gate([0.0010, 0.0011, 0.0012, 0.0018, 0.0021], CONFIG)
    assert not repeat_gate([0.0030] * 5, CONFIG)


def test_experiment_verdict_go_and_weak():
    passing = {
        "completed_seeds": 5,
        "sustained_slip_branches": 3,
        "sustained_jam_branches": 3,
        "gates": {name: True for name in "ABCDEF"},
    }
    assert experiment_verdict(passing, CONFIG) == "PHASE0B_FIXTURE_GO"
    weak = {
        **passing,
        "sustained_slip_branches": 2,
        "gates": {"A": True, "B": True, "C": True, "D": False, "E": True, "F": False},
    }
    assert experiment_verdict(weak, CONFIG) == "PHASE0B_FIXTURE_WEAK"
