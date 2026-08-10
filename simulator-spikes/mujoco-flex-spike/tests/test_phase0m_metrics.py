import numpy as np

from src.metrics import phase0m_verdict, trajectory_rmse_at_ms


def test_repeat_metric_identical_and_shifted():
    identical = np.zeros((300, 4, 3))
    assert trajectory_rmse_at_ms(identical, identical, 500.0, 500) == 0.0
    shifted = identical + 0.003
    assert trajectory_rmse_at_ms(identical, shifted, 500.0, 500) == 0.003


def test_phase0m_verdict_go_no_go_and_blocked():
    passing = {"repeat": True, "coverage": True}
    assert phase0m_verdict(5, 5, passing) == "PHASE0M_GO"
    assert phase0m_verdict(5, 5, {**passing, "coverage": False}) == "PHASE0M_NO_GO"
    assert phase0m_verdict(4, 5, passing) == "PHASE0M_ENGINEERING_BLOCKED"
