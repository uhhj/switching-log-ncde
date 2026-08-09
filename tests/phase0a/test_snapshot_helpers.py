import numpy as np

from slncde.phase0a.snapshot import cable_rmse


def test_cable_rmse_uses_all_coordinates():
    first = np.zeros((2, 3), dtype=np.float64)
    second = np.ones((2, 3), dtype=np.float64) * 0.003
    assert cable_rmse(first, second) == 0.003
