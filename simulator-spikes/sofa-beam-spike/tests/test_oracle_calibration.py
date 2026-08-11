import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from oracle import calibrate, progress_windows, sustained_true


def _traces(stick_speed=0.001, slip_speed=0.01, free_force=0.01, contact_force=0.2, free_progress=0.02, jam_progress=0.001):
    progress = lambda delta: np.arange(101, dtype=float) * delta / 50
    return {
        "free_calibration": {"reaction_force_proxy": [free_force] * 101},
        "stick_calibration": {"contact_reaction_force_proxy": [contact_force] * 101, "contact_tangent_speed_mps": [stick_speed] * 101},
        "slip_calibration": {"contact_reaction_force_proxy": [contact_force] * 101, "contact_tangent_speed_mps": [slip_speed] * 101},
        "free_forward_calibration": {"progress_m": progress(free_progress)},
        "jam_calibration": {"progress_m": progress(jam_progress)},
    }


def test_separated_calibration_places_speed_thresholds_inside_gap():
    result = calibrate(_traces(), dt_s=0.002, window_ms=100)
    assert result.passed
    assert 0.001 < result.values["stick_tangent_speed_max_mps"] < 0.01
    assert 0.001 < result.values["slip_tangent_speed_min_mps"] < 0.01


def test_overlapping_stick_and_slip_fails():
    result = calibrate(_traces(stick_speed=0.01, slip_speed=0.01), dt_s=0.002, window_ms=100)
    assert "stick_slip_speed_overlap" in result.failures


def test_overlapping_contact_reactions_fail():
    result = calibrate(_traces(free_force=0.2, contact_force=0.2), dt_s=0.002, window_ms=100)
    assert "contact_reaction_overlap" in result.failures


def test_overlapping_jam_and_free_progress_fail():
    result = calibrate(_traces(free_progress=0.001, jam_progress=0.001), dt_s=0.002, window_ms=100)
    assert "jam_free_progress_overlap" in result.failures


def test_dwell_requires_full_80_ms_and_progress_uses_100_ms_window():
    values = [False] + [True] * 39 + [False] + [True] * 40
    sustained = sustained_true(values, dt_s=0.002, minimum_ms=80)
    assert not sustained[1:40].any()
    assert sustained[41:].all()
    assert np.allclose(progress_windows(np.arange(101), 0.002, 100), 50)
