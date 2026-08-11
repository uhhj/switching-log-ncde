import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from contact_native import (
    contact_episode_stats,
    derived_excitation,
    find_breakaway,
    line_contact_velocity,
    point_tangent_velocity,
    reaction_proxy_clean,
)


def test_point_primitive_velocity_projection():
    assert point_tangent_velocity([[0, 0, 0], [3, 4, 0]], 1, [1, 0, 0]) == 3.0


def test_line_primitive_contact_velocity_interpolates_at_contact_point():
    velocity = line_contact_velocity([[0, 0, 0], [2, 0, 0]], [[0, 0, 0], [4, 0, 0]], 0, 1, [0.5, 0, 0])
    assert np.allclose(velocity, [1, 0, 0])


def test_episode_duration_uses_native_contact_mask():
    stats = contact_episode_stats([False, True, True, False, True, True, True], 0.002)
    assert stats["episodes"] == 2
    assert stats["longest_duration_s"] == 0.006


def test_anchored_free_contamination_requires_empty_contact_and_constraint_signal():
    assert reaction_proxy_clean([0, 0], [0, 0], [0.0, 0.0])
    assert not reaction_proxy_clean([0, 0], [3, 0], [0.0, 0.0])


def test_breakaway_uses_one_50ms_native_contact_window():
    result = find_breakaway([0.001] * 30, [0.01] * 30, [1] * 30, [True] * 30, dt_s=0.002, window_ms=50, radius_m=0.002)
    assert result is not None
    assert result["travel_m"] >= 0.0005


def test_breakaway_missing_contact_window_returns_none():
    result = find_breakaway([0.001] * 30, [0.01] * 30, [0] * 30, [True] * 30, dt_s=0.002, window_ms=50, radius_m=0.002)
    assert result is None


def test_derived_stick_and_slip_forces_are_fixed_ratios():
    assert derived_excitation(0.0008) == (0.0004, 0.001)


def test_formal_metric_does_not_use_proxy_values():
    result = find_breakaway([0.001] * 30, [0.0] * 30, [1] * 30, [True] * 30, dt_s=0.002, window_ms=50, radius_m=0.002)
    assert result is None
