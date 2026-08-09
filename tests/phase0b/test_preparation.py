from __future__ import annotations

import numpy as np

from slncde.phase0b.preparation import (
    canonical_local_targets,
    endpoint_local_indices,
    local_geometry_passes,
    local_segment_diagnostics_from_state,
)


def test_endpoint_local_indices_follow_inward_order():
    assert endpoint_local_indices(20, 0, 5) == [0, 1, 2, 3, 4]
    assert endpoint_local_indices(20, 19, 5) == [19, 18, 17, 16, 15]


def test_canonical_targets_step_away_from_entry():
    targets = canonical_local_targets(
        [0.483, 0.0, 0.0], [1.0, 0.0, 0.0], 0.014, 5
    )
    np.testing.assert_allclose(np.diff(targets[:, 0]), -0.014)
    np.testing.assert_allclose(targets[:, 1:], 0.0)


def test_straight_local_segment_passes_and_entry_crossing_fails():
    positions = canonical_local_targets(
        [0.483, 0.0, 0.0], [1.0, 0.0, 0.0], 0.014, 5
    )
    velocities = np.zeros_like(positions)
    thresholds = {
        "entry_clearance_margin_m": 0.002,
        "alignment_cosine_min": 0.90,
        "max_local_speed_mps": 0.01,
    }
    diagnostics = local_segment_diagnostics_from_state(
        positions,
        velocities,
        [0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    )
    assert np.isclose(diagnostics["median_alignment_cosine"], 1.0)
    assert local_geometry_passes(diagnostics, thresholds)

    crossing = positions.copy()
    crossing[0, 0] = 0.501
    crossed = local_segment_diagnostics_from_state(
        crossing,
        velocities,
        [0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    )
    assert not local_geometry_passes(crossed, thresholds)
