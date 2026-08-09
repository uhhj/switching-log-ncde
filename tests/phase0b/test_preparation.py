from __future__ import annotations

import numpy as np

from slncde.phase0b.preparation import (
    canonical_active_endpoint_index,
    endpoint_local_indices,
    local_geometry_passes,
    local_segment_diagnostics_from_state,
)


def test_endpoint_local_indices_follow_inward_order():
    assert endpoint_local_indices(20, 0, 5) == [0, 1, 2, 3, 4]
    assert endpoint_local_indices(20, 19, 5) == [19, 18, 17, 16, 15]
    assert canonical_active_endpoint_index(20) == 19


def straight_positions():
    return np.asarray(
        [[0.483 - rank * 0.014, 0.0, 0.005] for rank in range(5)]
    )


def test_straight_local_segment_passes_and_entry_crossing_fails():
    positions = straight_positions()
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
    assert np.isclose(diagnostics["max_local_spacing_error_m"], 0.0)
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


def test_spacing_diagnostic_rejects_perturbed_bead():
    positions = straight_positions()
    velocities = np.zeros_like(positions)
    thresholds = {
        "entry_clearance_margin_m": 0.002,
        "alignment_cosine_min": 0.90,
        "max_local_speed_mps": 0.01,
        "max_local_spacing_error_m": 0.002,
    }
    straight = local_segment_diagnostics_from_state(
        positions,
        velocities,
        [0.5, 0.0, 0.005],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.014,
    )
    assert np.isclose(straight["median_alignment_cosine"], 1.0)
    assert np.isclose(straight["max_local_spacing_error_m"], 0.0)
    assert local_geometry_passes(straight, thresholds)

    perturbed = positions.copy()
    perturbed[2, 1] += 0.010
    diagnostics = local_segment_diagnostics_from_state(
        perturbed,
        velocities,
        [0.5, 0.0, 0.005],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.014,
    )
    assert diagnostics["max_local_spacing_error_m"] > 0.002
    assert not local_geometry_passes(diagnostics, thresholds)
