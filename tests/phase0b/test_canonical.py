from __future__ import annotations

from pathlib import Path

import numpy as np

from slncde.phase0b.runner import (
    _branch_targets,
    canonical_frame_arrays,
    choose_active_endpoint_from_positions,
)


def _config():
    return {
        "canonical_frame": {
            "mode": "canonical",
            "entry_center_xy_m": [0.50, 0.00],
            "insertion_axis_xy": [1.0, 0.0],
        },
        "motion": {
            "insertion_distance_m": 0.060,
            "nominal_lateral_offset_m": 0.0,
            "slide_lateral_offset_m": 0.0035,
            "jam_lateral_offset_m": 0.0080,
        },
    }


def test_canonical_frame_is_independent_of_endpoints():
    first = np.asarray([[0.3, -0.2, 0.01], [0.4, 0.1, 0.01]])
    second = np.asarray([[0.7, 0.3, 0.01], [0.6, -0.3, 0.01]])
    frame_a = canonical_frame_arrays(_config(), first)
    frame_b = canonical_frame_arrays(_config(), second)
    for value_a, value_b in zip(frame_a, frame_b):
        np.testing.assert_array_equal(value_a, value_b)


def test_endpoint_nearest_staging_is_selected():
    positions = np.asarray(
        [[0.31, -0.20, 0.01], [0.40, 0.0, 0.01], [0.49, 0.01, 0.01]]
    )
    assert choose_active_endpoint_from_positions(positions, [0.483, 0.0]) == 2


def test_branch_target_invariants():
    config = _config()
    staging = np.asarray([0.483, 0.0, 0.01])
    insertion = np.asarray([1.0, 0.0, 0.0])
    lateral = np.asarray([0.0, 1.0, 0.0])
    common = {
        "ee_position": np.asarray([0.483, 0.0, 0.02]),
        "ee_orientation": np.asarray([0.0, 0.0, 0.0, 1.0]),
    }
    targets = _branch_targets(
        config, staging, insertion, lateral, common, staging
    )

    assert targets["nominal"] is targets["nominal_repeat"]
    for branch, offset in (
        ("nominal", 0.0),
        ("slide_probe", 0.0035),
        ("jam_probe", 0.0080),
    ):
        align = targets[branch]["align_endpoint"]
        insert = targets[branch]["insert_endpoint"]
        np.testing.assert_allclose(align - staging, offset * lateral)
        np.testing.assert_allclose(insert - align, 0.060 * insertion)


def test_canonical_spawn_origin_matches_advance_before_create_semantics():
    from slncde.phase0a.runner import load_simulator

    repo_root = Path(__file__).resolve().parents[2]
    load_simulator(repo_root)
    from ravens.tasks.slncde_fixture_channel_cable import (
        canonical_cable_start_position,
    )

    count = 24
    spacing = 0.01 * np.sqrt(2.0)
    target = np.asarray([0.483, 0.0, 0.005])
    origin = canonical_cable_start_position(target, count, spacing)
    predicted_last = origin + count * spacing * np.asarray([1.0, 0.0, 0.0])
    np.testing.assert_allclose(predicted_last, target)
