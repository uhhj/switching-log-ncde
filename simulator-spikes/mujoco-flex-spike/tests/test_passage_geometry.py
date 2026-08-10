import numpy as np

from src.passage import funnel_wall_pose, geometry_from_config, passage_success


def _config():
    return {
        "geometry": {
            "funnel_entry_half_gap_r": 2.4,
            "throat_half_gap_r": 1.18,
            "funnel_length_r": 10.0,
            "throat_length_r": 10.0,
            "wall_thickness_r": 1.0,
            "wall_height_r": 4.0,
            "pre_entry_distance_r": 8.0,
            "exit_margin_r": 3.0,
        }
    }


def test_passage_geometry_is_symmetric_and_finite():
    geometry = geometry_from_config(0.005, _config())
    assert geometry.entry_half_gap_m > geometry.throat_half_gap_m
    upper_midpoint, upper_length, upper_yaw = funnel_wall_pose(
        0.0,
        geometry.funnel_exit_x_m,
        geometry.entry_half_gap_m,
        geometry.throat_half_gap_m,
        geometry.wall_thickness_m,
    )
    lower_midpoint, lower_length, lower_yaw = funnel_wall_pose(
        0.0,
        geometry.funnel_exit_x_m,
        -geometry.entry_half_gap_m,
        -geometry.throat_half_gap_m,
        geometry.wall_thickness_m,
    )
    assert np.isfinite(upper_length) and upper_length > 0.0
    np.testing.assert_allclose(lower_midpoint, upper_midpoint * [1.0, -1.0])
    assert lower_length == upper_length
    assert lower_yaw == -upper_yaw


def test_leading_four_success_requires_all_vertices():
    crossed = np.asarray([[0.12, 0.0, 0.0]] * 4)
    assert passage_success(crossed, 0.115)
    crossed[-1, 0] = 0.11
    assert not passage_success(crossed, 0.115)
