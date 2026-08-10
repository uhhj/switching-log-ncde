import numpy as np
import pytest

from slncde.phase0.capability_probe import (
    aggregate_contact_points,
    capability_verdict,
    progress_over_window,
)


def _contact(normal_force=2.0, friction_1=0.3, friction_2=0.4):
    point = [None] * 14
    point[7] = [0.0, 0.0, 1.0]
    point[9] = normal_force
    point[10] = friction_1
    point[11] = [1.0, 0.0, 0.0]
    point[12] = friction_2
    point[13] = [0.0, 1.0, 0.0]
    return point


def test_contact_aggregation_zero_sum_and_tangent_projection():
    empty = aggregate_contact_points([], [1.0, 2.0, 3.0])
    assert empty["contact_count"] == 0
    assert empty["normal_force_sum"] == 0.0

    result = aggregate_contact_points([_contact(), _contact(normal_force=3.0)], [3.0, 4.0, 5.0])
    assert result["normal_force_sum"] == pytest.approx(5.0)
    assert result["friction_force_sum"] == pytest.approx(1.0)
    assert result["tangential_speed"] == pytest.approx(5.0)


@pytest.mark.parametrize(
    ("matrix", "expected"),
    [
        ({name: {"raw": True, "label": True} for name in ("stick", "slip", "jam")}, "PHASE0C0_LABELS_CAPABLE_TASK_REPRESENTATION_SUSPECT"),
        ({"stick": {"raw": True, "label": False}, "slip": {"raw": True, "label": True}, "jam": {"raw": True, "label": True}}, "PHASE0C0_ORACLE_DEFINITION_FAIL"),
        ({"stick": {"raw": False, "label": False}, "slip": {"raw": True, "label": True}, "jam": {"raw": True, "label": True}}, "PHASE0C0_SIM_CONTACT_CAPABILITY_FAIL"),
        ({"stick": {"raw": True, "label": True}, "slip": {"raw": True, "label": True}, "jam": {"raw": True, "label": False}}, "PHASE0C0_JAM_DEFINITION_FAIL"),
    ],
)
def test_capability_verdict(matrix, expected):
    assert capability_verdict(matrix) == expected


def test_progress_window_matches_t2_semantics():
    progress = np.asarray([0.0, 0.1, 0.2, 0.25, 0.26])
    np.testing.assert_allclose(progress_over_window(progress, 2), [0.0, 0.0, 0.2, 0.15, 0.06])
