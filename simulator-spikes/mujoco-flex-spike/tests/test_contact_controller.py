from pathlib import Path

import numpy as np

from src.controller import (
    BoundedCartesianImpedance,
    bounded_impedance_force,
    lateral_control_active,
)
from src.passage import PassageScene
from src.phase0m import (
    branch_y_target,
    resolve_c1_drive_protocol,
    resolve_phase0m,
)


ROOT = Path(__file__).resolve().parents[1]
C1 = ROOT / "configs" / "phase0m_c1_reachable_horizon.yaml"
C2 = ROOT / "configs" / "phase0m_c2_contact_control.yaml"


def test_bounded_impedance_force_clips_positive_and_negative_components():
    force, raw, saturated = bounded_impedance_force(
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0, -1.0],
        [0.0, 0.0],
        [40.0, 40.0],
        [1.0, 1.0],
        [2.0, 2.0],
    )
    np.testing.assert_allclose(raw, [40.0, -40.0])
    np.testing.assert_allclose(force, [2.0, -2.0])
    assert saturated.tolist() == [True, True]


def test_lateral_force_is_zero_before_geometry_trigger_and_bounded_after():
    assert not lateral_control_active(-0.0201, 0.0, 0.01, 2.0)
    assert lateral_control_active(-0.0200, 0.0, 0.01, 2.0)
    _, config, _ = resolve_phase0m(C2)
    controller = BoundedCartesianImpedance.from_config(
        [0.0, 0.0, 0.0], 0.1, config, 0.2
    )
    before = controller.command(0.0, [0.0, 0.0], [0.0, 0.0], True, False)
    after = controller.command(0.0, [0.0, 0.0], [0.0, 0.0], True, True)
    assert before[0][1] == 0.0
    assert 0.0 < after[0][1] <= 2.0


def test_c2_branch_targets_equal_c1_targets():
    _, c1, _ = resolve_phase0m(C1)
    _, c2, _ = resolve_phase0m(C2)
    radius = 0.005
    for branch in ("centered", "centered_repeat", "offset_medium", "offset_large"):
        assert branch_y_target(branch, radius, c2) == branch_y_target(
            branch, radius, c1
        )


def test_c2_drive_protocol_equals_c1_protocol():
    root1, c1, _ = resolve_phase0m(C1)
    root2, c2, _ = resolve_phase0m(C2)
    scene1 = PassageScene(root1 / c1["simulation"]["source_model"], c1)
    scene2 = PassageScene(root2 / c2["simulation"]["source_model"], c2)
    protocol1 = resolve_c1_drive_protocol(root1, scene1, c1)
    protocol2 = resolve_c1_drive_protocol(root2, scene2, c2)
    assert protocol2["command_distance_m"] == protocol1["command_distance_m"]
    assert protocol2["max_duration_s"] == protocol1["max_duration_s"]
