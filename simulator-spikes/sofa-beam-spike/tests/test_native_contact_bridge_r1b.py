import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from contact_native import point_tangent_velocity, unpack_bridge_frame


def _frame(**overrides):
    frame = {
        "contact_count": 1,
        "contact_ids": [-17],
        "beam_element_ids": [3],
        "fixture_element_ids": [1],
        "beam_contact_points": [[0.1, 0.0, 0.0]],
        "fixture_contact_points": [[0.1, 0.0, 0.0]],
        "beam_outward_normals": [[0.0, 1.0, 0.0]],
        "detection_values": [-0.001],
    }
    frame.update(overrides)
    return frame


def test_unpack_valid_bridge_frame_preserves_native_identity():
    result = unpack_bridge_frame(_frame())
    assert result["count"] == 1
    assert result["contact_ids"].tolist() == [-17]
    assert result["beam_ids"].tolist() == [3]
    assert np.isfinite(result["beam_points"]).all()
    assert np.isclose(np.linalg.norm(result["normals"][0]), 1.0)


def test_unpack_rejects_inconsistent_flat_xyz_length():
    with pytest.raises(ValueError):
        unpack_bridge_frame(_frame(contact_count=2, beam_contact_points=[0.0, 0.0, 0.0]))


def test_native_primitive_id_selects_contact_velocity_not_nearest_plane():
    velocities = [[99.0, 0.0, 0.0], [98.0, 0.0, 0.0], [97.0, 0.0, 0.0], [2.5, 0.0, 0.0]]
    assert point_tangent_velocity(velocities, 3, [1.0, 0.0, 0.0]) == 2.5


def test_formal_bridge_frame_has_no_proxy_velocity_field():
    result = unpack_bridge_frame(_frame())
    assert "nearest_plane" not in result
    assert "endpoint_velocity" not in result
