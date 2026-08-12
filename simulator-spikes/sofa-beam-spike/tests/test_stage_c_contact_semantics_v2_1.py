import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stage_c_contact_semantics import (
    classify_record,
    local_min_distance_gap_m,
    selected_geometric_contacts,
)


def _contact(beam_y: float, fixture_y: float = 0.0):
    return {
        "contact_id": 0,
        "beam_element_id": 1,
        "fixture_element_id": 0,
        "beam_contact_point_m": [0.1, beam_y, 0.0],
        "fixture_contact_point_m": [0.1, fixture_y, 0.0],
        "beam_outward_normal": [0.0, 1.0, 0.0],
    }


def _record(contacts, *, lcp_size=0, reaction=0.0):
    return {
        "native_contact_count": len(contacts),
        "native_contacts": contacts,
        "constraint_vector_size": lcp_size,
        "reaction_force_proxy": reaction,
        "step_end_time_s": 0.002,
    }


def test_three_mm_separation_with_two_mm_contact_distance_is_positive_gap():
    gap = local_min_distance_gap_m([0.0, 0.003, 0.0], [0.0, 0.0, 0.0], 0.002)
    assert np.isclose(gap, 0.001)


def test_two_mm_separation_is_zero_gap():
    gap = local_min_distance_gap_m([0.0, 0.002, 0.0], [0.0, 0.0, 0.0], 0.002)
    assert np.isclose(gap, 0.0)


def test_one_mm_separation_is_negative_gap():
    gap = local_min_distance_gap_m([0.0, 0.001, 0.0], [0.0, 0.0, 0.0], 0.002)
    assert np.isclose(gap, -0.001)


def test_proximity_detection_is_not_geometric_contact():
    out = classify_record(_record([_contact(0.003)], lcp_size=3, reaction=0.0), effective_contact_distance_m=0.002, reaction_floor=0.0)
    assert out["native_detection"]
    assert out["proximity_only"]
    assert not out["geometric_contact"]
    assert out["lcp_rows_present"]
    assert not out["reaction_active"]
    assert not out["load_bearing_contact"]


def test_geometric_contact_and_reaction_yield_load_bearing_contact():
    out = classify_record(_record([_contact(0.001)], lcp_size=3, reaction=1e-4), effective_contact_distance_m=0.002, reaction_floor=0.0)
    assert out["native_detection"]
    assert out["geometric_contact"]
    assert out["reaction_active"]
    assert out["load_bearing_contact"]


def test_mixed_detection_frame_counts_only_nonpositive_gap_as_geometric():
    record = _record([_contact(0.003), _contact(0.001)])
    out = classify_record(record, effective_contact_distance_m=0.002, reaction_floor=0.0)
    assert out["native_detection_count"] == 2
    assert out["geometric_contact_count"] == 1
    selected = selected_geometric_contacts([record], [out], effective_contact_distance_m=0.002)
    assert len(selected) == 1
    assert selected[0]["local_min_distance_gap_m"] < 0.0


def test_native_count_mismatch_is_rejected():
    record = _record([_contact(0.003)])
    record["native_contact_count"] = 2
    try:
        classify_record(record, effective_contact_distance_m=0.002, reaction_floor=0.0)
    except ValueError as exc:
        assert "mismatch" in str(exc)
    else:
        raise AssertionError("expected ValueError")
