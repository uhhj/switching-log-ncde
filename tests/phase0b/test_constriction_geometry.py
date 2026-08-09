import numpy as np

from slncde.phase0b.metrics import (
    contact_reachability_proxy,
    required_passage_progress,
)


def test_passage_progress_and_contact_reachability():
    config = {
        "motion": {"staging_before_entry_m": 0.017},
        "fixture": {"funnel_length_m": 0.025, "throat_length_m": 0.020},
        "analysis": {"exit_margin_m": 0.005},
    }
    assert np.isclose(required_passage_progress(config), 0.067)
    for offset in (0.0015, 0.0035, 0.0060):
        assert contact_reachability_proxy(offset, 0.005, 0.012) > 0.0


def test_constriction_gap_order():
    outer_gap, throat_gap, bead_width = 0.034, 0.012, 0.010
    assert outer_gap > throat_gap > bead_width
