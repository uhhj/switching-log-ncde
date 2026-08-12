import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stage_c_contact_loss_morphology import (
    VERDICT_AMBIGUOUS, VERDICT_CONTACT_MIGRATION, VERDICT_CROSS_THROUGH,
    VERDICT_FIXTURE_EDGE_ESCAPE, VERDICT_REACTION_COLLAPSE,
    VERDICT_REBOUND_ESCAPE, classify_contact_loss_morphology,
    frame_wall_geometry, trailing_true_run,
)


def _config():
    return {"contact_wall": {"plane_y_m": 0.0, "x_min_m": 0.08, "x_max_m": 0.18, "z_half_extent_m": 0.05}, "simulation": {"dt_s": 0.002}, "gates": {"maximum_fixture_plane_error_m": 1e-6, "maximum_native_lcp_onset_lag_steps": 2, "minimum_observed_contact_dwell_ms": 300}}


def _record(time_s, ys, xs=None):
    if xs is None: xs = [0.08 + 0.01 * i for i in range(len(ys))]
    return {"step_end_time_s": float(time_s), "beam_node_positions_m": [[float(x), float(y), 0.0] for x, y in zip(xs, ys)]}


def _classification(*, native=False, geometric=False, reaction=False):
    return {"native_detection": bool(native), "geometric_contact": bool(geometric), "reaction_active": bool(reaction), "load_bearing_contact": bool(geometric and reaction)}


def _audit(records, classifications):
    return classify_contact_loss_morphology(records, classifications, config=_config(), effective_contact_distance_m=0.002)


def test_trailing_true_run_is_terminal_episode_not_global_maximum():
    assert trailing_true_run([True, True, True, False, True, True]) == 2
    assert trailing_true_run([True, True, True, False]) == 0


def test_frame_geometry_uses_frozen_effective_envelope():
    out = frame_wall_geometry(_record(0.1, [0.003, 0.001]), wall=_config()["contact_wall"], effective_contact_distance_m=0.002, plane_tol_m=1e-6)
    assert out["footprint_node_count"] == 2 and out["within_effective_envelope"]
    assert abs(out["surrogate_gap_m"] + 0.001) < 1e-12


def test_gradual_cross_through_deadband_is_detected():
    records = [_record(0.002, [0.003]), _record(0.004, [0.001]), _record(0.006, [0.5e-6]), _record(0.008, [-0.5e-6]), _record(0.010, [-0.003])]
    classifications = [_classification(), _classification(native=True, geometric=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True, reaction=True), _classification()]
    out = _audit(records, classifications)
    assert out["verdict"] == VERDICT_CROSS_THROUGH and out["crossing_event_count"] >= 1


def test_leaving_and_reentering_footprint_does_not_fake_cross_through():
    records = [_record(0.002, [0.003]), _record(0.004, [0.001]), _record(0.006, [0.001], xs=[0.30]), _record(0.008, [-0.003])]
    out = _audit(records, [_classification(), _classification(native=True, geometric=True, reaction=True), _classification(), _classification()])
    assert out["crossing_event_count"] == 0


def test_geometric_first_is_diagnostic_onset_even_when_load_bearing_is_later():
    records = [_record(0.002, [0.003]), _record(0.004, [0.001]), _record(0.006, [-0.003]), _record(0.008, [-0.003])]
    out = _audit(records, [_classification(), _classification(native=True, geometric=True), _classification(native=True, geometric=True, reaction=True), _classification()])
    assert out["formal_onsets"]["diagnostic_onset_index"] == 1
    assert out["formal_onsets"]["diagnostic_onset_source"] == "geometric_contact"
    assert out["verdict"] == VERDICT_CROSS_THROUGH


def test_recovered_native_loss_cannot_trigger_contact_migration():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(6)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True), _classification(), _classification(), _classification(), _classification(native=True, geometric=True, reaction=True), _classification()])
    assert not out["candidate_flags"]["contact_migration"] and out["final_loss_no_native_near_wall_terminal_run_steps"] == 1


def test_short_terminal_native_loss_does_not_trigger_contact_migration():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(4)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True), _classification(), _classification(), _classification()])
    assert not out["candidate_flags"]["contact_migration"]
    assert abs(out["final_loss_no_native_near_wall_terminal_duration_s"] - 0.004) < 1e-12
    assert out["weak_minimum_dwell_s"] == 0.3 and out["weak_mechanism_duration_semantics"] == "timestamp_span_end_minus_start"


def test_150_terminal_loss_samples_are_298ms_and_do_not_trigger_migration():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(151)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True)] + [_classification() for _ in range(150)])
    assert not out["candidate_flags"]["contact_migration"] and out["final_loss_no_native_near_wall_terminal_run_steps"] == 150
    assert abs(out["final_loss_no_native_near_wall_terminal_duration_s"] - 0.298) < 1e-12


def test_151_terminal_loss_samples_span_300ms_and_trigger_migration():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(152)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True)] + [_classification() for _ in range(151)])
    assert out["verdict"] == VERDICT_CONTACT_MIGRATION and out["final_loss_no_native_near_wall_terminal_run_steps"] == 151
    assert abs(out["final_loss_no_native_near_wall_terminal_duration_s"] - 0.300) < 1e-12


def test_recovered_reaction_gap_cannot_trigger_reaction_collapse():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(7)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True, reaction=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True)])
    assert not out["candidate_flags"]["reaction_collapse"] and out["final_geometric_without_reaction_terminal_run_steps"] == 2


def test_short_final_reaction_gap_does_not_trigger_reaction_collapse():
    records = [_record(0.002 * (i + 1), [0.001]) for i in range(4)]
    out = _audit(records, [_classification(native=True, geometric=True, reaction=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True), _classification(native=True, geometric=True)])
    assert not out["candidate_flags"]["reaction_collapse"]
    assert abs(out["final_geometric_without_reaction_terminal_duration_s"] - 0.004) < 1e-12


def test_150_and_151_reactionless_samples_use_timestamp_span():
    records150 = [_record(0.002 * (i + 1), [0.001]) for i in range(151)]
    out150 = _audit(records150, [_classification(native=True, geometric=True, reaction=True)] + [_classification(native=True, geometric=True) for _ in range(150)])
    assert not out150["candidate_flags"]["reaction_collapse"]
    assert abs(out150["final_geometric_without_reaction_terminal_duration_s"] - 0.298) < 1e-12
    records151 = [_record(0.002 * (i + 1), [0.001]) for i in range(152)]
    out151 = _audit(records151, [_classification(native=True, geometric=True, reaction=True)] + [_classification(native=True, geometric=True) for _ in range(151)])
    assert out151["verdict"] == VERDICT_REACTION_COLLAPSE
    assert abs(out151["final_geometric_without_reaction_terminal_duration_s"] - 0.300) < 1e-12


def test_rebound_and_edge_escape_remain_distinct():
    rebound = _audit([_record(0.002, [0.003]), _record(0.004, [0.001]), _record(0.006, [0.0035])], [_classification(), _classification(native=True, geometric=True, reaction=True), _classification()])
    assert rebound["verdict"] == VERDICT_REBOUND_ESCAPE
    edge = _audit([_record(0.002, [0.003]), _record(0.004, [0.001]), _record(0.006, [0.001], xs=[0.30])], [_classification(), _classification(native=True, geometric=True, reaction=True), _classification()])
    assert edge["verdict"] == VERDICT_FIXTURE_EDGE_ESCAPE


def test_no_formal_onset_is_ambiguous():
    out = _audit([_record(0.002, [0.003]), _record(0.004, [0.003])], [_classification(), _classification()])
    assert out["verdict"] == VERDICT_AMBIGUOUS
    assert out["offline_geometry_surrogate_is_formal_contact_signal"] is False
