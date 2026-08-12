import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "audit_stage_b_reaction_proxy.py"
SPEC = importlib.util.spec_from_file_location("stage_b_audit", SCRIPT)
stage_b = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stage_b)


CONFIG = {
    "simulation": {"steps": 3},
    "gates": {"tail_translation_max_m": 1e-9, "minimum_sentinel_clearance_m": 0.5},
}


def _record(serial, *, contact=0, vector_size=0, reaction=0.0, tail=0.0, sentinel=1.0):
    return {
        "measurement_active": True,
        "bridge_ready": True,
        "bridge_frame_serial": serial,
        "native_contact_count": contact,
        "constraint_vector_size": vector_size,
        "reaction_force_proxy": reaction,
        "tail_translation_displacement_m": tail,
        "tip_displacement_m": 0.001,
        "minimum_centerline_to_sentinel_m": sentinel,
        "command_active": True,
    }


def _result(**changes):
    records = [_record(1), _record(2), _record(3)]
    for record in records:
        record.update(changes)
    return stage_b.analyze_trace({"records": records}, CONFIG)


def test_clean_synthetic_trace_is_clean():
    assert _result()["verdict"] == "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CLEAN"


def test_nonempty_vector_is_contaminated_without_contact():
    assert _result(constraint_vector_size=1)["verdict"] == "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CONTAMINATED"


def test_nonzero_reaction_is_contaminated_without_contact():
    assert _result(reaction_force_proxy=0.1)["verdict"] == "PHASE0S_SOFA_STAGE_B_REACTION_PROXY_CONTAMINATED"


def test_native_contact_invalidates_sentinel_control():
    assert _result(native_contact_count=1)["verdict"] == "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED"


def test_tail_motion_invalidates_anchor_control():
    assert _result(tail_translation_displacement_m=2e-9)["verdict"] == "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED"


def test_small_sentinel_clearance_invalidates_control():
    assert _result(minimum_centerline_to_sentinel_m=0.5)["verdict"] == "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED"


def test_nonadvancing_serial_invalidates_control():
    records = [_record(1), _record(1), _record(1)]
    assert stage_b.analyze_trace({"records": records}, CONFIG)["verdict"] == "PHASE0S_SOFA_STAGE_B_ENGINEERING_BLOCKED"
