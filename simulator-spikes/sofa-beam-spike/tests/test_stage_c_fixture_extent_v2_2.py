import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
SCRIPT = SPIKE / "scripts" / "audit_stage_c_fixture_extent_v2_2.py"
BASE = SPIKE / "configs" / "stage_c_stable_contact_v2.json"
CANDIDATE = SPIKE / "configs" / "stage_c_stable_contact_v2_2_fixture_extent.json"


def module():
    spec = importlib.util.spec_from_file_location("audit_v22", SCRIPT)
    value = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(value)
    return value


def gate_config(time_tolerance_s=1e-10):
    return {"gates": {"minimum_measurement_contact_occupancy": .90, "minimum_observed_contact_dwell_ms": 300, "maximum_fixture_plane_error_m": 1e-6, "minimum_contact_lcp_fraction": .90, "minimum_contact_positive_reaction_fraction": .90, "maximum_native_lcp_onset_lag_steps": 2}, "integrity": {"time_tolerance_s": float(time_tolerance_s)}, "simulation": {"dt_s": .002}, "beam": {"nodes": 7}, "tail_constraint": {"node_index": 0, "fixed_directions": [1, 1, 1, 0, 0, 0]}, "contact_wall": {"x_min_m": 0., "x_max_m": .18, "z_half_extent_m": .05, "plane_y_m": 0.}}


def test_single_variable_config_change():
    m=module(); result=m.config_delta(json.loads(BASE.read_text()),json.loads(CANDIDATE.read_text()))
    assert result["pass"] and result["diff_paths"] == ["contact_wall.x_min_m"]
    assert result["old_x_min_m"] == .08 and result["new_x_min_m"] == result["nominal_beam_x_min_m"] == 0.


def test_second_config_change_is_rejected():
    m=module(); base=json.loads(BASE.read_text()); candidate=json.loads(CANDIDATE.read_text()); candidate["contact_wall"]["z_half_extent_m"]=.06
    assert set(m.config_delta(base,candidate)["diff_paths"]) == {"contact_wall.x_min_m","contact_wall.z_half_extent_m"}


def _record(index): return {"step_end_time_s": (index+1)*.002, "measurement_active": True}
def _cls(value): return {"geometric_contact": bool(value)}


def test_timestamp_span_dwell_boundaries():
    m=module(); out150=m.contact_gate([_record(i) for i in range(150)],[_cls(True)]*150,gate_config()); out151=m.contact_gate([_record(i) for i in range(151)],[_cls(True)]*151,gate_config())
    assert abs(out150["episode"]["longest_duration_s"]-.298)<1e-12 and not out150["pass"]
    assert abs(out151["episode"]["longest_duration_s"]-.300)<1e-12 and out151["pass"]


def test_contact_gate_uses_configured_time_tolerance():
    m=module(); records=[{"step_end_time_s":.1,"measurement_active":True},{"step_end_time_s":.39999999995,"measurement_active":True}]; classes=[_cls(True),_cls(True)]
    assert m.contact_gate(records,classes,gate_config(1e-10))["pass"]
    assert not m.contact_gate(records,classes,gate_config(1e-12))["pass"]


def _edge_record(*, x_mobile=.10, tail_x=0., native_ids=()):
    positions=[[tail_x,.003,0.]]+[[x_mobile,.003,0.] for _ in range(6)]
    return {"step_end_time_s":.1,"beam_node_positions_m":positions,"native_contact_count":len(tuple(native_ids)),"native_contacts":[{"beam_element_id":int(node)} for node in native_ids]}


def test_mobile_fixture_gate_excludes_fixed_tail():
    m=module(); out=m.mobile_fixture_edge_gate([_edge_record(x_mobile=.10,native_ids=(1,)),_edge_record(x_mobile=.30,native_ids=(0,))],[{"geometric_contact":True},{"geometric_contact":False}],gate_config())
    assert out["excluded_translation_fixed_node_ids"] == [0] and out["mobile_node_ids"] == [1,2,3,4,5,6]
    assert out["terminal_all_node_footprint_node_count_diagnostic"] == 1 and out["terminal_all_native_detection_diagnostic"]
    assert not out["terminal_mobile_native_detection"] and out["fixture_edge_escape"]


def _reaction_record(time_s,reaction): return {"step_end_time_s":float(time_s),"measurement_active":True,"reaction_force_proxy":float(reaction)}
def _reaction_class(*,geometric,reaction,lcp=True): return {"geometric_contact":bool(geometric),"reaction_active":bool(reaction),"lcp_rows_present":bool(lcp)}


def test_reaction_before_geometric_must_fail():
    m=module(); records=[_reaction_record(.122,1),_reaction_record(.124,1),_reaction_record(.126,1)]; classes=[_reaction_class(geometric=False,reaction=True),_reaction_class(geometric=True,reaction=True),_reaction_class(geometric=True,reaction=True)]; out=m.reaction_gate(records,classes,gate_config(),0.)
    assert abs(out["reaction_minus_geometric_onset_s"]+.002)<1e-12 and not out["causal_order_pass"] and out["pre_geometric_reaction_frames"]==1 and not out["pass"]


def test_reaction_after_geometric_within_lag_passes():
    m=module(); records=[_reaction_record(.122+.002*i,0 if i==0 else 1) for i in range(10)]; classes=[_reaction_class(geometric=True,reaction=i>0) for i in range(10)]; out=m.reaction_gate(records,classes,gate_config(),0.)
    assert abs(out["reaction_minus_geometric_onset_s"]-.002)<1e-12 and out["causal_order_pass"] and out["upper_lag_pass"] and out["pass"]


def test_verdict_precedence():
    m=module(); edge=lambda v=False:{"fixture_edge_escape":v}; morph=lambda v=0:{"crossing_event_count":v}
    assert m.choose_verdict(False,True,edge(),morph(),True,True,True)==m.V_INST
    assert m.choose_verdict(True,False,edge(),morph(),True,True,True)==m.V_ZERO
    assert m.choose_verdict(True,True,edge(True),morph(1),True,True,True)==m.V_EDGE
    assert m.choose_verdict(True,True,edge(),morph(1),True,True,True)==m.V_CROSS
    assert m.choose_verdict(True,True,edge(),morph(),False,False,False)==m.V_CONTACT
    assert m.choose_verdict(True,True,edge(),morph(),True,False,False)==m.V_REACTION
    assert m.choose_verdict(True,True,edge(),morph(),True,True,False)==m.V_GEOM
    assert m.choose_verdict(True,True,edge(),morph(),True,True,True)==m.V_PASS
