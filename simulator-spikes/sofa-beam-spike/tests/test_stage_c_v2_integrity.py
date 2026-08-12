import importlib.util
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from contact_native import contact_episode_stats_timestamps

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "audit_stage_c_stable_contact_v2.py"
SPEC = importlib.util.spec_from_file_location("stage_c", SCRIPT); stage_c = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(stage_c)

CONFIG={"beam":{"nodes":7,"young_modulus_pa":1e6,"poisson_ratio":.4,"radius_m":.002,"mass_density_kg_m3":1000.},"simulation":{"dt_s":.002},"integrity":{"time_tolerance_s":1e-10}}
def _record(i,serial=None,force=-.001):
    serial=i+1 if serial is None else serial
    return {"step_index":i,"bridge_frame_serial":serial,"native_contact_source_serial":serial,"step_start_time_s":i*.002,"step_end_time_s":(i+1)*.002,"scheduled_force_n":[0.,force,0.],"force_field_indices_begin":[6],"force_field_indices_end":[6],"force_field_force_data_begin_n":[[0.,force,0.,0.,0.,0.]],"force_field_force_data_end_n":[[0.,force,0.,0.,0.,0.]]}
def _payload(e=1e6): return {"metadata":{"material_snapshot":{"default_young_modulus_pa":[e]*6,"default_poisson_ratio":[.4]*6,"radius_m":[.002]*6,"mass_density_kg_m3":[1000.]*6}}}

def test_one_sample_timestamp_dwell_is_zero(): assert contact_episode_stats_timestamps([.002],[True])["longest_duration_s"]==0.
def test_two_samples_timestamp_dwell_is_2ms(): assert contact_episode_stats_timestamps([.002,.004],[True,True])["longest_duration_s"]==.002
def test_150_samples_timestamp_dwell_is_298ms(): assert np.isclose(contact_episode_stats_timestamps(np.arange(150)*.002,[True]*150)["longest_duration_s"],.298)
def test_151_samples_timestamp_dwell_is_300ms(): assert np.isclose(contact_episode_stats_timestamps(np.arange(151)*.002,[True]*151)["longest_duration_s"],.300)
def test_fresh_serial_sequence_passes(): assert stage_c.bridge_freshness([_record(i) for i in range(20)])["pass"]
def test_duplicate_serial_fails(): assert not stage_c.bridge_freshness([_record(0),_record(1,1)])["pass"]
def test_serial_gap_fails(): assert not stage_c.bridge_freshness([_record(0),_record(1,3)])["pass"]
def test_serial_regression_fails(): assert not stage_c.bridge_freshness([_record(0,2),_record(1,1)])["pass"]
def test_material_binding_correct_six_edge_values_passes(): assert stage_c.material_binding(_payload(),CONFIG)["pass"]
def test_material_default_e_is_caught(): assert not stage_c.material_binding(_payload(1e5),CONFIG)["pass"]
def test_force_exact_data_and_target_passes(): assert stage_c.force_provenance([_record(0)],6)["pass"]
def test_force_data_mismatch_fails():
    record = _record(0)
    record["force_field_force_data_end_n"] = [[0., 0., 0., 0., 0., 0.]]
    assert not stage_c.force_provenance([record], 6)["pass"]
def test_step_time_shift_fails():
    r=_record(0);r["step_start_time_s"],r["step_end_time_s"]=.002,.004
    assert not stage_c.timing_integrity([r],.002,1e-10)["pass"]
def test_stale_serial_would_block_perfect_science():
    records=[_record(0),_record(1,1)]; assert not stage_c.bridge_freshness(records)["pass"]
def test_c0_clean_reaction_baseline_passes():
    c=dict(CONFIG);c["baseline_sentinel"]={"minimum_clearance_m":.5}; payload={"records":[{**_record(0),"native_contact_count":0,"constraint_vector_size":0,"reaction_force_proxy":0.,"minimum_centerline_to_fixture_m":1.} ]}; assert stage_c._baseline(payload,c)["pass"]
def test_c0_nonempty_lcp_fails():
    c=dict(CONFIG);c["baseline_sentinel"]={"minimum_clearance_m":.5}; payload={"records":[{**_record(0),"native_contact_count":0,"constraint_vector_size":1,"reaction_force_proxy":0.,"minimum_centerline_to_fixture_m":1.} ]}; assert not stage_c._baseline(payload,c)["pass"]
