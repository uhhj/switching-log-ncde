import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];SPIKE=ROOT/'simulator-spikes'/'sofa-beam-spike';sys.path.insert(0,str(SPIKE/'src'));sys.path.insert(0,str(SPIKE/'scripts'))
from stage_c_coupled_correction_v2_4 import classify_trace,semantics_gate
from audit_stage_c_coupled_correction_v2_4_r1 import CORRECT_POINT_BASE,SOURCE_METRICS,SOURCE_TRACE,V24_CONFIG,WRONG_SPHERE_BASE,corrected_semantics,load_json,source_failure_signature
def test_source_failure_signature_is_exact_wrong_baseline_case():
 r=source_failure_signature(load_json(SOURCE_METRICS),load_json(CORRECT_POINT_BASE),load_json(V24_CONFIG));assert r['pass'];assert r['wrong_point_baseline_used_by_source_audit']=={'contact_m':0.,'alarm_m':.008};assert r['correct_point_baseline']=={'contact_m':.002,'alarm_m':.01};assert r['actual_sphere_effective_envelope']=={'contact_m':.002,'alarm_m':.01}
def test_corrected_point_baseline_passes_frozen_v24_semantics():
 s=load_json(SOURCE_METRICS);_,sem,eff=corrected_semantics(load_json(SOURCE_TRACE),load_json(V24_CONFIG),load_json(CORRECT_POINT_BASE),s['reaction_floor_c0_p99']);assert eff==.002 and sem['sphere_triangle']['pass'] and sem['response_stack']['pass'] and sem['pass']
def test_wrong_v23_sphere_baseline_reproduces_false_negative():
 s=load_json(SOURCE_METRICS);p=load_json(SOURCE_TRACE);c=load_json(V24_CONFIG);x=p['metadata']['collision_snapshot'];eff=sum(float(x[k]) for k in ('intersection_contact_distance_m','beam_default_radius_m','beam_model_contact_distance_m','fixture_model_contact_distance_m'));classes=classify_trace(p['records'],effective_contact_distance_m=eff,reaction_floor=float(s['reaction_floor_c0_p99']));r=semantics_gate(p,classes,c,load_json(WRONG_SPHERE_BASE));assert not r['sphere_triangle']['pass'] and r['response_stack']['pass'] and not r['pass']
