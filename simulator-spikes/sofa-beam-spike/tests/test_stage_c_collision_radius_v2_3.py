import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'src'));sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'scripts'))
from stage_c_collision_radius_v2_3 import *
from audit_stage_c_collision_radius_v2_3 import existing_evidence_paths
SPIKE=ROOT/'simulator-spikes'/'sofa-beam-spike'
def configs():return json.loads((SPIKE/'configs'/'stage_c_stable_contact_v2_2_fixture_extent.json').read_text()),json.loads((SPIKE/'configs'/'stage_c_collision_radius_v2_3.json').read_text())
def contact(value):return {'native_contact_count':1,'native_contacts':[{'beam_element_id':6,'fixture_element_id':0,'beam_contact_point_m':[.12,.002,0],'fixture_contact_point_m':[.12,0,0],'beam_outward_normal':[0,1,0],'detection_value_m':value}],'constraint_vector_size':0,'reaction_force_proxy':0}
def test_only_compensated_values_change():
 a,b=configs();r=compensated_intervention_gate(a,b);assert r['pass'];assert r['diff_paths']==['simulation.alarm_distance_m','simulation.contact_distance_m']
def test_sphere_effective_distances_preserve_point_values():
 r=effective_sphere_distances(sphere_radius_m=.002,global_contact_distance_m=0,global_alarm_distance_m=.008,sphere_model_contact_distance_m=0,triangle_model_contact_distance_m=0);assert r=={'effective_contact_distance_m':.002,'effective_alarm_distance_m':.01}
def test_semantics_requires_a_raw_detection_sample():
 a,b=configs();p={'metadata':{'collision_snapshot':{'beam_collision_model':'SphereCollisionModel','beam_default_radius_m':.002,'beam_list_radius_m':[.002]*7,'beam_model_contact_distance_m':0,'fixture_model_contact_distance_m':0,'intersection_contact_distance_m':0,'intersection_alarm_distance_m':.008}}};assert not semantics_gate(p,[{'detection_value_residuals_m':[]}],b,a)['pass']
def test_raw_detection_value_is_formal_contact_authority():
 assert classify_record(contact(.001),effective_contact_distance_m=.002,reaction_floor=0)['proximity_only'];assert classify_record(contact(0),effective_contact_distance_m=.002,reaction_floor=0)['geometric_contact']
def test_timestamp_span_dwell_is_not_sample_count():
 _,c=configs();rs=[{'step_end_time_s':(i+1)*.002,'measurement_active':True} for i in range(150)];q=contact_gate(rs,[{'geometric_contact':True} for _ in rs],c);assert q['episode']['longest_duration_s']==.298 and not q['pass']
def test_existing_evidence_is_detected(tmp_path):
 ps=tuple(tmp_path/x for x in ('trace','metrics','report'));assert existing_evidence_paths(ps)==[];ps[1].write_text('{}');assert existing_evidence_paths(ps)==[ps[1]]
