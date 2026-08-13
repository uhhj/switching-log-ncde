import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'src'));sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'scripts'))
from stage_c_collision_radius_v2_3 import *
from audit_stage_c_collision_radius_v2_3 import build_runsofa_command, timing_force_gate
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
def test_reaction_after_contact_loss_is_not_pre_geometric():
 _,c=configs();records=[{'step_end_time_s':.100,'measurement_active':False,'reaction_force_proxy':0.},{'step_end_time_s':.102,'measurement_active':True,'reaction_force_proxy':1.},{'step_end_time_s':.104,'measurement_active':True,'reaction_force_proxy':1.},{'step_end_time_s':.106,'measurement_active':True,'reaction_force_proxy':1.}];classes=[{'geometric_contact':False,'lcp_rows_present':False},{'geometric_contact':True,'lcp_rows_present':True},{'geometric_contact':True,'lcp_rows_present':True},{'geometric_contact':False,'lcp_rows_present':False}];assert reaction_gate(records,classes,c,reaction_floor=0)['pre_geometric_reaction_frames']==0
def test_reaction_before_geometric_fails():
 _,c=configs();records=[{'step_end_time_s':.100,'measurement_active':False,'reaction_force_proxy':1.},{'step_end_time_s':.102,'measurement_active':True,'reaction_force_proxy':1.},{'step_end_time_s':.104,'measurement_active':True,'reaction_force_proxy':1.}];classes=[{'geometric_contact':False,'lcp_rows_present':False},{'geometric_contact':True,'lcp_rows_present':True},{'geometric_contact':True,'lcp_rows_present':True}];r=reaction_gate(records,classes,c,reaction_floor=0);assert r['pre_geometric_reaction_frames']==1 and not r['causal_order_pass'] and not r['pass']
def test_runsofa_command_has_explicit_step_count():
 _,c=configs();cmd=build_runsofa_command(c);i=cmd.index('-n');assert cmd[i+1]=='500' and cmd[-1].endswith('stage_c_collision_radius_v2_3_scene.py')
def test_timing_and_force_readback_gate():
 _,c=configs();rs=[]
 for i in range(3):
  s=.098+i*.002;z=s<.100;f=[0.,0.,0.] if z else [0.,-.001,0.];f6=[[*f,0.,0.,0.]];rs.append({'step_start_time_s':s,'step_end_time_s':s+.002,'scheduled_force_n':f,'actual_applied_force_n':f,'force_field_indices_begin':[6],'force_field_indices_end':[6],'force_field_force_data_begin_n':f6,'force_field_force_data_end_n':f6,'zero_load_phase':z,'measurement_active':False})
 r=timing_force_gate(rs,c);assert r['pass'] and r['target_index_pass'] and r['max_force_readback_error_n']==0
