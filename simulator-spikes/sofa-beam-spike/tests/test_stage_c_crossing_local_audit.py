import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'src'))
from stage_c_crossing_local_audit import V_LOCAL_CONTACT_GLOBAL_ACTIVE,V_LOCAL_CONTACT_GLOBAL_INACTIVE,V_LOCAL_CONTACT_ABSENT,audit_crossings
TIME_TOL=1e-10;DIST_TOL=1e-6
def record(step,time,y,*,detection=None,normal_y=1.,lcp=0,reaction=0.):
 cs=[]
 if detection is not None:cs=[{'contact_id':100+step,'beam_element_id':6,'fixture_element_id':0,'detection_value_m':float(detection),'beam_outward_normal':[0.,float(normal_y),0.]}]
 ps=[[.02*i,.003,0.] for i in range(7)];ps[6][1]=float(y);return {'step':step,'step_end_time_s':time,'native_contacts':cs,'constraint_vector_size':lcp,'reaction_force_proxy':reaction,'beam_node_positions_m':ps}
def first():return {'node_id':6,'from_record_index':1,'to_record_index':2,'from_time_s':.142,'to_time_s':.144,'transition_duration_s':.002,'from_signed_plane_distance_m':.00001,'to_signed_plane_distance_m':-.0002}
def run(detection,lcp,reaction,*,normal=-1.):
 rs=[record(0,.140,.0002,detection=-.0001,lcp=1,reaction=1.),record(1,.142,.00001,detection=-.001,lcp=1,reaction=1.),record(2,.144,-.0002,detection=detection,normal_y=normal,lcp=lcp,reaction=reaction)];return audit_crossings(rs,[first()],wall_y=0,reaction_floor=0,time_tolerance_s=TIME_TOL,distance_tolerance_m=DIST_TOL)
def test_local_contact_and_global_response_active():
 r=run(-.001,1,1.);s=r['primary_event']['completion_state'];assert r['verdict']==V_LOCAL_CONTACT_GLOBAL_ACTIVE and s['local_geometric_contact'] and s['global_lcp_rows_present'] and s['global_reaction_active'] and s['local_geometric_normal_y_median']==-1 and r['primary_event']['normal_y_sign_flip_across_crossing'] is True
def test_local_contact_persists_global_response_inactive():assert run(-.001,0,0)['verdict']==V_LOCAL_CONTACT_GLOBAL_INACTIVE
def test_local_contact_absent_at_completion():
 r=run(None,0,0);p=r['primary_event'];assert r['verdict']==V_LOCAL_CONTACT_ABSENT and p['from_state']['local_geometric_contact'] and not p['completion_state']['local_geometric_contact'] and p['last_local_geometric_contact']['time_s']==.142 and abs(p['crossing_minus_last_geometric_s']-.002)<1e-12
def test_first_crossing_is_primary():
 rs=[record(0,.140,.0002,detection=-.0001,lcp=1,reaction=1),record(1,.142,.00001,detection=-.001,lcp=1,reaction=1),record(2,.144,-.0002,detection=-.001,normal_y=-1,lcp=1,reaction=1),record(3,.198,.00002,detection=-.001,lcp=1,reaction=1),record(4,.2,-.0001,detection=-.001,normal_y=-1,lcp=1,reaction=1)];later={**first(),'from_record_index':3,'to_record_index':4,'from_time_s':.198,'to_time_s':.2,'from_signed_plane_distance_m':.00002,'to_signed_plane_distance_m':-.0001};r=audit_crossings(rs,[later,first()],wall_y=0,reaction_floor=0,time_tolerance_s=TIME_TOL,distance_tolerance_m=DIST_TOL);assert r['primary_event']['crossing']['to_time_s']==.144 and r['verdict']==V_LOCAL_CONTACT_GLOBAL_ACTIVE
def test_crossing_trace_alignment_is_required():
 rs=[record(0,.140,.0002,detection=-.0001,lcp=1,reaction=1),record(1,.142,.00001,detection=-.001,lcp=1,reaction=1),record(2,.144,-.0002,detection=-.001,lcp=1,reaction=1)];b=first();b['to_time_s']=.146
 try:audit_crossings(rs,[b],wall_y=0,reaction_floor=0,time_tolerance_s=TIME_TOL,distance_tolerance_m=DIST_TOL)
 except RuntimeError as e:assert 'to_time' in str(e)
 else:raise AssertionError('misaligned crossing must fail')
