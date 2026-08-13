"""Offline crossing-local audit for frozen Stage-C V2.3-R2."""
from __future__ import annotations
import math
from statistics import median
V_LOCAL_CONTACT_GLOBAL_ACTIVE='PHASE0S_SOFA_STAGE_C_V2_3L_LOCAL_CONTACT_PERSISTS_GLOBAL_RESPONSE_ACTIVE_AT_FIRST_CROSSING'
V_LOCAL_CONTACT_GLOBAL_INACTIVE='PHASE0S_SOFA_STAGE_C_V2_3L_LOCAL_CONTACT_PERSISTS_GLOBAL_RESPONSE_INACTIVE_AT_FIRST_CROSSING'
V_LOCAL_CONTACT_ABSENT='PHASE0S_SOFA_STAGE_C_V2_3L_LOCAL_CONTACT_ABSENT_AT_FIRST_CROSSING_COMPLETION'
def _close(actual,expected,tol,label):
 a,e,t=map(float,(actual,expected,tol))
 if not all(map(math.isfinite,(a,e,t))) or t<0 or abs(a-e)>t:raise RuntimeError(f'{label} mismatch: actual={actual!r}, expected={expected!r}, tolerance={tol!r}')
def validate_crossing_alignment(records,crossing,*,wall_y,time_tolerance_s,distance_tolerance_m):
 n=int(crossing['node_id']);i0=int(crossing['from_record_index']);i1=int(crossing['to_record_index'])
 if i0<0 or i1<0 or i0>=len(records) or i1>=len(records):raise RuntimeError('crossing record index out of bounds')
 if i1!=i0+1:raise RuntimeError('crossing records must be consecutive')
 r0,r1=records[i0],records[i1];_close(r0['step_end_time_s'],crossing['from_time_s'],time_tolerance_s,'crossing from_time');_close(r1['step_end_time_s'],crossing['to_time_s'],time_tolerance_s,'crossing to_time');y0=float(r0['beam_node_positions_m'][n][1])-float(wall_y);y1=float(r1['beam_node_positions_m'][n][1])-float(wall_y);_close(y0,crossing['from_signed_plane_distance_m'],distance_tolerance_m,'crossing from signed distance');_close(y1,crossing['to_signed_plane_distance_m'],distance_tolerance_m,'crossing to signed distance')
 if not(y0>0 and y1<0):raise RuntimeError('source crossing is not positive-to-negative')
 return {'from_record_index':i0,'to_record_index':i1,'from_signed_plane_distance_m':y0,'to_signed_plane_distance_m':y1,'pass':True}
def contacts_for_node(record,node_id):return [c for c in record['native_contacts'] if int(c['beam_element_id'])==int(node_id)]
def local_state(record,node_id,*,wall_y,reaction_floor):
 cs=contacts_for_node(record,node_id);geo=[c for c in cs if float(c['detection_value_m'])<=0];vals=[float(c['detection_value_m']) for c in cs];norm=[float(c['beam_outward_normal'][1]) for c in geo];pos=record['beam_node_positions_m'][int(node_id)]
 return {'node_id':int(node_id),'step':int(record['step']),'time_s':float(record['step_end_time_s']),'signed_center_y_m':float(pos[1])-float(wall_y),'local_native_detection':bool(cs),'local_native_count':len(cs),'local_geometric_contact':bool(geo),'local_geometric_count':len(geo),'local_min_detection_value_m':min(vals) if vals else None,'local_detection_values_m':vals,'local_geometric_normal_y_values':norm,'local_geometric_normal_y_median':None if not norm else float(median(norm)),'contact_ids':[int(c['contact_id']) for c in cs],'fixture_triangle_ids':sorted({int(c['fixture_element_id']) for c in cs}),'global_constraint_vector_size':int(record['constraint_vector_size']),'global_lcp_rows_present':int(record['constraint_vector_size'])>0,'global_reaction_force_proxy':float(record['reaction_force_proxy']),'global_reaction_active':float(record['reaction_force_proxy'])>float(reaction_floor)}
def last_local_geometric(records,node_id,*,end_index,wall_y,reaction_floor):
 for i in range(int(end_index),-1,-1):
  s=local_state(records[i],node_id,wall_y=wall_y,reaction_floor=reaction_floor)
  if s['local_geometric_contact']:return {'record_index':i,**s}
 return None
def normal_sign_flip(a,b):return None if a is None or b is None else float(a)*float(b)<0
def audit_event(records,crossing,*,wall_y,reaction_floor,time_tolerance_s,distance_tolerance_m,window_half_width_s=.020):
 alignment=validate_crossing_alignment(records,crossing,wall_y=wall_y,time_tolerance_s=time_tolerance_s,distance_tolerance_m=distance_tolerance_m);n=int(crossing['node_id']);i0=int(crossing['from_record_index']);i1=int(crossing['to_record_index']);time=float(crossing['to_time_s']);before=local_state(records[i0],n,wall_y=wall_y,reaction_floor=reaction_floor);complete=local_state(records[i1],n,wall_y=wall_y,reaction_floor=reaction_floor);last=last_local_geometric(records,n,end_index=i1,wall_y=wall_y,reaction_floor=reaction_floor);window=[{'record_index':i,**local_state(r,n,wall_y=wall_y,reaction_floor=reaction_floor)} for i,r in enumerate(records) if time-window_half_width_s<=float(r['step_end_time_s'])<=time+window_half_width_s]
 return {'node_id':n,'alignment':alignment,'crossing':dict(crossing),'from_state':before,'completion_state':complete,'normal_y_sign_flip_across_crossing':normal_sign_flip(before['local_geometric_normal_y_median'],complete['local_geometric_normal_y_median']),'last_local_geometric_contact':last,'crossing_minus_last_geometric_s':None if last is None else time-float(last['time_s']),'window_half_width_s':float(window_half_width_s),'window':window}
def primary_verdict(event):
 s=event['completion_state']
 if not s['local_geometric_contact']:return V_LOCAL_CONTACT_ABSENT,'Node-local formal geometric contact is absent at first crossing completion.'
 if s['global_lcp_rows_present'] and s['global_reaction_active']:return V_LOCAL_CONTACT_GLOBAL_ACTIVE,'Node-local formal geometric contact persists while global LCP rows and global reaction response are active at first crossing completion.'
 return V_LOCAL_CONTACT_GLOBAL_INACTIVE,'Node-local formal geometric contact persists, but global LCP/reaction response is not fully active at first crossing completion.'
def audit_crossings(records,crossings,*,wall_y,reaction_floor,time_tolerance_s,distance_tolerance_m):
 if not crossings:raise ValueError('no V2.3-R2 crossings')
 events=[audit_event(records,x,wall_y=wall_y,reaction_floor=reaction_floor,time_tolerance_s=time_tolerance_s,distance_tolerance_m=distance_tolerance_m) for x in sorted(crossings,key=lambda x:float(x['to_time_s']))];v,reason=primary_verdict(events[0]);summary=[]
 for e in events:
  b,c,last=e['from_state'],e['completion_state'],e['last_local_geometric_contact'];summary.append({'node_id':e['node_id'],'crossing_time_s':float(e['crossing']['to_time_s']),'from_signed_center_y_m':b['signed_center_y_m'],'completion_signed_center_y_m':c['signed_center_y_m'],'local_native_at_crossing':c['local_native_detection'],'local_geometric_at_crossing':c['local_geometric_contact'],'local_min_detection_value_m':c['local_min_detection_value_m'],'from_geometric_normal_y_median':b['local_geometric_normal_y_median'],'completion_geometric_normal_y_median':c['local_geometric_normal_y_median'],'normal_y_sign_flip_across_crossing':e['normal_y_sign_flip_across_crossing'],'global_lcp_at_crossing':c['global_lcp_rows_present'],'global_reaction_active_at_crossing':c['global_reaction_active'],'reaction_force_proxy_at_crossing':c['global_reaction_force_proxy'],'last_local_geometric_time_s':None if last is None else last['time_s'],'crossing_minus_last_geometric_s':e['crossing_minus_last_geometric_s'],'contact_ids_at_crossing':c['contact_ids'],'fixture_triangle_ids_at_crossing':c['fixture_triangle_ids']})
 return {'verdict':v,'reason':reason,'primary_event':events[0],'crossing_events':events,'per_node_summary':summary,'reaction_floor':float(reaction_floor),'wall_plane_y_m':float(wall_y),'time_tolerance_s':float(time_tolerance_s),'distance_tolerance_m':float(distance_tolerance_m),'primary_verdict_uses_first_crossing_only':True,'constraint_response_is_global_not_node_mapped':True,'normal_y_is_diagnostic_only':True,'normal_y_selects_next_intervention':False,'temporal_state_evidence_is_causal_proof':False}
