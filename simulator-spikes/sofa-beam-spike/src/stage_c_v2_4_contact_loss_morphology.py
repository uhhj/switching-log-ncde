"""Offline stable-contact loss morphology for frozen Stage-C V2.4."""
from __future__ import annotations
from collections import Counter
import math
import numpy as np
from contact_native import contact_episode_stats_timestamps
GAP_PROXIMITY_ONLY='PROXIMITY_ONLY';GAP_NATIVE_DROPOUT='NATIVE_DROPOUT_WITHIN_ALARM_BAND';GAP_ALARM_EXIT='ALARM_BAND_EXIT';GAP_FOOTPRINT_LOSS='TEMPORARY_FIXTURE_FOOTPRINT_LOSS';V_PROXIMITY='PHASE0S_SOFA_STAGE_C_V2_4L_ALL_INTERNAL_GAPS_PROXIMITY_ONLY';V_DROPOUT='PHASE0S_SOFA_STAGE_C_V2_4L_NATIVE_DROPOUT_WITHIN_ALARM_BAND';V_ALARM_EXIT='PHASE0S_SOFA_STAGE_C_V2_4L_ALARM_BAND_EXIT_CONTACT_LOSS';V_FOOTPRINT='PHASE0S_SOFA_STAGE_C_V2_4L_TEMPORARY_FIXTURE_FOOTPRINT_LOSS';V_MIXED='PHASE0S_SOFA_STAGE_C_V2_4L_MIXED_CONTACT_GAP_MORPHOLOGY'
def fixed(c):return {int(c['tail_constraint']['node_index'])} if list(map(int,c['tail_constraint']['fixed_directions'][:3]))==[1,1,1] else set()
def mobile(c):return sorted(set(range(int(c['beam']['nodes'])))-fixed(c))
def finite_surface_distance(p,w):
 x,y,z=map(float,p);dx=max(float(w['x_min_m'])-x,0.,x-float(w['x_max_m']));dz=max(-float(w['z_half_extent_m'])-z,0.,z-float(w['z_half_extent_m']));dy=y-float(w['plane_y_m']);return math.sqrt(dx*dx+dy*dy+dz*dz)
def footprint_ids(r,c):
 w=c['contact_wall'];tol=float(c['gates']['maximum_fixture_plane_error_m']);out=[]
 for n in mobile(c):
  p=r['beam_node_positions_m'][n]
  if float(w['x_min_m'])-tol<=float(p[0])<=float(w['x_max_m'])+tol and -float(w['z_half_extent_m'])-tol<=float(p[2])<=float(w['z_half_extent_m'])+tol:out.append(n)
 return out
def native_ids(r,c):return sorted({int(x['beam_element_id']) for x in r['native_contacts'] if int(x['beam_element_id']) in set(mobile(c))})
def frame(r,cls,c,alarm):
 ids=mobile(c);ds=[finite_surface_distance(r['beam_node_positions_m'][n],c['contact_wall']) for n in ids];vals=[float(x['detection_value_m']) for x in r['native_contacts'] if int(x['beam_element_id']) in set(ids)];fp=footprint_ids(r,c);nat=native_ids(r,c);return {'time_s':float(r['step_end_time_s']),'footprint_node_ids':fp,'mobile_footprint_count':len(fp),'mobile_native_node_ids':nat,'mobile_native_detection_count':len(nat),'minimum_mobile_center_to_fixture_distance_m':min(ds),'minimum_mobile_center_plane_distance_m':min(abs(float(r['beam_node_positions_m'][n][1])-float(c['contact_wall']['plane_y_m'])) for n in ids),'global_lcp_present':bool(cls['lcp_rows_present']),'global_reaction_active':bool(cls['reaction_active']),'reaction_force_proxy':float(r['reaction_force_proxy']),'positive_detection_values_m':[v for v in vals if v>0],'minimum_positive_detection_value_m':min((v for v in vals if v>0),default=None),'maximum_positive_detection_value_m':max((v for v in vals if v>0),default=None),'within_alarm_band':min(ds)<=float(alarm)+float(c['gates']['maximum_fixture_plane_error_m'])}
def gap_class(frames,c,alarm):
 if any(x['mobile_footprint_count']==0 for x in frames):return GAP_FOOTPRINT_LOSS
 native_any=any(x['mobile_native_detection_count']>0 for x in frames)
 if any(not x['within_alarm_band'] for x in frames):
  if native_any:raise ValueError('finite-fixture alarm exit conflicts with native detection')
  return GAP_ALARM_EXIT
 if any(x['mobile_native_detection_count']==0 for x in frames):return GAP_NATIVE_DROPOUT
 return GAP_PROXIMITY_ONLY
def trace_verdict(gaps):
 kinds={x['gap_class'] for x in gaps}
 if len(kinds)>1:return V_MIXED
 return {GAP_PROXIMITY_ONLY:V_PROXIMITY,GAP_NATIVE_DROPOUT:V_DROPOUT,GAP_ALARM_EXIT:V_ALARM_EXIT,GAP_FOOTPRINT_LOSS:V_FOOTPRINT}[next(iter(kinds))]
def interval(records,classes,indices,c,alarm,pre,post):
 fs=[frame(records[i],classes[i],c,alarm) for i in indices];return {'start_record_index':indices[0] if indices else None,'end_record_index':indices[-1] if indices else None,'samples':len(indices),'start_time_s':None if not indices else fs[0]['time_s'],'end_time_s':None if not indices else fs[-1]['time_s'],'pre_contact_node_ids':pre,'post_contact_node_ids':post,'overlap_node_ids':sorted(set(pre)&set(post)),'same_node_recontact':bool(set(pre)&set(post)),'node_set_changed':set(pre)!=set(post),'loss_to_recontact_s':None if not indices else fs[-1]['time_s']-fs[0]['time_s'],'mobile_native_detection_fraction':sum(x['mobile_native_detection_count']>0 for x in fs)/len(fs) if fs else 0.,'global_lcp_present_fraction':sum(x['global_lcp_present'] for x in fs)/len(fs) if fs else 0.,'global_reaction_active_fraction':sum(x['global_reaction_active'] for x in fs)/len(fs) if fs else 0.,'reaction_proxy_min':min((x['reaction_force_proxy'] for x in fs),default=None),'reaction_proxy_max':max((x['reaction_force_proxy'] for x in fs),default=None),'minimum_mobile_center_to_fixture_distance_m':min((x['minimum_mobile_center_to_fixture_distance_m'] for x in fs),default=None),'maximum_framewise_minimum_mobile_center_to_fixture_distance_m':max((x['minimum_mobile_center_to_fixture_distance_m'] for x in fs),default=None),'minimum_mobile_center_plane_distance_m':min((x['minimum_mobile_center_plane_distance_m'] for x in fs),default=None),'maximum_positive_detection_value_m':max((x['maximum_positive_detection_value_m'] for x in fs if x['maximum_positive_detection_value_m'] is not None),default=None),'minimum_positive_detection_value_m':min((x['minimum_positive_detection_value_m'] for x in fs if x['minimum_positive_detection_value_m'] is not None),default=None),'frames':fs}
def audit_stable_contact_loss(records,classes,*,config,effective_alarm_distance_m):
 pairs=[i for i,(r,cl) in enumerate(zip(records,classes)) if r['measurement_active']];times=[records[i]['step_end_time_s'] for i in pairs];flags=[classes[i]['geometric_contact'] for i in pairs];stats=contact_episode_stats_timestamps(times,flags);episodes=stats['episode_records'];gaps=[]
 for a,b in zip(episodes,episodes[1:]):
  idx=list(range(pairs[a['end_index']]+1,pairs[b['start_index']]));pre=native_ids(records[pairs[a['end_index']]],config);post=native_ids(records[pairs[b['start_index']]],config);g=interval(records,classes,idx,config,effective_alarm_distance_m,pre,post);g['gap_class']=gap_class(g['frames'],config,effective_alarm_distance_m);gaps.append(g)
 first=pairs[episodes[0]['start_index']];last=pairs[episodes[-1]['end_index']];initial=interval(records,classes,list(range(pairs[0],first)),config,effective_alarm_distance_m,[],native_ids(records[first],config));terminal=interval(records,classes,list(range(last+1,pairs[-1]+1)),config,effective_alarm_distance_m,native_ids(records[last],config),[]);counts=dict(Counter(g['gap_class'] for g in gaps));migration={'same_node_recontact_gaps':sum(g['same_node_recontact'] for g in gaps),'disjoint_recontact_gaps':sum(not g['same_node_recontact'] for g in gaps),'node_set_changed_gaps':sum(g['node_set_changed'] for g in gaps),'formal_mechanism_selector':False}
 return {'verdict':trace_verdict(gaps),'recomputed_contact_occupancy':stats['occupancy'],'recomputed_episode_count':stats['episodes'],'recomputed_longest_timestamp_span_s':stats['longest_duration_s'],'internal_gap_count':len(gaps),'gap_class_counts':counts,'internal_gaps':gaps,'initial_precontact_interval':initial,'terminal_postcontact_interval':terminal,'migration_diagnostic':migration}
