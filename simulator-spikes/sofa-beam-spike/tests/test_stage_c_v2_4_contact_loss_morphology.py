import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulator-spikes'/'sofa-beam-spike'/'src'))
from stage_c_v2_4_contact_loss_morphology import *
def config():return {'beam':{'nodes':3},'tail_constraint':{'node_index':0,'fixed_directions':[1,1,1,0,0,0]},'contact_wall':{'plane_y_m':0.,'x_min_m':0.,'x_max_m':1.,'z_half_extent_m':1.},'gates':{'maximum_fixture_plane_error_m':1e-6}}
def rec(t,y=.001,contacts=True,foot=True):
 x=.5 if foot else 2.;cs=[{'beam_element_id':1,'detection_value_m':.001,'contact_id':1,'fixture_element_id':0}] if contacts else [];return {'step_end_time_s':t,'beam_node_positions_m':[[0,0,0],[x,y,0],[x,y,0]],'native_contacts':cs,'reaction_force_proxy':1.,'measurement_active':True}
def test_proximity_gap_class():
 c=config();frames=[frame(rec(.1),{'lcp_rows_present':True,'reaction_active':True},c,.01)];assert gap_class(frames,c,.01)==GAP_PROXIMITY_ONLY
def test_native_dropout_class():
 c=config();frames=[frame(rec(.1,contacts=False),{'lcp_rows_present':True,'reaction_active':True},c,.01)];assert gap_class(frames,c,.01)==GAP_NATIVE_DROPOUT
def test_alarm_exit_class():
 c=config();frames=[frame(rec(.1,y=.1,contacts=False),{'lcp_rows_present':True,'reaction_active':True},c,.01)];assert gap_class(frames,c,.01)==GAP_ALARM_EXIT
def test_footprint_has_precedence():
 c=config();frames=[frame(rec(.1,y=.1,contacts=False,foot=False),{'lcp_rows_present':True,'reaction_active':True},c,.01)];assert gap_class(frames,c,.01)==GAP_FOOTPRINT_LOSS
def test_mixed_trace_verdict():assert trace_verdict([{'gap_class':GAP_PROXIMITY_ONLY},{'gap_class':GAP_NATIVE_DROPOUT}])==V_MIXED
