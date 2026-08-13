import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from stage_c_loss_temporal_order import VERDICT_AMBIGUOUS,VERDICT_CROSSING_FIRST,VERDICT_EXIT_FIRST,audit_temporal_order,crossing_consistency,first_permanent_true_suffix_index

def config(): return {"beam":{"nodes":3},"tail_constraint":{"node_index":0,"fixed_directions":[1,1,1,0,0,0]},"contact_wall":{"plane_y_m":0.,"x_min_m":0.,"x_max_m":.18,"z_half_extent_m":.05},"gates":{"maximum_fixture_plane_error_m":1e-6},"integrity":{"time_tolerance_s":1e-10}}
def record(t,n1,n2): return {"step_end_time_s":t,"beam_node_positions_m":[[0.,.003,0.],n1,n2],"native_contacts":[]}
def cls(value=False): return {"geometric_contact":value}
def event(node,frm,to,ft,tt): return {"node_id":node,"from_record_index":frm,"to_record_index":to,"from_time_s":ft,"to_time_s":tt}
def test_permanent_suffix_ignores_temporary_true_run(): assert first_permanent_true_suffix_index([False,True,True,False,True,True],start_index=0)==4
def test_crossing_consistency(): assert crossing_consistency([event(2,4,5,.01,.012)],[event(2,4,5,.01,.012)],time_tolerance_s=1e-10)["pass"]
def test_crossing_first():
 r=[record(.002,[.1,.001,0],[.12,.003,0]),record(.004,[.1,-.001,0],[.12,.003,0]),record(.006,[.3,-.002,0],[.12,.003,0]),record(.008,[.3,-.003,0],[.31,.003,0])];a=audit_temporal_order(r,[cls(1),cls(1),cls(),cls()],config=config(),frozen_crossing_events=[event(1,0,1,.002,.004)]);assert a["verdict"]==VERDICT_CROSSING_FIRST and a["first_permanent_node_exit_time_s"]==.006
def test_exit_first():
 r=[record(.002,[.1,.003,0],[.12,.001,0]),record(.004,[.3,.003,0],[.12,.001,0]),record(.006,[.3,.003,0],[.12,-.001,0]),record(.008,[.3,.003,0],[.31,-.002,0])];a=audit_temporal_order(r,[cls(1),cls(1),cls(1),cls()],config=config(),frozen_crossing_events=[event(2,1,2,.004,.006)]);assert a["verdict"]==VERDICT_EXIT_FIRST
def test_simultaneous_is_ambiguous():
 r=[record(.002,[.1,.003,0],[.12,.001,0]),record(.004,[.3,.003,0],[.12,-.001,0]),record(.006,[.3,.003,0],[.31,-.002,0])];a=audit_temporal_order(r,[cls(1),cls(1),cls()],config=config(),frozen_crossing_events=[event(2,0,1,.002,.004)]);assert a["verdict"]==VERDICT_AMBIGUOUS
def test_disagreement_forces_ambiguous():
 r=[record(.002,[.1,.001,0],[.12,.003,0]),record(.004,[.1,-.001,0],[.12,.003,0]),record(.006,[.3,-.002,0],[.31,.003,0])];a=audit_temporal_order(r,[cls(1),cls(1),cls()],config=config(),frozen_crossing_events=[]);assert a["verdict"]==VERDICT_AMBIGUOUS and not a["crossing_consistency"]["pass"]
