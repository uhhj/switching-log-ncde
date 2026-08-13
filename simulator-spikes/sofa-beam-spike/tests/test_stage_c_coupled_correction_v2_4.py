import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];SPIKE=ROOT/'simulator-spikes'/'sofa-beam-spike';sys.path.insert(0,str(SPIKE/'src'))
from stage_c_coupled_correction_v2_4 import component_state_is_valid,intervention_gate,response_stack_gate
def configs():return json.loads((SPIKE/'configs'/'stage_c_collision_radius_v2_3.json').read_text()),json.loads((SPIKE/'configs'/'stage_c_coupled_correction_v2_4.json').read_text())
def snapshot(state='Valid'):
 return {'correction_class':'LinearSolverConstraintCorrection','correction_component_state':state,'correction_mode':'linear_solver_constraint_correction','correction_wire_optimization':False,'correction_regularization_term':0.,'linear_solver_class':'BTDLinearSolver','ode_solver_class':'EulerImplicitSolver','ode_rayleigh_stiffness':.05,'ode_rayleigh_mass':.05,'lcp_solver_class':'LCPConstraintSolver','lcp_mu':.8,'lcp_tolerance':1e-8,'lcp_max_it':1000,'lcp_build_lcp':False,'lcp_compute_constraint_forces':True,'collision_response_class':'CollisionResponse','collision_response':'FrictionContactConstraint','collision_response_params':'mu=0.8','fixture_both_side':True,'fixture_triangle_indices':[[0,1,2],[0,2,3]],'intersection_contact_distance_m':0.,'intersection_alarm_distance_m':.008,'beam_default_radius_m':.002,'beam_model_contact_distance_m':0.,'fixture_model_contact_distance_m':0.}
def test_only_correction_is_added():
 a,b=configs();r=intervention_gate(a,b);assert r['pass'] and r['diff_paths']==['constraint_correction']
def test_component_state_token_accepts_valid_not_invalid():assert component_state_is_valid('ComponentState.Valid') and not component_state_is_valid('Invalid')
def test_frozen_response_stack_passes():_,c=configs();assert response_stack_gate(snapshot(),c)['pass']
def test_bad_correction_state_fails():_,c=configs();assert not response_stack_gate(snapshot('Invalid'),c)['pass']
def test_wire_optimization_true_fails():_,c=configs();s=snapshot();s['correction_wire_optimization']=True;assert not response_stack_gate(s,c)['pass']
