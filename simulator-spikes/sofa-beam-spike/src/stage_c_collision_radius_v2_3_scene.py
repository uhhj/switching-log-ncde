"""The one permitted V2.3 Sphere/Triangle C1 scene."""
from __future__ import annotations
import json, os
from pathlib import Path
import numpy as np
import Sofa
def _cfg(): return json.loads(Path(os.environ["SOFA_STAGE_C_V23_CONFIG"]).read_text())
def _fixture(root,s):
 y,x0,x1,z=map(float,(s["plane_y_m"],s["x_min_m"],s["x_max_m"],s["z_half_extent_m"])); f=root.addChild("ContactWall")
 f.addObject("MeshTopology",name="topology",position=[[x0,y,-z],[x1,y,-z],[x1,y,z],[x0,y,z]],triangles=[[0,1,2],[0,2,3]])
 f.addObject("MechanicalObject",name="dofs",position="@topology.position"); return f.addObject("TriangleCollisionModel",name="triangles",moving=False,simulated=False,bothSide=True,contactDistance=0.)
class Trace(Sofa.Core.Controller):
 def __init__(self,*a,**k):
  super().__init__(*a,**k);self.root=k["root"];self.dofs=k["dofs"];self.solver=k["solver"];self.force=k["force"];self.bridge=k["bridge"];self.sphere=k["sphere"];self.fixture=k["fixture"];self.inter=k["inter"];self.ff=k["ff"];self.c=k["config"];self.limit=k["steps"];self.records=[];self.pending=None;self.written=False;self.initial=np.asarray(self.dofs.position.value,float)[:,:3].copy();self.tail=self.initial[0].copy();self.snap=None
 def _snapshot(self):
  return {"beam_collision_model":"SphereCollisionModel","beam_default_radius_m":float(self.sphere.radius.value),"beam_list_radius_m":np.asarray(self.sphere.listRadius.value,float).reshape(-1).tolist(),"beam_model_contact_distance_m":float(self.sphere.contactDistance.value),"fixture_collision_model":"TriangleCollisionModel","fixture_model_contact_distance_m":float(self.fixture.contactDistance.value),"intersection_contact_distance_m":float(self.root.getObject("LocalMinDistance").contactDistance.value),"intersection_alarm_distance_m":float(self.root.getObject("LocalMinDistance").alarmDistance.value)}
 def _contacts(self,serial):
  n=int(self.bridge.contactCount.value); arrays=[np.asarray(getattr(self.bridge,x).value).reshape(-1) for x in ("beamElementIds","fixtureElementIds","contactIds","detectionValues")]
  b,f,ids,vals=arrays; bp=np.asarray(self.bridge.beamContactPoints.value,float).reshape(n,3);fp=np.asarray(self.bridge.fixtureContactPoints.value,float).reshape(n,3); norm=np.asarray(self.bridge.beamOutwardNormals.value,float).reshape(n,3)
  if any(x.size!=n for x in arrays): raise RuntimeError("native bridge contact-array mismatch")
  return [{"source_frame_serial":serial,"contact_id":int(ids[i]),"beam_element_id":int(b[i]),"fixture_element_id":int(f[i]),"beam_contact_point_m":bp[i].tolist(),"fixture_contact_point_m":fp[i].tolist(),"beam_outward_normal":norm[i].tolist(),"detection_value_m":float(vals[i])} for i in range(n)]
 def onAnimateBeginEvent(self,e):
  if self.snap is None:self.snap=self._snapshot()
  t=float(self.root.time.value); zero=t<float(self.c["simulation"]["zero_load_end_ms"])/1000.; force=[0.,0.,0.] if zero else [0.,-float(self.c["simulation"]["normal_load_n"]),0.]
  self.force.forces.value=np.asarray([[*force,0.,0.,0.]],float);self.pending={"step":len(self.records),"step_start_time_s":t,"scheduled_force_n":force,"actual_applied_force_n":force,"zero_load_phase":zero,"measurement_active":t>=float(self.c["simulation"]["measurement_start_ms"])/1000.,"command_active":not zero}
 def onAnimateEndEvent(self,e):
  if self.pending is None:raise RuntimeError("unpaired animation event")
  p=np.asarray(self.dofs.position.value,float);q=np.asarray(self.solver.constraintForces.value,float).reshape(-1);serial=int(self.bridge.frameSerial.value);cs=self._contacts(serial)
  self.records.append({**self.pending,"step_end_time_s":float(self.root.time.value),"bridge_ready":bool(self.bridge.ready.value),"bridge_frame_serial":serial,"native_contact_source_serial":serial,"native_contact_count":len(cs),"native_contacts":cs,"constraint_vector_size":int(q.size),"constraint_vector":q.tolist(),"reaction_force_proxy":float(np.linalg.norm(q)) if q.size else 0.,"tail_translation_displacement_m":float(np.linalg.norm(p[0,:3]-self.tail)),"beam_node_positions_m":p[:,:3].tolist(),"max_node_displacement_m":float(np.max(np.linalg.norm(p[:,:3]-self.initial,axis=1)))});self.pending=None
  if len(self.records)>=self.limit:self._write()
 def _write(self):
  if not self.written:Path(os.environ["SOFA_STAGE_C_V23_TRACE"]).write_text(json.dumps({"metadata":{"mode":"collision_radius_c1","collision_snapshot":self.snap,"material_snapshot":{"default_young_modulus_pa":np.asarray(self.inter.defaultYoungModulus.value,float).reshape(-1).tolist(),"default_poisson_ratio":np.asarray(self.inter.defaultPoissonRatio.value,float).reshape(-1).tolist(),"mass_density_kg_m3":np.asarray(self.ff.massDensity.value,float).reshape(-1).tolist()}},"records":self.records},indent=2)+"\n");self.written=True
 def onSimulationEndedEvent(self,e):self._write()
def createScene(root):
 c=_cfg();b,s=c["beam"],c["simulation"];n=int(b["nodes"]);L,r=float(b["length_m"]),float(b["radius_m"]);root.gravity=[0.,0.,0.];root.dt=float(s["dt_s"])
 root.addObject("RequiredPlugin",pluginName="BeamAdapter Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.Mapping.Linear Sofa.Component.MechanicalLoad Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant")
 root.addObject("FreeMotionAnimationLoop");solver=root.addObject("LCPConstraintSolver",name="constraint_solver",mu=float(s["friction"]),tolerance=1e-8,maxIt=1000,build_lcp=False,computeConstraintForces=True);root.addObject("CollisionPipeline",depth=6);root.addObject("BruteForceBroadPhase");root.addObject("BVHNarrowPhase",name="narrow_phase");root.addObject("LocalMinDistance",name="LocalMinDistance",alarmDistance=float(s["alarm_distance_m"]),contactDistance=float(s["contact_distance_m"]));root.addObject("CollisionResponse",response="FrictionContactConstraint",responseParams=f"mu={float(s['friction'])}")
 beam=root.addChild("Beam");beam.addObject("EulerImplicitSolver",rayleighStiffness=.05,rayleighMass=.05);beam.addObject("BTDLinearSolver",verbose=False);dofs=beam.addObject("MechanicalObject",template="Rigid3d",name="dofs",position=[[i*L/(n-1),float(c["contact_wall"]["beam_initial_y_m"]),0.,0.,0.,0.,1.] for i in range(n)]);beam.addObject("MeshTopology",name="lines",lines=[[i,i+1] for i in range(n-1)])
 inter=beam.addObject("BeamInterpolation",name="interpolation",radius=[r],defaultYoungModulus=[float(b["young_modulus_pa"])],defaultPoissonRatio=[float(b["poisson_ratio"])]);ff=beam.addObject("AdaptiveBeamForceFieldAndMass",name="beam_force_field",computeMass=True,massDensity=[float(b["mass_density_kg_m3"])],interpolation="@interpolation");beam.addObject("UncoupledConstraintCorrection",defaultCompliance=1e-6);tail=c["tail_constraint"];beam.addObject("PartialFixedProjectiveConstraint",name="tail_translation_anchor",indices=[int(tail["node_index"])],fixedDirections=[int(v) for v in tail["fixed_directions"]]);force=beam.addObject("ConstantForceField",name="endpoint_force",indices=[n-1],forces=[0.]*6)
 col=beam.addChild("Collision");col.addObject("MechanicalObject",name="collision_dofs");col.addObject("IdentityMapping");sphere=col.addObject("SphereCollisionModel",name="beam_spheres",radius=r,contactDistance=0.);fixture=_fixture(root,c["contact_wall"]);bridge=root.addObject("NativeContactBridge",name="native_contact_bridge",beamModel="@Beam/Collision/beam_spheres",fixtureModel="@ContactWall/triangles",narrowPhase="@narrow_phase")
 root.addObject(Trace(name="stage_c_v23_trace",root=root,dofs=dofs,solver=solver,force=force,bridge=bridge,sphere=sphere,fixture=fixture,inter=inter,ff=ff,config=c,steps=int(s["contact_steps"])));return root
