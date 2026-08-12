"""Provenance-correct Stage C V2 material baseline and stable-contact scene."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import Sofa


def _config(): return json.loads(Path(os.environ["SOFA_STAGE_C_V2_CONFIG"]).read_text())
def _rows(value):
    a = np.asarray(value, dtype=float)
    if not a.size: return np.zeros((0, 6), dtype=float)
    if a.size % 6: raise RuntimeError("ConstantForceField forces cannot be reshaped into Rigid3 Deriv rows")
    return a.reshape(-1, 6)


def _fixture(root, name, spec):
    y, x0, x1, z = float(spec["plane_y_m"]), float(spec["x_min_m"]), float(spec["x_max_m"]), float(spec["z_half_extent_m"])
    node = root.addChild(name)
    node.addObject("MeshTopology", name="topology", position=[[x0,y,-z],[x1,y,-z],[x1,y,z],[x0,y,z]], triangles=[[0,1,2],[0,2,3]])
    node.addObject("MechanicalObject", name="dofs", position="@topology.position")
    node.addObject("TriangleCollisionModel", name="triangles", moving=False, simulated=False, bothSide=True)


class Controller(Sofa.Core.Controller):
    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.root, self.dofs, self.collision_dofs = kw["root"], kw["dofs"], kw["collision_dofs"]
        self.solver, self.force_field, self.bridge = kw["solver"], kw["force_field"], kw["bridge"]
        self.interpolation, self.beam_force_field = kw["interpolation"], kw["beam_force_field"]
        self.config, self.mode, self.steps = kw["config"], kw["mode"], int(kw["steps"])
        sim = self.config["simulation"]; self.zero_end = float(sim["zero_load_end_ms"])/1000.; self.measure_start = float(sim["measurement_start_ms"])/1000.; self.load = float(sim["normal_load_n"])
        self.next_step_index, self.pending_step, self.material_snapshot, self.records, self.written = 0, None, None, [], False
        p = np.asarray(self.dofs.position.value, dtype=float); self.initial_positions = p[:, :3].copy(); self.initial_tail = p[0,:3].copy()

    def _material(self):
        return {"default_young_modulus_pa": np.asarray(self.interpolation.defaultYoungModulus.value,dtype=float).reshape(-1).tolist(), "default_poisson_ratio": np.asarray(self.interpolation.defaultPoissonRatio.value,dtype=float).reshape(-1).tolist(), "radius_m": np.asarray(self.interpolation.radius.value,dtype=float).reshape(-1).tolist(), "mass_density_kg_m3": np.asarray(self.beam_force_field.massDensity.value,dtype=float).reshape(-1).tolist()}
    def _schedule(self, start):
        zero = start < self.zero_end; measurement = self.mode == "stable_contact" and start >= self.measure_start
        return {"scheduled_force_n": [0.,0.,0.] if zero else [0.,-self.load,0.], "zero_load_phase": zero, "measurement_active": measurement, "command_active": not zero, "phase": "zero_load" if zero else "measurement" if measurement else "preload"}
    def onAnimateBeginEvent(self, _event):
        if self.material_snapshot is None: self.material_snapshot = self._material()
        start = float(self.root.time.value); schedule = self._schedule(start); force = schedule["scheduled_force_n"]
        self.force_field.forces.value = np.asarray([[*force,0.,0.,0.]],dtype=float)
        self.pending_step = {"step_index": int(self.next_step_index), "step_start_time_s": start, **schedule, "force_field_indices_begin": np.asarray(self.force_field.indices.value,dtype=int).reshape(-1).tolist(), "force_field_force_data_begin_n": _rows(self.force_field.forces.value).tolist()}
    def _contacts(self, serial):
        count=int(self.bridge.contactCount.value); beam=np.asarray(self.bridge.beamElementIds.value,dtype=int).reshape(-1); fixture=np.asarray(self.bridge.fixtureElementIds.value,dtype=int).reshape(-1)
        points=np.asarray(self.bridge.beamContactPoints.value,dtype=float).reshape(count,3); fpoints=np.asarray(self.bridge.fixtureContactPoints.value,dtype=float).reshape(count,3); normals=np.asarray(self.bridge.beamOutwardNormals.value,dtype=float).reshape(count,3); ids=np.asarray(self.bridge.contactIds.value,dtype=np.int64).reshape(-1)
        if any(a.size != count for a in (beam, fixture, ids)): raise RuntimeError("native bridge contact array length mismatch")
        return [{"source_frame_serial":serial,"contact_id":int(ids[i]),"beam_element_id":int(beam[i]),"fixture_element_id":int(fixture[i]),"beam_contact_point_m":points[i].tolist(),"fixture_contact_point_m":fpoints[i].tolist(),"beam_outward_normal":normals[i].tolist()} for i in range(count)]
    def onAnimateEndEvent(self, _event):
        if self.pending_step is None: raise RuntimeError("AnimateEnd without matching AnimateBegin")
        pending=self.pending_step; end=float(self.root.time.value); endrows=_rows(self.force_field.forces.value); endidx=np.asarray(self.force_field.indices.value,dtype=int).reshape(-1)
        serial=int(self.bridge.frameSerial.value); contacts=self._contacts(serial); pos=np.asarray(self.dofs.position.value,dtype=float); constraints=np.asarray(self.solver.constraintForces.value,dtype=float).reshape(-1)
        wall = self.config["baseline_sentinel"] if self.mode == "material_anchor_baseline" else self.config["contact_wall"]
        self.records.append({**pending,"step_end_time_s":end,"force_field_indices_end":endidx.tolist(),"force_field_force_data_end_n":endrows.tolist(),"actual_applied_force_n":np.asarray(pending["force_field_force_data_begin_n"],dtype=float).reshape(-1,6)[0,:3].tolist(),"bridge_ready":bool(self.bridge.ready.value),"bridge_frame_serial":serial,"native_contact_source_serial":serial,"native_contact_count":len(contacts),"native_contacts":contacts,"constraint_vector_size":int(constraints.size),"constraint_vector":constraints.tolist(),"reaction_force_proxy":float(np.linalg.norm(constraints)) if constraints.size else 0.,"tail_translation_displacement_m":float(np.linalg.norm(pos[0,:3]-self.initial_tail)),"minimum_centerline_to_fixture_m":float(np.min(np.abs(pos[:,1]-float(wall["plane_y_m"])))),"beam_node_positions_m":pos[:,:3].tolist(),"max_node_displacement_m":float(np.max(np.linalg.norm(pos[:,:3]-self.initial_positions,axis=1))),"max_transverse_deflection_m":float(np.max(np.abs(pos[:,1]-self.initial_positions[:,1])) )})
        self.next_step_index += 1; self.pending_step=None
        if len(self.records)>=self.steps: self._write()
    def _write(self):
        if not self.written:
            Path(os.environ["SOFA_STAGE_C_V2_TRACE"]).write_text(json.dumps({"metadata":{"mode":self.mode,"material_snapshot":self.material_snapshot,"actual_applied_force_definition":"Exact ConstantForceField.forces Data readback for configured target DOF during this step; not net system force and not contact reaction."},"records":self.records},indent=2)+"\n"); self.written=True
    def onSimulationEndedEvent(self,_event): self._write()


def createScene(root):
    c=_config(); mode=os.environ["SOFA_STAGE_C_V2_MODE"]; b,s=c["beam"],c["simulation"]; n=int(b["nodes"]); length,r=float(b["length_m"]),float(b["radius_m"]); steps=int(s["baseline_steps"] if mode=="material_anchor_baseline" else s["contact_steps"])
    root.gravity=[0.,0.,0.]; root.dt=float(s["dt_s"])
    root.addObject("RequiredPlugin",pluginName="BeamAdapter Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.Mapping.Linear Sofa.Component.MechanicalLoad Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant")
    root.addObject("FreeMotionAnimationLoop"); solver=root.addObject("LCPConstraintSolver",name="constraint_solver",mu=float(s["friction"]),tolerance=1e-8,maxIt=1000,build_lcp=False,computeConstraintForces=True); root.addObject("CollisionPipeline",depth=6); root.addObject("BruteForceBroadPhase"); root.addObject("BVHNarrowPhase",name="narrow_phase"); root.addObject("LocalMinDistance",alarmDistance=float(s["alarm_distance_m"]),contactDistance=float(s["contact_distance_m"])); root.addObject("CollisionResponse",response="FrictionContactConstraint",responseParams=f"mu={float(s['friction'])}")
    y=0. if mode=="material_anchor_baseline" else float(c["contact_wall"]["beam_initial_y_m"]); beam=root.addChild("Beam"); beam.addObject("EulerImplicitSolver",rayleighStiffness=0.05,rayleighMass=0.05); beam.addObject("BTDLinearSolver",verbose=False); dofs=beam.addObject("MechanicalObject",template="Rigid3d",name="dofs",position=[[i*length/(n-1),y,0.,0.,0.,0.,1.] for i in range(n)]); beam.addObject("MeshTopology",name="lines",lines=[[i,i+1] for i in range(n-1)])
    interpolation=beam.addObject("BeamInterpolation",name="interpolation",radius=[r],defaultYoungModulus=[float(b["young_modulus_pa"])],defaultPoissonRatio=[float(b["poisson_ratio"])]); beam_force_field=beam.addObject("AdaptiveBeamForceFieldAndMass",name="beam_force_field",computeMass=True,massDensity=[float(b["mass_density_kg_m3"])],interpolation="@interpolation"); beam.addObject("UncoupledConstraintCorrection",defaultCompliance=1e-6); tail=c["tail_constraint"]; beam.addObject("PartialFixedProjectiveConstraint",name="tail_translation_anchor",indices=[int(tail["node_index"])],fixedDirections=[int(v) for v in tail["fixed_directions"]]); force=beam.addObject("ConstantForceField",name="endpoint_force",indices=[n-1],forces=[0.]*6)
    collision=beam.addChild("Collision"); cdofs=collision.addObject("MechanicalObject",name="collision_dofs"); collision.addObject("IdentityMapping"); collision.addObject("PointCollisionModel",name="beam_points"); fixture=_fixture(root,"MaterialSentinel" if mode=="material_anchor_baseline" else "ContactWall",c["baseline_sentinel"] if mode=="material_anchor_baseline" else c["contact_wall"]); root.addObject("NativeContactBridge",name="native_contact_bridge",beamModel="@Beam/Collision/beam_points",fixtureModel="@MaterialSentinel/triangles" if mode=="material_anchor_baseline" else "@ContactWall/triangles",narrowPhase="@narrow_phase"); bridge=root.getObject("native_contact_bridge"); root.addObject(Controller(name="stage_c_v2_controller",root=root,dofs=dofs,collision_dofs=cdofs,solver=solver,force_field=force,bridge=bridge,interpolation=interpolation,beam_force_field=beam_force_field,config=c,mode=mode,steps=steps)); return root
