"""Stage B: tail projective constraint versus LCP constraintForces."""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import Sofa


def _config() -> dict:
    return json.loads(Path(os.environ["SOFA_STAGE_B_CONFIG"]).read_text())


def _sentinel_fixture(root, config: dict):
    sentinel = config["sentinel_fixture"]
    y, x0, x1, z = float(sentinel["plane_y_m"]), float(sentinel["x_min_m"]), float(sentinel["x_max_m"]), float(sentinel["z_half_extent_m"])
    node = root.addChild("SentinelFixture")
    node.addObject("MeshTopology", name="topology", position=[[x0, y, -z], [x1, y, -z], [x1, y, z], [x0, y, z]], triangles=[[0, 1, 2], [0, 2, 3]])
    node.addObject("MechanicalObject", name="dofs", position="@topology.position")
    node.addObject("TriangleCollisionModel", name="triangles", moving=False, simulated=False, bothSide=True)


class StageBController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.root, self.dofs, self.solver = kwargs["root"], kwargs["dofs"], kwargs["solver"]
        self.force_field, self.bridge, self.config = kwargs["force_field"], kwargs["bridge"], kwargs["config"]
        simulation = self.config["simulation"]
        self.steps, self.load_n = int(simulation["steps"]), float(simulation["normal_load_n"])
        self.settle_s, self.measurement_start_s = float(simulation["settle_ms"]) / 1000.0, float(simulation["measurement_start_ms"]) / 1000.0
        initial = np.asarray(self.dofs.position.value, dtype=float)
        self.initial_tail_xyz, self.initial_tip_xyz = initial[0, :3].copy(), initial[-1, :3].copy()
        self.records, self.written = [], False

    def _force(self, time_s: float) -> list[float]:
        return [0.0, -self.load_n, 0.0] if time_s >= self.settle_s else [0.0, 0.0, 0.0]

    def onAnimateBeginEvent(self, _event):
        force = self._force(float(self.root.time.value))
        self.force_field.forces.value = np.asarray([[*force, 0.0, 0.0, 0.0]], dtype=float)

    def _write(self):
        if not self.written:
            Path(os.environ["SOFA_STAGE_B_TRACE"]).write_text(json.dumps({"metadata": {"tail_constraint": "PartialFixedProjectiveConstraint", "fixed_directions": [1, 1, 1, 0, 0, 0], "sentinel_plane_y_m": float(self.config["sentinel_fixture"]["plane_y_m"])}, "records": self.records}, indent=2) + "\n")
            self.written = True

    def onAnimateEndEvent(self, _event):
        time_s = float(self.root.time.value)
        positions = np.asarray(self.dofs.position.value, dtype=float)
        constraints = np.asarray(self.solver.constraintForces.value, dtype=float).reshape(-1)
        sentinel_y = float(self.config["sentinel_fixture"]["plane_y_m"])
        self.records.append({
            "timestamp_s": time_s, "measurement_active": bool(time_s >= self.measurement_start_s),
            "command_force_n": self._force(time_s), "command_active": bool(time_s >= self.settle_s),
            "bridge_ready": bool(self.bridge.ready.value), "bridge_frame_serial": int(self.bridge.frameSerial.value),
            "native_contact_count": int(self.bridge.contactCount.value),
            "native_contact_ids": [int(value) for value in self.bridge.contactIds.value],
            "constraint_vector_size": int(constraints.size), "constraint_vector": constraints.tolist(),
            "reaction_force_proxy": float(np.linalg.norm(constraints)) if constraints.size else 0.0,
            "tail_translation_displacement_m": float(np.linalg.norm(positions[0, :3] - self.initial_tail_xyz)),
            "tip_displacement_m": float(np.linalg.norm(positions[-1, :3] - self.initial_tip_xyz)),
            "minimum_centerline_to_sentinel_m": float(np.min(np.abs(positions[:, 1] - sentinel_y))),
        })
        if len(self.records) >= self.steps: self._write()

    def onSimulationEndedEvent(self, _event): self._write()


def createScene(root):
    config = _config(); beam, simulation = config["beam"], config["simulation"]
    length, radius, nodes = float(beam["length_m"]), float(beam["radius_m"]), int(beam["nodes"])
    root.gravity, root.dt = [0.0, 0.0, 0.0], float(simulation["dt_s"])
    root.addObject("RequiredPlugin", pluginName="BeamAdapter Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.Mapping.Linear Sofa.Component.MechanicalLoad Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant")
    root.addObject("FreeMotionAnimationLoop")
    solver = root.addObject("LCPConstraintSolver", name="constraint_solver", mu=float(simulation["friction"]), tolerance=1e-8, maxIt=1000, build_lcp=False, computeConstraintForces=True)
    root.addObject("CollisionPipeline", depth=6); root.addObject("BruteForceBroadPhase"); root.addObject("BVHNarrowPhase", name="narrow_phase")
    root.addObject("LocalMinDistance", alarmDistance=0.01, contactDistance=radius)
    root.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams=f"mu={float(simulation['friction'])}")
    beam_node = root.addChild("Beam")
    beam_node.addObject("EulerImplicitSolver", rayleighStiffness=0.05, rayleighMass=0.05); beam_node.addObject("BTDLinearSolver", verbose=False)
    dofs = beam_node.addObject("MechanicalObject", template="Rigid3d", name="dofs", position=[[i * length / (nodes - 1), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] for i in range(nodes)])
    beam_node.addObject("MeshTopology", name="lines", lines=[[i, i + 1] for i in range(nodes - 1)]); beam_node.addObject("BeamInterpolation", name="interpolation", radius=radius)
    beam_node.addObject("AdaptiveBeamForceFieldAndMass", name="beam_force_field", computeMass=True, massDensity=float(beam["mass_density_kg_m3"])); beam_node.addObject("UncoupledConstraintCorrection", defaultCompliance=1e-6)
    tail = config["tail_constraint"]
    beam_node.addObject("PartialFixedProjectiveConstraint", name="tail_translation_anchor", indices=[int(tail["node_index"])], fixedDirections=[int(value) for value in tail["fixed_directions"]])
    force_field = beam_node.addObject("ConstantForceField", name="endpoint_force", indices=[nodes - 1], forces=[0.0] * 6)
    collision = beam_node.addChild("Collision"); collision.addObject("MechanicalObject", name="collision_dofs"); collision.addObject("IdentityMapping"); collision.addObject("PointCollisionModel", name="beam_points")
    _sentinel_fixture(root, config)
    root.addObject("NativeContactBridge", name="native_contact_bridge", beamModel="@Beam/Collision/beam_points", fixtureModel="@SentinelFixture/triangles", narrowPhase="@narrow_phase")
    bridge = root.getObject("native_contact_bridge")
    root.addObject(StageBController(name="stage_b_controller", root=root, dofs=dofs, solver=solver, force_field=force_field, bridge=bridge, config=config))
    return root
