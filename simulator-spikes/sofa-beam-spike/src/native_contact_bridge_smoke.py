"""20-step BeamAdapter native-contact bridge smoke; no regime measurement."""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import Sofa


def _config() -> dict:
    return json.loads(Path(os.environ["SOFA_NATIVE_BRIDGE_CONFIG"]).read_text())


def _xyz(value, count: int) -> np.ndarray:
    data = np.asarray(value, dtype=float).reshape(-1)
    if data.size != 3 * count:
        raise RuntimeError(f"native bridge xyz length {data.size} != {3 * count}")
    return data.reshape(count, 3)


class BridgeRecorder(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.root = kwargs["root"]
        self.bridge = kwargs["bridge"]
        self.collision_dofs = kwargs["collision_dofs"]
        self.fixture_triangle_count = int(kwargs["fixture_triangle_count"])
        self.expected_steps = int(kwargs["expected_steps"])
        self.records: list[dict] = []
        self.written = False

    def _read_frame(self) -> dict:
        count = int(self.bridge.contactCount.value)
        ids = np.asarray(self.bridge.contactIds.value, dtype=np.int64).reshape(-1)
        beam_ids = np.asarray(self.bridge.beamElementIds.value, dtype=int).reshape(-1)
        fixture_ids = np.asarray(self.bridge.fixtureElementIds.value, dtype=int).reshape(-1)
        detection_values = np.asarray(self.bridge.detectionValues.value, dtype=float).reshape(-1)
        for name, values in {"contact_ids": ids, "beam_ids": beam_ids, "fixture_ids": fixture_ids, "detection_values": detection_values}.items():
            if values.size != count:
                raise RuntimeError(f"{name} length {values.size} != {count}")
        return {
            "timestamp_s": float(self.root.time.value),
            "ready": bool(self.bridge.ready.value),
            "frame_serial": int(self.bridge.frameSerial.value),
            "contact_count": count,
            "collision_point_count": int(np.asarray(self.collision_dofs.position.value).shape[0]),
            "fixture_triangle_count": self.fixture_triangle_count,
            "contact_ids": ids.tolist(),
            "beam_element_ids": beam_ids.tolist(),
            "fixture_element_ids": fixture_ids.tolist(),
            "beam_contact_points": _xyz(self.bridge.beamContactPoints.value, count).tolist(),
            "fixture_contact_points": _xyz(self.bridge.fixtureContactPoints.value, count).tolist(),
            "beam_outward_normals": _xyz(self.bridge.beamOutwardNormals.value, count).tolist(),
            "detection_values": detection_values.tolist(),
        }

    def _write(self) -> None:
        if not self.written:
            Path(os.environ["SOFA_NATIVE_BRIDGE_TRACE"]).write_text(json.dumps(self.records, indent=2) + "\n")
            self.written = True

    def onAnimateEndEvent(self, _event):
        self.records.append(self._read_frame())
        if len(self.records) >= self.expected_steps:
            self._write()

    def onSimulationEndedEvent(self, _event):
        self._write()


def _wall(root, length: float):
    wall = root.addChild("NativeBridgeWall")
    vertices = [[-0.02, 0.0, -0.05], [length + 0.06, 0.0, -0.05], [length + 0.06, 0.0, 0.05], [-0.02, 0.0, 0.05]]
    wall.addObject("MeshTopology", name="topology", position=vertices, triangles=[[0, 1, 2], [0, 2, 3]])
    wall.addObject("MechanicalObject", name="dofs", position="@topology.position")
    # The smoke is deliberately a two-sided static contact surface so the
    # narrow phase is tested independently of triangle winding conventions.
    return wall.addObject("TriangleCollisionModel", name="triangles", moving=False, simulated=False, bothSide=True), 2


def createScene(root):
    config = _config()
    beam, simulation, smoke = config["beam"], config["simulation"], config["smoke"]
    length, radius, nodes = float(beam["length_m"]), float(beam["radius_m"]), int(beam["nodes"])
    spacing = length / (nodes - 1)
    root.gravity, root.dt = [0.0, 0.0, 0.0], float(simulation["dt_s"])
    root.addObject("RequiredPlugin", pluginName="BeamAdapter Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.LinearSolver.Direct Sofa.Component.Mapping.Linear Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant")
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("LCPConstraintSolver", name="constraint_solver", mu=float(simulation["friction"]), tolerance=1e-8, maxIt=1000, build_lcp=False, computeConstraintForces=True)
    root.addObject("CollisionPipeline", depth=6)
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase", name="narrow_phase")
    root.addObject("LocalMinDistance", alarmDistance=0.01, contactDistance=radius)
    root.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams=f"mu={float(simulation['friction'])}")

    y0 = float(smoke["wall_clearance_fraction_of_contact_distance"]) * radius
    beam_node = root.addChild("Beam")
    beam_node.addObject("EulerImplicitSolver", rayleighStiffness=0.05, rayleighMass=0.05)
    beam_node.addObject("BTDLinearSolver", verbose=False)
    beam_node.addObject("MechanicalObject", template="Rigid3d", name="dofs", position=[[i * spacing, y0, 0.0, 0.0, 0.0, 0.0, 1.0] for i in range(nodes)])
    beam_node.addObject("MeshTopology", name="lines", lines=[[i, i + 1] for i in range(nodes - 1)])
    beam_node.addObject("BeamInterpolation", name="interpolation", radius=radius)
    beam_node.addObject("AdaptiveBeamForceFieldAndMass", name="beam_force_field", computeMass=True, massDensity=float(beam["mass_density_kg_m3"]))
    beam_node.addObject("UncoupledConstraintCorrection", defaultCompliance=1e-6)
    collision = beam_node.addChild("Collision")
    collision_dofs = collision.addObject("MechanicalObject", name="collision_dofs")
    collision.addObject("IdentityMapping")
    collision.addObject("PointCollisionModel", name="points")
    _, triangle_count = _wall(root, length)
    root.addObject("NativeContactBridge", name="native_contact_bridge", beamModel="@Beam/Collision/points", fixtureModel="@NativeBridgeWall/triangles", narrowPhase="@narrow_phase")
    bridge = root.getObject("native_contact_bridge")
    root.addObject(BridgeRecorder(name="bridge_recorder", root=root, bridge=bridge, collision_dofs=collision_dofs, fixture_triangle_count=triangle_count, expected_steps=int(simulation["steps"])))
    return root
