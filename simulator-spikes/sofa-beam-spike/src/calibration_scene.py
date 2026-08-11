"""Controlled BeamAdapter traces for the one-shot SOFA Oracle calibration."""

import json
import os
from pathlib import Path

import numpy as np
import Sofa


def _configuration():
    return json.loads(Path(os.environ["SOFA_CALIBRATION_CONFIG"]).read_text())


def _wall(node, name, vertices, triangles):
    wall = node.addChild(name)
    wall.addObject("MeshTopology", name="topology", position=vertices, triangles=triangles)
    wall.addObject("MechanicalObject", name="dofs", position="@topology.position")
    wall.addObject("TriangleCollisionModel", moving=False, simulated=False)
    return wall


class TraceRecorder(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.root = kwargs["root"]
        self.dofs = kwargs["dofs"]
        self.solver = kwargs["solver"]
        self.command_force = [float(value) for value in kwargs["command_force"]]
        self.command_active = bool(kwargs["command_active"])
        self.wall_identity = kwargs["wall_identity"]
        self.expected_steps = int(kwargs["expected_steps"])
        self.initial_tip_x = float(np.asarray(self.dofs.position.value)[-1, 0])
        self.records = []
        self.written = False

    def _write(self):
        if not self.written:
            Path(os.environ["SOFA_CALIBRATION_TRACE"]).write_text(json.dumps(self.records))
            self.written = True

    def onAnimateEndEvent(self, _event):
        positions = np.asarray(self.dofs.position.value, dtype=float)
        velocities = np.asarray(self.dofs.velocity.value, dtype=float)
        constraints = np.asarray(self.solver.constraintForces.value, dtype=float)
        # FrictionContactConstraint emits normal/tangent triplets.  The solver
        # vector is global, so its norm is deliberately recorded as a proxy.
        active_contact = bool(constraints.size)
        contact_count = int(constraints.size // 3) if active_contact else 0
        reaction = float(np.linalg.norm(constraints)) if active_contact else 0.0
        self.records.append(
            {
                "timestamp_s": float(self.root.time.value),
                "beam_node_positions_m": positions[:, :3].tolist(),
                "beam_node_velocities_mps": velocities[:, :3].tolist(),
                "command_force_n": self.command_force,
                "command_active": self.command_active,
                "contact_count_proxy": contact_count,
                "active_contact": active_contact,
                "contact_wall_identity": self.wall_identity if active_contact else None,
                "reaction_force_proxy": reaction,
                "wall_tangent_relative_speed_mps": abs(float(velocities[-1, 0])),
                "task_progress_m": float(positions[-1, 0] - self.initial_tip_x),
            }
        )
        if len(self.records) >= self.expected_steps:
            self._write()

    def onSimulationEndedEvent(self, _event):
        self._write()


def createScene(root):
    config = _configuration()
    mode = os.environ["SOFA_CALIBRATION_MODE"]
    beam = config["beam"]
    simulation = config["simulation"]
    commands = config["commands"]
    length = float(beam["length_m"])
    radius = float(beam["radius_m"])
    nodes = int(beam["nodes"])
    spacing = length / (nodes - 1)

    root.gravity = [0.0, 0.0, 0.0]
    root.dt = float(simulation["dt_s"])
    root.addObject(
        "RequiredPlugin",
        pluginName=(
            "BeamAdapter Sofa.Component.AnimationLoop "
            "Sofa.Component.Collision.Detection.Algorithm "
            "Sofa.Component.Collision.Detection.Intersection "
            "Sofa.Component.Collision.Geometry "
            "Sofa.Component.Collision.Response.Contact "
            "Sofa.Component.Constraint.Lagrangian.Correction "
            "Sofa.Component.Constraint.Lagrangian.Solver "
            "Sofa.Component.LinearSolver.Direct Sofa.Component.MechanicalLoad "
            "Sofa.Component.Mapping.Linear "
            "Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer "
            "Sofa.Component.Topology.Container.Constant"
        ),
    )
    root.addObject("FreeMotionAnimationLoop")
    solver = root.addObject(
        "LCPConstraintSolver",
        name="constraint_solver",
        mu=float(simulation["friction"]),
        tolerance=1e-8,
        maxIt=1000,
        build_lcp=False,
        computeConstraintForces=True,
    )
    root.addObject("CollisionPipeline", depth=6)
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("LocalMinDistance", alarmDistance=0.01, contactDistance=radius)
    root.addObject(
        "CollisionResponse",
        response="FrictionContactConstraint",
        responseParams=f"mu={float(simulation['friction'])}",
    )

    horizontal = mode in {"stick_calibration", "slip_calibration"}
    blocker = mode == "jam_calibration"
    y0 = radius * 2.5 if horizontal else 0.02
    positions = [[index * spacing, y0, 0.0, 0.0, 0.0, 0.0, 1.0] for index in range(nodes)]
    beam_node = root.addChild("Beam")
    beam_node.addObject("EulerImplicitSolver", rayleighStiffness=0.05, rayleighMass=0.05)
    beam_node.addObject("BTDLinearSolver", verbose=False)
    dofs = beam_node.addObject("MechanicalObject", template="Rigid3d", name="dofs", position=positions)
    beam_node.addObject("MeshTopology", name="lines", lines=[[index, index + 1] for index in range(nodes - 1)])
    beam_node.addObject("BeamInterpolation", name="interpolation", radius=radius)
    beam_node.addObject(
        "AdaptiveBeamForceFieldAndMass",
        name="beam_force_field",
        computeMass=True,
        massDensity=float(beam["mass_density_kg_m3"]),
    )
    # The official constrained-contact tutorials use this correction for
    # rigid-coordinate collision models.  LinearSolverConstraintCorrection is
    # used by the deployment example, but that combination is not valid for
    # this fixed-topology direct BeamInterpolation scene.
    beam_node.addObject("UncoupledConstraintCorrection", defaultCompliance=1e-6)

    forward_force = float(commands["forward_force_n"])
    preload = float(commands["normal_preload_force_n"])
    if mode == "stick_calibration":
        force = [0.0, -preload, 0.0, 0.0, 0.0, 0.0]
    elif mode == "slip_calibration":
        force = [forward_force, -preload, 0.0, 0.0, 0.0, 0.0]
    elif mode in {"free_forward_calibration", "jam_calibration", "force_sanity"}:
        force = [forward_force, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        force = [0.0] * 6
    beam_node.addObject("ConstantForceField", name="endpoint_force", indices=[nodes - 1], forces=force)
    collision = beam_node.addChild("Collision")
    collision.addObject("MechanicalObject", name="collision_dofs")
    collision.addObject("IdentityMapping")
    collision.addObject("PointCollisionModel")

    wall_identity = "none"
    if horizontal:
        _wall(
            root,
            "horizontal_wall",
            [[-0.02, 0.0, -0.05], [length + 0.06, 0.0, -0.05], [length + 0.06, 0.0, 0.05], [-0.02, 0.0, 0.05]],
            [[0, 1, 2], [0, 2, 3]],
        )
        wall_identity = "horizontal_wall"
    elif blocker:
        x = length + 0.008
        _wall(
            root,
            "forward_blocker",
            [[x, -0.05, -0.05], [x, 0.05, -0.05], [x, 0.05, 0.05], [x, -0.05, 0.05]],
            [[0, 1, 2], [0, 2, 3]],
        )
        wall_identity = "forward_blocker"

    root.addObject(
        TraceRecorder(
            name="trace_recorder",
            root=root,
            dofs=dofs,
            solver=solver,
            command_force=force[:3],
            command_active=mode != "free_calibration",
            wall_identity=wall_identity,
            expected_steps=int(simulation["steps"]),
        )
    )
    return root
