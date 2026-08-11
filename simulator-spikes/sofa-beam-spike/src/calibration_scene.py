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


def create_calibration_scene(root):
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


def _r1_wall(node, name, vertices, triangles):
    wall = node.addChild(name)
    wall.addObject("MeshTopology", name="topology", position=vertices, triangles=triangles)
    wall.addObject("MechanicalObject", name="dofs", position="@topology.position")
    return wall.addObject("TriangleCollisionModel", name="triangles", moving=False, simulated=False)


def _r1_native_contacts(listener, collision_positions, collision_velocities, tangent, fixture_normal):
    """Read the v26.06 Python-bound ContactListener payload without log parsing.

    The C++ v26.06 component exposes pair identity and contact points.  This
    branch is reached only if the active SofaPython3 binding exposes those
    methods; the Stage-A preflight rejects runtimes where it does not.  The
    fixture normal comes from explicitly constructed static triangle winding,
    never from a nearest-point proxy.
    """
    count = int(listener.getNumberOfContacts())
    elements = list(listener.getContactElements())
    points = list(listener.getContactPoints())
    if count and (len(elements) != count or len(points) != count):
        raise RuntimeError("ContactListener returned incomplete native contact data")
    records = []
    for index in range(count):
        model_a, element_a, model_b, element_b = elements[index]
        point_a_model, point_a, point_b_model, point_b = points[index]
        if int(model_a) == 0:
            beam_element, beam_point = int(element_a), point_a
        elif int(model_b) == 0:
            beam_element, beam_point = int(element_b), point_b
        else:
            raise RuntimeError("ContactListener could not identify the beam collision primitive")
        normal = np.asarray(fixture_normal, dtype=float)
        tangent_speed = float(abs(np.dot(np.asarray(collision_velocities)[beam_element, :3], tangent)))
        records.append(
            {
                "contact_id": index,
                "beam_element_id": beam_element,
                "fixture_element_id": int(element_b if int(model_a) == 0 else element_a),
                "contact_point_m": np.asarray(beam_point, dtype=float)[:3].tolist(),
                "contact_normal": normal[:3].tolist(),
                "tangent_speed_mps": tangent_speed,
                "normal_speed_mps": float(np.dot(np.asarray(collision_velocities)[beam_element, :3], normal[:3])),
            }
        )
    return records


class R1TraceRecorder(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.root = kwargs["root"]
        self.dofs = kwargs["dofs"]
        self.collision_dofs = kwargs["collision_dofs"]
        self.listener = kwargs.get("listener")
        self.solver = kwargs["solver"]
        self.force_field = kwargs["force_field"]
        self.mode = kwargs["mode"]
        self.config = kwargs["config"]
        self.expected_steps = int(kwargs["expected_steps"])
        self.forward_force = float(kwargs.get("forward_force", 0.0))
        self.initial_tip_x = float(np.asarray(self.dofs.position.value)[-1, 0])
        self.records = []
        self.api = {"listener_methods": sorted(name for name in dir(self.listener) if name.startswith("get"))} if self.listener is not None else {}
        self.written = False

    def _phase_force(self, time_s):
        normal = float(self.config["simulation"]["normal_preload_force_n"])
        if self.mode == "anchored_free_control":
            return ([0.0, -normal if time_s >= 0.1 else 0.0, 0.0], time_s >= 0.1, "preload" if time_s >= 0.1 else "settle", False)
        if self.mode == "normal_contact_hold":
            return ([0.0, -normal if time_s >= 0.1 else 0.0, 0.0], time_s >= 0.1, "measure" if time_s >= 0.3 else "preload" if time_s >= 0.1 else "settle", time_s >= 0.3)
        if self.mode == "breakaway_ramp":
            maximum = 2.0 * float(self.config["simulation"]["friction"]) * normal
            if time_s < 0.1:
                return ([0.0, 0.0, 0.0], False, "settle", False)
            if time_s < 0.3:
                return ([0.0, -normal, 0.0], True, "preload", False)
            fx = maximum * min(max((time_s - 0.3) / 0.6, 0.0), 1.0)
            return ([fx, -normal, 0.0], True, "ramp" if time_s < 0.9 else "hold", True)
        if self.mode in {"fresh_stick", "fresh_slip"}:
            if time_s < 0.1:
                return ([0.0, 0.0, 0.0], False, "settle", False)
            if time_s < 0.3:
                return ([0.0, -normal, 0.0], True, "preload", False)
            return ([self.forward_force, -normal, 0.0], True, "measure", True)
        if self.mode in {"free_forward", "blocked_jam"}:
            active = time_s >= 0.1
            return ([self.forward_force if active else 0.0, 0.0, 0.0], active, "measure" if active else "settle", active)
        return ([0.0, 0.0, 0.0], False, "api_audit", False)

    def onAnimateBeginEvent(self, _event):
        force, _active, _phase, _measurement = self._phase_force(float(self.root.time.value))
        self.force_field.forces.value = np.asarray([[force[0], force[1], force[2], 0.0, 0.0, 0.0]], dtype=float)

    def _write(self):
        if not self.written:
            Path(os.environ["SOFA_CONTACT_R1_TRACE"]).write_text(json.dumps({"api": self.api, "records": self.records}))
            self.written = True

    def onAnimateEndEvent(self, _event):
        time_s = float(self.root.time.value)
        force, command_active, phase, measurement_active = self._phase_force(time_s)
        positions = np.asarray(self.dofs.position.value, dtype=float)
        velocities = np.asarray(self.dofs.velocity.value, dtype=float)
        collision_positions = np.asarray(self.collision_dofs.position.value, dtype=float)
        collision_velocities = np.asarray(self.collision_dofs.velocity.value, dtype=float)
        tangent = np.asarray([1.0, 0.0, 0.0] if self.mode not in {"free_forward", "blocked_jam"} else [0.0, 1.0, 0.0])
        fixture_normal = np.asarray([0.0, 1.0, 0.0] if self.mode not in {"blocked_jam"} else [1.0, 0.0, 0.0])
        contacts = _r1_native_contacts(self.listener, collision_positions, collision_velocities, tangent, fixture_normal) if self.listener is not None else []
        constraints = np.asarray(self.solver.constraintForces.value, dtype=float)
        nearest = int(np.argmin(collision_positions[:, 1]))
        native_speeds = [record["tangent_speed_mps"] for record in contacts]
        self.records.append(
            {
                "timestamp_s": time_s,
                "beam_node_positions_m": positions[:, :3].tolist(),
                "beam_node_velocities_mps": velocities[:, :3].tolist(),
                "collision_positions_m": collision_positions[:, :3].tolist(),
                "collision_velocities_mps": collision_velocities[:, :3].tolist(),
                "command_force_n": force,
                "command_active": command_active,
                "phase": phase,
                "measurement_active": measurement_active,
                "native_contact_count": len(contacts),
                "native_contacts": contacts,
                "constraint_vector_size": int(constraints.size),
                "reaction_force_proxy": float(np.linalg.norm(constraints)) if constraints.size else 0.0,
                "native_contact_tangent_speed_median": float(np.median(native_speeds)) if native_speeds else 0.0,
                "native_contact_tangent_speed_max": float(np.max(native_speeds)) if native_speeds else 0.0,
                "endpoint_tangent_speed_mps": abs(float(velocities[-1, 0])),
                "nearest_plane_proxy_tangent_speed_mps": abs(float(collision_velocities[nearest, 0])),
                "nearest_beam_node_proxy_tangent_speed_mps": abs(float(velocities[nearest, 0])),
                "task_progress_m": float(positions[-1, 0] - self.initial_tip_x),
            }
        )
        if len(self.records) >= self.expected_steps:
            self._write()

    def onSimulationEndedEvent(self, _event):
        self._write()


def create_r1_scene(root):
    config = json.loads(Path(os.environ["SOFA_CONTACT_R1_CONFIG"]).read_text())
    mode = os.environ["SOFA_CONTACT_R1_MODE"]
    beam = config["beam"]
    simulation = config["simulation"]
    length = float(beam["length_m"])
    nodes = int(beam["nodes"])
    spacing = length / (nodes - 1)
    radius = float(beam["radius_m"])
    horizontal_wall = mode in {"native_contact_audit", "normal_contact_hold", "breakaway_ramp", "fresh_stick", "fresh_slip"}
    blocker = mode == "blocked_jam"
    anchored = mode not in {"free_forward", "blocked_jam"}
    y0 = float(simulation["initial_clearance_m"]) if horizontal_wall else 0.02
    expected_steps = int(os.environ["SOFA_CONTACT_R1_STEPS"])
    forward_force = float(os.environ.get("SOFA_CONTACT_R1_FORWARD_FORCE", "0"))

    root.gravity = [0.0, 0.0, 0.0]
    root.dt = float(simulation["dt_s"])
    root.addObject("RequiredPlugin", pluginName="BeamAdapter Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.Mapping.Linear Sofa.Component.MechanicalLoad Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant")
    root.addObject("FreeMotionAnimationLoop")
    solver = root.addObject("LCPConstraintSolver", name="constraint_solver", mu=float(simulation["friction"]), tolerance=1e-8, maxIt=1000, build_lcp=False, computeConstraintForces=True)
    root.addObject("CollisionPipeline", depth=6)
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("LocalMinDistance", alarmDistance=0.01, contactDistance=float(simulation["contact_distance_m"]))
    root.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams=f"mu={float(simulation['friction'])}")

    positions = [[index * spacing, y0, 0.0, 0.0, 0.0, 0.0, 1.0] for index in range(nodes)]
    beam_node = root.addChild("Beam")
    beam_node.addObject("EulerImplicitSolver", rayleighStiffness=0.05, rayleighMass=0.05)
    beam_node.addObject("BTDLinearSolver", verbose=False)
    dofs = beam_node.addObject("MechanicalObject", template="Rigid3d", name="dofs", position=positions)
    beam_node.addObject("MeshTopology", name="lines", lines=[[index, index + 1] for index in range(nodes - 1)])
    beam_node.addObject("BeamInterpolation", name="interpolation", radius=radius)
    beam_node.addObject("AdaptiveBeamForceFieldAndMass", name="beam_force_field", computeMass=True, massDensity=float(beam["mass_density_kg_m3"]))
    beam_node.addObject("UncoupledConstraintCorrection", defaultCompliance=1e-6)
    if anchored:
        beam_node.addObject("PartialFixedProjectiveConstraint", indices=[0], fixedDirections=[1, 1, 1, 0, 0, 0])
    force_field = beam_node.addObject("ConstantForceField", name="endpoint_force", indices=[nodes - 1], forces=[0.0] * 6)
    collision = beam_node.addChild("Collision")
    collision_dofs = collision.addObject("MechanicalObject", name="collision_dofs")
    collision.addObject("IdentityMapping")
    beam_points = collision.addObject("PointCollisionModel", name="beam_points")

    fixture_model = None
    fixture_path = None
    if horizontal_wall:
        fixture_model = _r1_wall(root, "horizontal_wall", [[-0.02, 0.0, -0.05], [length + 0.06, 0.0, -0.05], [length + 0.06, 0.0, 0.05], [-0.02, 0.0, 0.05]], [[0, 1, 2], [0, 2, 3]])
        fixture_path = "@horizontal_wall/triangles"
    elif blocker:
        x = length + float(simulation["initial_clearance_m"])
        fixture_model = _r1_wall(root, "forward_blocker", [[x, -0.05, -0.05], [x, 0.05, -0.05], [x, 0.05, 0.05], [x, -0.05, 0.05]], [[0, 1, 2], [0, 2, 3]])
        fixture_path = "@forward_blocker/triangles"
    listener = None
    if fixture_path is not None:
        root.addObject("ContactListener", name="native_contact_listener", collisionModel1="@Beam/Collision/beam_points", collisionModel2=fixture_path)
        listener = root.getObject("native_contact_listener")
    root.addObject(R1TraceRecorder(name="r1_trace", root=root, dofs=dofs, collision_dofs=collision_dofs, listener=listener, solver=solver, force_field=force_field, mode=mode, config=config, expected_steps=expected_steps, forward_force=forward_force))
    return root


def createScene(root):
    if os.environ.get("SOFA_CONTACT_R1_CONFIG"):
        return create_r1_scene(root)
    return create_calibration_scene(root)
