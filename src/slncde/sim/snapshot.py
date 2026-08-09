import numpy as np
import pybullet as p


def capture_world_state(env, task):
    bead_pose = [p.getBasePositionAndOrientation(int(x)) for x in task.cable_bead_IDs]
    bead_vel = [p.getBaseVelocity(int(x)) for x in task.cable_bead_IDs]
    joint = [p.getJointState(env.ur5, int(j)) for j in env.joints]
    ee = p.getLinkState(env.ur5, env.ee_tip_link, computeLinkVelocity=1, computeForwardKinematics=True)
    return {
        "bead_positions": np.asarray([x[0] for x in bead_pose], dtype=np.float64),
        "bead_linear_velocities": np.asarray([x[0] for x in bead_vel], dtype=np.float64),
        "joint_positions": np.asarray([x[0] for x in joint], dtype=np.float64),
        "joint_velocities": np.asarray([x[1] for x in joint], dtype=np.float64),
        "ee_position": np.asarray(ee[0], dtype=np.float64),
        "ee_orientation": np.asarray(ee[1], dtype=np.float64),
    }


def capture_runtime_state(env):
    constraint = getattr(env.ee, "contact_constraint", None)
    return {"ee_activated": bool(getattr(env.ee, "activated", False)), "ee_contact_constraint": None if constraint is None else int(constraint)}


def restore_runtime_state(env, state):
    env.ee.activated = bool(state["ee_activated"])
    env.ee.contact_constraint = state["ee_contact_constraint"]


def restore_joint_control_state(env, world_state):
    targets = np.asarray(world_state["joint_positions"], dtype=np.float64)
    p.setJointMotorControlArray(bodyIndex=env.ur5, jointIndices=env.joints, controlMode=p.POSITION_CONTROL, targetPositions=targets.tolist(), positionGains=np.ones(len(env.joints)))


def cable_rmse(a, b):
    return float(np.sqrt(np.mean((np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)) ** 2)))
