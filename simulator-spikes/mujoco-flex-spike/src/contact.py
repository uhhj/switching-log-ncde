from __future__ import annotations

from typing import Any, Dict, Iterable

import mujoco
import numpy as np


def extract_contact(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    flex_id: int,
    target_geom_id: int,
    relative_velocity_xyz,
) -> Dict[str, Any]:
    """Aggregate active flex/geom contacts in Phase 0C0-compatible units."""
    aggregate = extract_contacts(
        model,
        data,
        flex_id,
        [target_geom_id],
        relative_velocity_xyz,
    )
    return {
        "contact_count": aggregate["contact_count"],
        "normal_force": aggregate["normal_force_sum"],
        "tangent_force": aggregate["tangent_force_sum"],
        "tangent_velocity": aggregate["tangent_velocity_mean"],
        "relative_velocity": aggregate["relative_velocity"],
    }


def extract_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    flex_id: int,
    target_geom_ids: Iterable[int],
    relative_velocity_xyz,
) -> Dict[str, Any]:
    """Aggregate active contacts over a named fixture channel."""
    relative_velocity = np.asarray(relative_velocity_xyz, dtype=np.float64)
    targets = {int(value) for value in target_geom_ids}
    normal_forces = []
    tangent_forces = []
    tangent_velocities = []
    element_ids = []
    vertex_ids = []
    for contact_id in range(int(data.ncon)):
        contact = data.contact[contact_id]
        if targets.isdisjoint(int(value) for value in contact.geom):
            continue
        flex_sides = [
            index
            for index, value in enumerate(contact.flex)
            if int(value) == int(flex_id)
        ]
        if not flex_sides:
            continue
        if int(contact.efc_address) < 0:
            continue
        force_torque = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, contact_id, force_torque)
        normal_force = abs(float(force_torque[0]))
        if normal_force <= 0.0:
            continue
        normal = np.asarray(contact.frame[:3], dtype=np.float64)
        tangent_velocity = relative_velocity - np.dot(relative_velocity, normal) * normal
        normal_forces.append(normal_force)
        tangent_forces.append(float(np.linalg.norm(force_torque[1:3])))
        tangent_velocities.append(float(np.linalg.norm(tangent_velocity)))
        flex_side = flex_sides[0]
        element_ids.append(int(contact.elem[flex_side]))
        vertex_ids.append(int(contact.vert[flex_side]))
    return {
        "contact_count": len(normal_forces),
        "normal_force_sum": float(np.sum(normal_forces)) if normal_forces else 0.0,
        "normal_force_max": float(np.max(normal_forces)) if normal_forces else 0.0,
        "tangent_force_sum": float(np.sum(tangent_forces)) if tangent_forces else 0.0,
        "tangent_force_max": float(np.max(tangent_forces)) if tangent_forces else 0.0,
        "tangent_velocity_mean": (
            float(np.mean(tangent_velocities)) if tangent_velocities else 0.0
        ),
        "tangent_velocity_max": (
            float(np.max(tangent_velocities)) if tangent_velocities else 0.0
        ),
        "relative_velocity": relative_velocity.copy(),
        "element_ids": element_ids,
        "vertex_ids": vertex_ids,
    }
