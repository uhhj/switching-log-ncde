from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Tuple

import mujoco
import numpy as np


class FlexSimulator:
    """Small CPU-only MuJoCo world containing one 1D flex cable."""

    def __init__(self, model_path: Path = None, model: mujoco.MjModel = None):
        if model is None:
            if model_path is None:
                raise ValueError("model_path or model is required")
            model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.model = model
        self.data = mujoco.MjData(self.model)
        self.flex_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_FLEX, "cable"
        )
        self.floor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        self.wall_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "wall"
        )
        first_vertex = int(self.model.flex_vertadr[self.flex_id])
        vertex_count = int(self.model.flex_vertnum[self.flex_id])
        self.vertex_body_ids = np.asarray(
            self.model.flex_vertbodyid[
                first_vertex : first_vertex + vertex_count
            ],
            dtype=np.int64,
        ).copy()
        self.endpoint_body_id = int(
            self.vertex_body_ids[-1]
        )
        self._wall_contype = (
            int(self.model.geom_contype[self.wall_id]) if self.wall_id >= 0 else 0
        )
        self._wall_conaffinity = (
            int(self.model.geom_conaffinity[self.wall_id]) if self.wall_id >= 0 else 0
        )

    @classmethod
    def from_xml_string(cls, xml: str) -> "FlexSimulator":
        return cls(model=mujoco.MjModel.from_xml_string(xml))

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def reset(self, gravity_z: float, wall_enabled: bool) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.model.opt.gravity[:] = [0.0, 0.0, float(gravity_z)]
        if self.wall_id >= 0:
            if wall_enabled:
                self.model.geom_contype[self.wall_id] = self._wall_contype
                self.model.geom_conaffinity[self.wall_id] = self._wall_conaffinity
            else:
                self.model.geom_contype[self.wall_id] = 0
                self.model.geom_conaffinity[self.wall_id] = 0
        mujoco.mj_forward(self.model, self.data)

    def endpoint_state(self) -> Tuple[np.ndarray, np.ndarray]:
        position = np.asarray(
            self.data.xpos[self.endpoint_body_id], dtype=np.float64
        ).copy()
        spatial_velocity = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.endpoint_body_id,
            spatial_velocity,
            0,
        )
        return position, spatial_velocity[3:].copy()

    def vertex_positions(self) -> np.ndarray:
        return np.asarray(
            self.data.xpos[self.vertex_body_ids], dtype=np.float64
        ).copy()

    def vertex_velocities(self) -> np.ndarray:
        velocities = []
        for body_id in self.vertex_body_ids:
            spatial_velocity = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(
                self.model,
                self.data,
                mujoco.mjtObj.mjOBJ_BODY,
                int(body_id),
                spatial_velocity,
                0,
            )
            velocities.append(spatial_velocity[3:].copy())
        return np.asarray(velocities, dtype=np.float64)

    def translate_vertices(self, delta_xyz) -> None:
        delta = np.asarray(delta_xyz, dtype=np.float64)
        for body_id in self.vertex_body_ids:
            joint_start = int(self.model.body_jntadr[int(body_id)])
            joint_count = int(self.model.body_jntnum[int(body_id)])
            for joint_id in range(joint_start, joint_start + joint_count):
                qpos_address = int(self.model.jnt_qposadr[joint_id])
                axis = np.asarray(self.model.jnt_axis[joint_id], dtype=np.float64)
                self.data.qpos[qpos_address] += float(np.dot(delta, axis))
        mujoco.mj_forward(self.model, self.data)

    def copy_state(self) -> Dict[str, Any]:
        return {
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
            "act": self.data.act.copy(),
            "time": float(self.data.time),
            "mocap_pos": self.data.mocap_pos.copy(),
            "mocap_quat": self.data.mocap_quat.copy(),
        }

    def restore_state(self, state: Dict[str, Any]) -> None:
        self.data.qpos[:] = state["qpos"]
        self.data.qvel[:] = state["qvel"]
        if self.data.act.size:
            self.data.act[:] = state["act"]
        self.data.time = float(state["time"])
        if self.data.mocap_pos.size:
            self.data.mocap_pos[:] = state["mocap_pos"]
            self.data.mocap_quat[:] = state["mocap_quat"]
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self, endpoint_force_xyz=(0.0, 0.0, 0.0)) -> None:
        self.data.xfrc_applied[self.endpoint_body_id, :3] = np.asarray(
            endpoint_force_xyz, dtype=np.float64
        )
        mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.endpoint_body_id, :] = 0.0
