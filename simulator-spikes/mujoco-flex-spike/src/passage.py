from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import mujoco
import numpy as np

from .contact import extract_contacts
from .simulator import FlexSimulator


FIXTURE_NAMES = (
    "upper_funnel",
    "lower_funnel",
    "upper_throat",
    "lower_throat",
)


@dataclass(frozen=True)
class PassageGeometry:
    radius_m: float
    entry_x_m: float
    funnel_exit_x_m: float
    exit_x_m: float
    entry_half_gap_m: float
    throat_half_gap_m: float
    wall_thickness_m: float
    wall_height_m: float
    pre_entry_distance_m: float
    exit_margin_m: float


def funnel_wall_pose(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    thickness: float,
) -> Tuple[np.ndarray, float, float]:
    del thickness
    midpoint = np.asarray([0.5 * (x0 + x1), 0.5 * (y0 + y1)])
    dx = float(x1 - x0)
    dy = float(y1 - y0)
    length = float(np.hypot(dx, dy))
    yaw = float(np.arctan2(dy, dx))
    return midpoint, length, yaw


def cable_radius(source_model: Path) -> float:
    model = mujoco.MjModel.from_xml_path(str(Path(source_model).resolve()))
    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "cable")
    return float(model.flex_radius[flex_id])


def geometry_from_config(
    radius_m: float, config: Mapping[str, Any]
) -> PassageGeometry:
    values = config["geometry"]
    funnel_length = float(values["funnel_length_r"]) * radius_m
    throat_length = float(values["throat_length_r"]) * radius_m
    return PassageGeometry(
        radius_m=radius_m,
        entry_x_m=0.0,
        funnel_exit_x_m=funnel_length,
        exit_x_m=funnel_length + throat_length,
        entry_half_gap_m=float(values["funnel_entry_half_gap_r"]) * radius_m,
        throat_half_gap_m=float(values["throat_half_gap_r"]) * radius_m,
        wall_thickness_m=float(values["wall_thickness_r"]) * radius_m,
        wall_height_m=float(values["wall_height_r"]) * radius_m,
        pre_entry_distance_m=float(values["pre_entry_distance_r"]) * radius_m,
        exit_margin_m=float(values["exit_margin_r"]) * radius_m,
    )


def _quat_z(yaw: float) -> str:
    return f"{np.cos(yaw / 2):.12g} 0 0 {np.sin(yaw / 2):.12g}"


def _funnel_geom(name: str, geometry: PassageGeometry, upper: bool) -> str:
    sign = 1.0 if upper else -1.0
    p0 = np.asarray(
        [geometry.entry_x_m, sign * geometry.entry_half_gap_m],
        dtype=np.float64,
    )
    p1 = np.asarray(
        [geometry.funnel_exit_x_m, sign * geometry.throat_half_gap_m],
        dtype=np.float64,
    )
    midpoint, length, yaw = funnel_wall_pose(
        p0[0], p1[0], p0[1], p1[1], geometry.wall_thickness_m
    )
    tangent = (p1 - p0) / length
    outward = sign * np.asarray([-tangent[1], tangent[0]])
    center = midpoint + 0.5 * geometry.wall_thickness_m * outward
    return (
        f'<geom name="{name}" type="box" '
        f'pos="{center[0]:.12g} {center[1]:.12g} {geometry.wall_height_m / 2:.12g}" '
        f'size="{length / 2:.12g} {geometry.wall_thickness_m / 2:.12g} {geometry.wall_height_m / 2:.12g}" '
        f'quat="{_quat_z(yaw)}" friction="0.60 0.005 0.0001"/>'
    )


def build_passage_xml(geometry: PassageGeometry) -> str:
    throat_center_x = 0.5 * (
        geometry.funnel_exit_x_m + geometry.exit_x_m
    )
    throat_half_length = 0.5 * (
        geometry.exit_x_m - geometry.funnel_exit_x_m
    )
    throat_center_y = geometry.throat_half_gap_m + 0.5 * geometry.wall_thickness_m
    upper_funnel = _funnel_geom("upper_funnel", geometry, True)
    lower_funnel = _funnel_geom("lower_funnel", geometry, False)
    return f"""<mujoco model="phase0m_constrained_passage">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" solver="Newton"
          iterations="100" noslip_iterations="10"/>
  <size memory="64M"/>
  <default>
    <geom friction="0.60 0.005 0.0001" solref="0.01 1"
          solimp="0.95 0.99 0.0001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 0" size="1 1 0.1"/>
    {upper_funnel}
    {lower_funnel}
    <geom name="upper_throat" type="box"
          pos="{throat_center_x:.12g} {throat_center_y:.12g} {geometry.wall_height_m / 2:.12g}"
          size="{throat_half_length:.12g} {geometry.wall_thickness_m / 2:.12g} {geometry.wall_height_m / 2:.12g}"
          friction="0.60 0.005 0.0001"/>
    <geom name="lower_throat" type="box"
          pos="{throat_center_x:.12g} {-throat_center_y:.12g} {geometry.wall_height_m / 2:.12g}"
          size="{throat_half_length:.12g} {geometry.wall_thickness_m / 2:.12g} {geometry.wall_height_m / 2:.12g}"
          friction="0.60 0.005 0.0001"/>
    <flexcomp name="cable" type="grid" dim="1" count="16 1 1"
              spacing="0.01 0.01 0.01" pos="-0.075 0 0.03"
              radius="{geometry.radius_m:.12g}" mass="0.16">
      <contact condim="3" friction="0.60 0.005 0.0001"
               solref="0.01 1" solimp="0.95 0.99 0.0001"
               selfcollide="none"/>
      <edge equality="true" damping="1"/>
    </flexcomp>
  </worldbody>
</mujoco>"""


class PassageScene:
    def __init__(self, source_model: Path, config: Mapping[str, Any]):
        radius = cable_radius(source_model)
        self.geometry = geometry_from_config(radius, config)
        self.simulator = FlexSimulator.from_xml_string(
            build_passage_xml(self.geometry)
        )
        self.fixture_ids = tuple(
            mujoco.mj_name2id(
                self.simulator.model, mujoco.mjtObj.mjOBJ_GEOM, name
            )
            for name in FIXTURE_NAMES
        )
        self.leading_indices = np.arange(
            self.simulator.vertex_body_ids.size - 1,
            self.simulator.vertex_body_ids.size - 5,
            -1,
            dtype=np.int64,
        )
        self.head_index = int(self.leading_indices[0])

    def reset_common(self, initial_y: float, gravity_z: float) -> None:
        self.simulator.reset(gravity_z=gravity_z, wall_enabled=False)
        vertices = self.simulator.vertex_positions()
        head = vertices[self.head_index]
        target_head = np.asarray(
            [
                self.geometry.entry_x_m - self.geometry.pre_entry_distance_m,
                float(initial_y),
                head[2],
            ]
        )
        self.simulator.translate_vertices(target_head - head)

    def fixture_contact(self, relative_velocity) -> Dict[str, Any]:
        return extract_contacts(
            self.simulator.model,
            self.simulator.data,
            self.simulator.flex_id,
            self.fixture_ids,
            relative_velocity,
        )

    def floor_contact(self, relative_velocity) -> Dict[str, Any]:
        return extract_contacts(
            self.simulator.model,
            self.simulator.data,
            self.simulator.flex_id,
            [self.simulator.floor_id],
            relative_velocity,
        )

    def leading_positions(self) -> np.ndarray:
        return self.simulator.vertex_positions()[self.leading_indices]

    def success(self) -> bool:
        threshold = self.geometry.exit_x_m + self.geometry.exit_margin_m
        return passage_success(self.leading_positions(), threshold)


def passage_success(leading_positions: Sequence[Sequence[float]], exit_x: float) -> bool:
    values = np.asarray(leading_positions, dtype=np.float64)
    if values.shape != (4, 3):
        raise ValueError("leading_positions must have shape (4, 3)")
    return bool(np.all(values[:, 0] >= float(exit_x)))


def reachability_clearance(
    geometry: PassageGeometry, branch_y_target: float
) -> float:
    return float(
        geometry.throat_half_gap_m
        - geometry.radius_m
        - abs(float(branch_y_target))
    )
