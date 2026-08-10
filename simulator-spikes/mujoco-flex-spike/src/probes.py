from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

import numpy as np
import yaml

from .contact import extract_contact
from .labels import derive_labels
from .simulator import FlexSimulator


PROBES = ("free", "stick", "slip", "jam")


def load_yaml(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"expected mapping in {path}")
    return value


def resolve_setup(config_path: Path) -> Tuple[Path, Dict[str, Any], Dict[str, Any]]:
    spike_root = Path(config_path).resolve().parent.parent
    config = load_yaml(Path(config_path).resolve())
    threshold_path = spike_root / str(config["labels"]["source_config"])
    threshold_config = load_yaml(threshold_path.resolve())
    return spike_root, config, dict(threshold_config["labels"])


def _force(config: Mapping[str, Any], probe: str) -> float:
    if probe == "stick":
        return float(config["probe"]["stick_force_n"])
    if probe == "slip":
        return float(config["probe"]["slip_force_n"])
    if probe == "jam":
        return float(config["probe"]["jam_force_n"])
    return 0.0


def run_episode(
    simulator: FlexSimulator,
    config: Mapping[str, Any],
    thresholds: Mapping[str, Any],
    probe: str,
) -> Dict[str, np.ndarray]:
    hz = float(config["simulation"]["hz"])
    gravity = 0.0 if probe == "free" else float(config["simulation"]["gravity_mps2"])
    simulator.reset(gravity_z=gravity, wall_enabled=probe == "jam")
    settle_steps = int(round(float(config["simulation"]["settle_s"]) * hz))
    for _ in range(settle_steps):
        simulator.step()
    origin, _ = simulator.endpoint_state()
    duration = (
        float(config["simulation"]["free_duration_s"])
        if probe == "free"
        else float(config["simulation"]["duration_s"])
    )
    steps = int(round(duration * hz))
    force_n = _force(config, probe)
    target_geom = simulator.wall_id if probe == "jam" else simulator.floor_id
    records = {
        key: []
        for key in (
            "time",
            "endpoint_position",
            "endpoint_velocity",
            "command",
            "command_force",
            "contact_force",
            "contact_count",
            "normal_force",
            "tangent_force",
            "tangent_velocity",
            "relative_velocity",
            "progress",
            "phase",
        )
    }
    for step in range(steps):
        simulator.step([force_n, 0.0, 0.0])
        position, velocity = simulator.endpoint_state()
        contact = extract_contact(
            simulator.model,
            simulator.data,
            simulator.flex_id,
            target_geom,
            velocity,
        )
        command = [1.0, 0.0, 0.0] if force_n > 0.0 else [0.0, 0.0, 0.0]
        records["time"].append((step + 1) / hz)
        records["endpoint_position"].append(position)
        records["endpoint_velocity"].append(velocity)
        records["command"].append(command)
        records["command_force"].append([force_n, 0.0, 0.0])
        records["contact_force"].append(
            [contact["normal_force"], contact["tangent_force"]]
        )
        records["contact_count"].append(contact["contact_count"])
        records["normal_force"].append(contact["normal_force"])
        records["tangent_force"].append(contact["tangent_force"])
        records["tangent_velocity"].append(contact["tangent_velocity"])
        records["relative_velocity"].append(contact["relative_velocity"])
        records["progress"].append(float(position[0] - origin[0]))
        records["phase"].append("free" if probe == "free" else "insertion")
    trace = {key: np.asarray(value) for key, value in records.items()}
    labels = derive_labels(trace, thresholds, hz)
    trace.update(
        {
            "contact_label": labels["contact_label"],
            "friction_label": labels["friction_label"],
            "failure_label": labels["failure_label"],
            "events": labels["events"],
            "event_names": labels["event_names"],
        }
    )
    return trace


def run_all(config_path: Path) -> int:
    spike_root, config, thresholds = resolve_setup(config_path)
    model_path = spike_root / str(config["simulation"]["model"])
    simulator = FlexSimulator(model_path)
    expected_timestep = 1.0 / float(config["simulation"]["hz"])
    if not np.isclose(simulator.timestep, expected_timestep):
        raise ValueError("model timestep and configured hz disagree")
    data_root = spike_root / str(config["outputs"]["report_root"]) / "data"
    data_root.mkdir(parents=True, exist_ok=True)
    count = 0
    for probe in PROBES:
        for repeat in range(1, int(config["experiment"]["repeats"]) + 1):
            trace = run_episode(simulator, config, thresholds, probe)
            np.savez_compressed(
                data_root / f"{probe}_repeat_{repeat}.npz", **trace
            )
            count += 1
            print(
                f"{probe} repeat {repeat}: "
                f"contacts={int(np.count_nonzero(trace['contact_count']))}"
            )
    return count
