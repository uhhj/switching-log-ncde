from pathlib import Path

import numpy as np

from src.contact import extract_contact
from src.labels import derive_labels
from src.simulator import FlexSimulator


ROOT = Path(__file__).resolve().parents[1]
HZ = 500.0
THRESHOLDS = {
    "contact_normal_force_min_n": 0.05,
    "stick_tangent_speed_max_mps": 0.002,
    "slip_tangent_speed_min_mps": 0.005,
    "command_active_speed_min_mps": 0.005,
    "jam_normal_force_min_n": 0.50,
    "jam_window_ms": 100,
    "jam_progress_max_m": 0.0005,
    "minimum_mode_duration_ms": 80,
}


def _trace(length=100, contact=True, tangent=0.0, progress=None, command=True):
    if progress is None:
        progress = np.zeros(length)
    return {
        "contact_count": np.full(length, 1 if contact else 0),
        "normal_force": np.full(length, 1.0 if contact else 0.0),
        "tangent_velocity": np.full(length, tangent),
        "command": np.tile([1.0, 0.0, 0.0] if command else [0.0, 0.0, 0.0], (length, 1)),
        "progress": np.asarray(progress),
        "phase": np.full(length, "insertion"),
    }


def test_contact_extraction():
    simulator = FlexSimulator(ROOT / "models" / "cable.xml")
    simulator.reset(gravity_z=-9.81, wall_enabled=False)
    for _ in range(250):
        simulator.step()
    _, velocity = simulator.endpoint_state()
    contact = extract_contact(
        simulator.model,
        simulator.data,
        simulator.flex_id,
        simulator.floor_id,
        velocity,
    )
    assert contact["contact_count"] > 0
    assert contact["normal_force"] > 0.0
    assert contact["relative_velocity"].shape == (3,)


def test_free_label():
    labels = derive_labels(_trace(contact=False, command=False), THRESHOLDS, HZ)
    assert np.all(labels["contact_label"] == "free")


def test_stick_label():
    labels = derive_labels(_trace(tangent=0.0001), THRESHOLDS, HZ)
    assert np.all(labels["stick"])
    assert np.all(labels["friction_label"] == "stick")


def test_slip_label():
    labels = derive_labels(_trace(tangent=0.01), THRESHOLDS, HZ)
    assert np.all(labels["slip"])
    assert np.all(labels["friction_label"] == "slip")


def test_jam_label():
    labels = derive_labels(_trace(progress=np.zeros(100)), THRESHOLDS, HZ)
    assert np.count_nonzero(labels["jam"]) >= 40
    assert "jam" in labels["failure_label"]
