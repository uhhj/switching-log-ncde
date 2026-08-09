import numpy as np

from slncde.phase0b.labels import compress_transitions, derive_labels


CONFIG = {
    "simulator": {"hz": 240, "trace_stride": 4},
    "labels": {
        "contact_normal_force_min_n": 0.05,
        "stick_tangent_speed_max_mps": 0.002,
        "slip_tangent_speed_min_mps": 0.005,
        "command_active_speed_min_mps": 0.005,
        "jam_normal_force_min_n": 0.50,
        "jam_window_ms": 100,
        "jam_progress_max_m": 0.0005,
        "minimum_mode_duration_ms": 80,
    },
}


def synthetic_trace():
    count = 32
    active = np.zeros(count, dtype=np.int64)
    force = np.zeros(count, dtype=np.float64)
    tangent_speed = np.zeros(count, dtype=np.float64)
    command = np.zeros((count, 3), dtype=np.float64)
    command[:, 0] = 0.01
    progress = np.linspace(0.0, 0.018, count)

    active[5:15] = 1
    force[5:15] = 0.2
    tangent_speed[5:10] = 0.006
    tangent_speed[10:15] = 0.001

    active[18:] = 1
    force[18:] = 1.0
    tangent_speed[18:] = 0.001
    progress[18:] = progress[18]
    return {
        "contact_active_beads": active,
        "normal_force_sum": force,
        "tangential_speed_mean": tangent_speed,
        "command_velocity_xyz": command,
        "progress_m": progress,
        "phase": np.full(count, "insertion"),
    }


def test_hierarchical_labels_and_events():
    labels = derive_labels(synthetic_trace(), CONFIG)
    assert np.all(labels["contact_state"][:5] == "free")
    assert labels["is_touch_event"][5]
    assert np.count_nonzero(labels["friction_regime"] == "slip") >= 5
    assert np.count_nonzero(labels["friction_regime"] == "stick") >= 5
    assert labels["is_release_event"][15]
    assert np.count_nonzero(labels["jam_state"] == "jam") >= 5
    assert np.count_nonzero(labels["is_jam_onset_event"]) == 1
