import numpy as np

from src.controller import EndpointServo
from src.reachability import compute_drive_protocol, required_success_translation


def test_required_success_translation_uses_rearmost_leading_vertex():
    leading_x = np.asarray([0.10, 0.09, 0.08, 0.07])
    assert required_success_translation(leading_x, 0.20) == 0.13


def test_compute_drive_protocol_adds_safety_and_extra_time():
    states = [
        {"leading_positions": np.column_stack((
            [0.10, 0.09, 0.08, 0.07], np.zeros(4), np.zeros(4)
        ))}
    ]
    protocol = compute_drive_protocol(states, 0.20, 0.03, 0.01, 2.0, 0.75)
    assert protocol["command_distance_m"] == 0.15
    assert protocol["max_duration_s"] > protocol["command_distance_m"] / 0.03


def test_endpoint_reference_never_exceeds_distance_cap():
    servo = EndpointServo(0.0, 0.0, 0.03, 40.0, 1.0, 2.0, 0.15)
    assert servo.reference_travel(4.0) == 0.12
    assert servo.reference_travel(5.0) == 0.15
    assert servo.reference_travel(100.0) == 0.15
