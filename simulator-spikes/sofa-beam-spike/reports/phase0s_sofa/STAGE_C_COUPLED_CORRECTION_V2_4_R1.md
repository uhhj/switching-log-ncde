# Phase 0S-SOFA Stage C V2.4-R1 Semantic-Baseline Repair + Offline Re-Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_4_CONTACT_CONSTRUCTION_FAIL`

## Execution

Offline-only re-audit of the already committed V2.4 physical trace. No SOFA, Docker, GPU, trace regeneration or physics change.

## Source failure signature

```json
{
  "checks": {
    "source_verdict_is_semantics_fail": true,
    "formal_c1_completed": true,
    "source_instrumentation_pass": true,
    "source_response_stack_pass": true,
    "source_sphere_semantics_failed": true,
    "reconstruction_samples_positive": true,
    "reconstruction_error_zero": true,
    "actual_contact_matches_correct_point": true,
    "actual_alarm_matches_correct_point": true,
    "wrong_expected_contact_equals_sphere_global": true,
    "wrong_expected_alarm_equals_sphere_global": true,
    "correct_point_contact_is_0p002": true,
    "correct_point_alarm_is_0p010": true,
    "zero_not_evaluated": true,
    "barrier_not_evaluated": true
  },
  "wrong_point_baseline_used_by_source_audit": {
    "contact_m": 0.0,
    "alarm_m": 0.008
  },
  "correct_point_baseline": {
    "contact_m": 0.002,
    "alarm_m": 0.01
  },
  "actual_sphere_effective_envelope": {
    "contact_m": 0.002,
    "alarm_m": 0.01
  },
  "pass": true
}
```

## Corrected gates

```json
{
  "semantics": {
    "sphere_triangle": {
      "official_sofa_source": {
        "tag": "v26.06.00",
        "local_min_distance_cpp_blob": "156df23d1c813888ebc98ec321e26292d8abbf07",
        "sphere_collision_model_inl_blob": "f1c9370aba829aabd6d8024a58d15f3ea7c19cf6",
        "triangle_sphere_contact_formula": "global_contact + sphere_radius + sphere_model_contact + triangle_model_contact",
        "triangle_sphere_alarm_formula": "global_alarm + sphere_radius + sphere_model_contact + triangle_model_contact",
        "detection_value_formula": "euclidean_native_point_distance - effective_contact_distance"
      },
      "collision_snapshot": {
        "beam_collision_model": "SphereCollisionModel",
        "beam_default_radius_m": 0.002,
        "beam_list_radius_m": [
          0.002,
          0.002,
          0.002,
          0.002,
          0.002,
          0.002,
          0.002
        ],
        "beam_model_contact_distance_m": 0.0,
        "fixture_collision_model": "TriangleCollisionModel",
        "fixture_model_contact_distance_m": 0.0,
        "fixture_both_side": true,
        "fixture_triangle_indices": [
          [
            0,
            1,
            2
          ],
          [
            0,
            2,
            3
          ]
        ],
        "intersection_contact_distance_m": 0.0,
        "intersection_alarm_distance_m": 0.008,
        "correction_class": "LinearSolverConstraintCorrection",
        "correction_component_state": "Valid",
        "correction_mode": "linear_solver_constraint_correction",
        "correction_wire_optimization": false,
        "correction_regularization_term": 0.0,
        "linear_solver_class": "BTDLinearSolver",
        "ode_solver_class": "EulerImplicitSolver",
        "ode_rayleigh_stiffness": 0.05,
        "ode_rayleigh_mass": 0.05,
        "lcp_solver_class": "LCPConstraintSolver",
        "lcp_mu": 0.8,
        "lcp_tolerance": 1e-08,
        "lcp_max_it": 1000,
        "lcp_build_lcp": false,
        "lcp_compute_constraint_forces": true,
        "collision_response_class": "CollisionResponse",
        "collision_response": "FrictionContactConstraint",
        "collision_response_params": "mu=0.8"
      },
      "requested_beam_radius_m": 0.002,
      "radius_count": 7,
      "expected_radius_count": 7,
      "all_radius_readbacks_match_requested": true,
      "actual_effective_distances": {
        "effective_contact_distance_m": 0.002,
        "effective_alarm_distance_m": 0.01
      },
      "frozen_point_effective_contact_distance_m": 0.002,
      "frozen_point_effective_alarm_distance_m": 0.01,
      "reconstruction_tolerance_m": 1e-06,
      "detection_value_reconstruction_samples": 3000,
      "detection_value_reconstruction_max_abs_error_m": 0.0,
      "pass": true
    },
    "response_stack": {
      "finite_pass": true,
      "beam_collision_model": "SphereCollisionModel",
      "beam_default_radius_m": 0.002,
      "beam_list_radius_m": [
        0.002,
        0.002,
        0.002,
        0.002,
        0.002,
        0.002,
        0.002
      ],
      "beam_model_contact_distance_m": 0.0,
      "fixture_collision_model": "TriangleCollisionModel",
      "fixture_model_contact_distance_m": 0.0,
      "fixture_both_side": true,
      "fixture_triangle_indices": [
        [
          0,
          1,
          2
        ],
        [
          0,
          2,
          3
        ]
      ],
      "intersection_contact_distance_m": 0.0,
      "intersection_alarm_distance_m": 0.008,
      "correction_class": "LinearSolverConstraintCorrection",
      "correction_component_state": "Valid",
      "correction_mode": "linear_solver_constraint_correction",
      "correction_wire_optimization": false,
      "correction_regularization_term": 0.0,
      "linear_solver_class": "BTDLinearSolver",
      "ode_solver_class": "EulerImplicitSolver",
      "ode_rayleigh_stiffness": 0.05,
      "ode_rayleigh_mass": 0.05,
      "lcp_solver_class": "LCPConstraintSolver",
      "lcp_mu": 0.8,
      "lcp_tolerance": 1e-08,
      "lcp_max_it": 1000,
      "lcp_build_lcp": false,
      "lcp_compute_constraint_forces": true,
      "collision_response_class": "CollisionResponse",
      "collision_response": "FrictionContactConstraint",
      "collision_response_params": "mu=0.8",
      "correction_component_state_valid": true,
      "pass": true
    },
    "pass": true
  },
  "zero_load": {
    "samples": 50,
    "native_detection_frames": 50,
    "proximity_only_frames": 50,
    "geometric_contact_frames": 0,
    "reaction_active_frames": 0,
    "pass": true
  },
  "barrier": {
    "evaluated": true,
    "formal_geometric_onset_index": 61,
    "formal_geometric_onset_time_s": 0.1240000000000001,
    "scan_start_index": 0,
    "excluded_translation_fixed_node_ids": [
      0
    ],
    "mobile_node_ids": [
      1,
      2,
      3,
      4,
      5,
      6
    ],
    "crossing_events": [],
    "crossing_event_count": 0,
    "pass": true
  },
  "mobile_fixture_edge": {
    "evaluated": true,
    "formal_geometric_onset_index": 61,
    "formal_geometric_onset_time_s": 0.1240000000000001,
    "mobile_node_ids": [
      1,
      2,
      3,
      4,
      5,
      6
    ],
    "excluded_translation_fixed_node_ids": [
      0
    ],
    "ever_mobile_footprint_after_onset": true,
    "terminal_mobile_footprint_node_ids": [
      1,
      2,
      3,
      4,
      5,
      6
    ],
    "terminal_mobile_native_detection_ids": [
      1,
      2,
      3,
      4,
      5,
      6
    ],
    "fixture_edge_escape": false,
    "pass": true
  },
  "contact": {
    "measurement_samples": 350,
    "episode": {
      "occupancy": 0.4257142857142857,
      "episodes": 13,
      "longest_duration_s": 0.09400000000000008,
      "median_duration_s": 0.0040000000000000036,
      "episode_records": [
        {
          "start_index": 42,
          "end_index": 84,
          "start_s": 0.3860000000000003,
          "end_s": 0.47000000000000036,
          "samples": 43,
          "duration_s": 0.08400000000000007
        },
        {
          "start_index": 134,
          "end_index": 181,
          "start_s": 0.5700000000000004,
          "end_s": 0.6640000000000005,
          "samples": 48,
          "duration_s": 0.09400000000000008
        },
        {
          "start_index": 248,
          "end_index": 248,
          "start_s": 0.7980000000000006,
          "end_s": 0.7980000000000006,
          "samples": 1,
          "duration_s": 0.0
        },
        {
          "start_index": 253,
          "end_index": 254,
          "start_s": 0.8080000000000006,
          "end_s": 0.8100000000000006,
          "samples": 2,
          "duration_s": 0.0020000000000000018
        },
        {
          "start_index": 268,
          "end_index": 268,
          "start_s": 0.8380000000000006,
          "end_s": 0.8380000000000006,
          "samples": 1,
          "duration_s": 0.0
        },
        {
          "start_index": 280,
          "end_index": 282,
          "start_s": 0.8620000000000007,
          "end_s": 0.8660000000000007,
          "samples": 3,
          "duration_s": 0.0040000000000000036
        },
        {
          "start_index": 289,
          "end_index": 289,
          "start_s": 0.8800000000000007,
          "end_s": 0.8800000000000007,
          "samples": 1,
          "duration_s": 0.0
        },
        {
          "start_index": 294,
          "end_index": 296,
          "start_s": 0.8900000000000007,
          "end_s": 0.8940000000000007,
          "samples": 3,
          "duration_s": 0.0040000000000000036
        },
        {
          "start_index": 299,
          "end_index": 300,
          "start_s": 0.9000000000000007,
          "end_s": 0.9020000000000007,
          "samples": 2,
          "duration_s": 0.0020000000000000018
        },
        {
          "start_index": 302,
          "end_index": 309,
          "start_s": 0.9060000000000007,
          "end_s": 0.9200000000000007,
          "samples": 8,
          "duration_s": 0.014000000000000012
        },
        {
          "start_index": 311,
          "end_index": 327,
          "start_s": 0.9240000000000007,
          "end_s": 0.9560000000000007,
          "samples": 17,
          "duration_s": 0.03200000000000003
        },
        {
          "start_index": 329,
          "end_index": 341,
          "start_s": 0.9600000000000007,
          "end_s": 0.9840000000000008,
          "samples": 13,
          "duration_s": 0.02400000000000002
        },
        {
          "start_index": 343,
          "end_index": 349,
          "start_s": 0.9880000000000008,
          "end_s": 1.0000000000000007,
          "samples": 7,
          "duration_s": 0.0119999999999999
        }
      ]
    },
    "longest_episode": {
      "start_index": 134,
      "end_index": 181,
      "start_s": 0.5700000000000004,
      "end_s": 0.6640000000000005,
      "samples": 48,
      "duration_s": 0.09400000000000008
    },
    "required_occupancy": 0.9,
    "required_dwell_s": 0.3,
    "duration_semantics": "timestamp_span_end_minus_start",
    "time_tolerance_s": 1e-10,
    "pass": false
  },
  "reaction": {
    "evaluated": false,
    "reason": "contact gate failed"
  },
  "geometry": {
    "evaluated": false,
    "reason": "contact gate failed"
  }
}
```

## Fact

Barrier and fixture-edge gates pass, but the frozen V2.4 trace fails stable-contact occupancy/dwell.

## Inference

Coupled correction prevents cross-through but does not establish required sustained contact.

## Unknown

The experiment does not isolate off-diagonal coupling from other formulation differences; node-specific constraint-row force attribution remains unavailable.

## Next action

Review contact-loss morphology with V2.4 physics frozen.
