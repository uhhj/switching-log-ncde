# Phase 0S-SOFA Stage C V2.4 Coupled Constraint Correction

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_4_COUPLED_CORRECTION_SEMANTICS_FAIL`

## Metrics

```json
{
  "stage": "Phase 0S-SOFA Stage C V2.4",
  "cpu_only": true,
  "formal_c1_requested": 1,
  "formal_c1_completed": 1,
  "automatic_retry": false,
  "intervention": {
    "diff_paths": [
      "constraint_correction"
    ],
    "expected_diff_paths": [
      "constraint_correction"
    ],
    "mode": "linear_solver_constraint_correction",
    "wire_optimization": false,
    "regularization_term": 0.0,
    "pass": true
  },
  "reaction_floor_c0_p99": 0.0,
  "source_v2_3l_verdict": "PHASE0S_SOFA_STAGE_C_V2_3L_LOCAL_CONTACT_PERSISTS_GLOBAL_RESPONSE_ACTIVE_AT_FIRST_CROSSING",
  "instrumentation": {
    "records": 500,
    "expected_records": 500,
    "finite_timing_force": true,
    "material_pass": true,
    "bridge_ready_all": true,
    "serials_fresh": true,
    "timing_force": {
      "dt_s": 0.002,
      "endpoint_index": 6,
      "max_step_duration_error_s": 1.0928757898653885e-16,
      "max_step_continuity_error_s": 0.0,
      "schedule_pass": true,
      "target_index_pass": true,
      "max_force_readback_error_n": 0.0,
      "pass": true
    },
    "tail_translation_max_m": 0.0,
    "tail_anchor_pass": true,
    "pass": true
  },
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
      "frozen_point_effective_contact_distance_m": 0.0,
      "frozen_point_effective_alarm_distance_m": 0.008,
      "reconstruction_tolerance_m": 1e-06,
      "detection_value_reconstruction_samples": 3000,
      "detection_value_reconstruction_max_abs_error_m": 0.0,
      "pass": false
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
    "pass": false
  },
  "verdict": "PHASE0S_SOFA_STAGE_C_V2_4_COUPLED_CORRECTION_SEMANTICS_FAIL",
  "zero_load": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "barrier": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "mobile_fixture_edge": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "contact": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "reaction": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "geometry": {
    "evaluated": false,
    "reason": "semantics failed"
  },
  "fact": "Coupled correction or frozen response-stack semantics failed.",
  "inference": "The C1 cannot be attributed to LinearSolverConstraintCorrection with the frozen solver/contact stack.",
  "unknown": "All gates after the first failure remain unqualified.",
  "next_action": "Review the first failed V2.4 gate before authorizing another bounded intervention."
}
```

## Fact

Coupled correction or frozen response-stack semantics failed.

## Inference

The C1 cannot be attributed to LinearSolverConstraintCorrection with the frozen solver/contact stack.

## Unknown

All gates after the first failure remain unqualified.

## Next action

Review the first failed V2.4 gate before authorizing another bounded intervention.
