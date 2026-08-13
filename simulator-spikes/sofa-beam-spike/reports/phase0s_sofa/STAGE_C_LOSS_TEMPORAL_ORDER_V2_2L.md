# Phase 0S-SOFA Stage C V2.2-L Loss Temporal-Precedence Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_2L_CROSSING_PRECEDES_FIRST_PERMANENT_NODE_EXIT`

## Scope

Offline-only analysis of the immutable V2.2 trace. No SOFA, Docker, GPU, trace regeneration, physics change, cleanup, Oracle work, Stage D/E/F, capability, passage, matched-state audit, or training occurred. Temporal precedence is not causal proof.

## Per-node crossing → permanent-exit table

| node | crossing time (s) | permanent exit time (s) | exit-crossing (s) | boundary |
|---:|---:|---:|---:|:---|
| 1 | 0.2580000000000002 | 0.5620000000000004 | 0.3040000000000002 | x_min |
| 2 | 0.21800000000000017 | 0.5620000000000004 | 0.3440000000000002 | x_min |
| 3 | 0.19400000000000014 | 0.5620000000000004 | 0.3680000000000002 | x_min |
| 4 | 0.17400000000000013 | 0.5620000000000004 | 0.38800000000000023 | x_min |
| 5 | 0.1560000000000001 | 0.5620000000000004 | 0.40600000000000025 | x_min |
| 6 | 0.1440000000000001 | 0.5600000000000004 | 0.41600000000000026 | x_min |

## Decision metrics

```json
{
  "stage": "Phase 0S-SOFA Stage C V2.2-L",
  "analysis_type": "temporal_precedence",
  "formal_comparison": "first_crossing_vs_earliest_permanent_mobile_node_fixture_exit",
  "execution": "offline-only",
  "sofa_executed": false,
  "docker_executed": false,
  "gpu_used": false,
  "trace_regenerated": false,
  "physics_changed": false,
  "cleanup_executed": false,
  "stage_d_e_f_executed": false,
  "oracle_fitted": false,
  "oracle_frozen": false,
  "capability_executed": false,
  "passage_executed": false,
  "matched_state_audit_executed": false,
  "training_executed": false,
  "input_gate": {
    "head": "3d844aa0cdde64c7a4427bcbc4280f0f84dac21d",
    "required_head": "3d844aa0cdde64c7a4427bcbc4280f0f84dac21d",
    "branch": "phase0s-sofa-beamadapter",
    "required_branch": "phase0s-sofa-beamadapter",
    "blob_hashes": {
      "simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2_2_fixture_extent.json": "0e7f1bc9a4b82defc602701bdeb14022acc4e543",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_semantics.py": "a130d83e11ba4702c44b94344f766f44df6abbc7",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py": "fc45ae04dbc4e27072cf5301b848ae661014e4ee",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/data/stage_c_v2_2_fixture_extent_trace.json": "ef9289179126509d4312884ab7540eded1b8890a",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_fixture_extent_v2_2_metrics.json": "7c63358435c7108226b5f2eca8a585775cc19388"
    },
    "expected_blob_hashes": {
      "simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2_2_fixture_extent.json": "0e7f1bc9a4b82defc602701bdeb14022acc4e543",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_semantics.py": "a130d83e11ba4702c44b94344f766f44df6abbc7",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py": "fc45ae04dbc4e27072cf5301b848ae661014e4ee",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/data/stage_c_v2_2_fixture_extent_trace.json": "ef9289179126509d4312884ab7540eded1b8890a",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_fixture_extent_v2_2_metrics.json": "7c63358435c7108226b5f2eca8a585775cc19388"
    },
    "pass": true
  },
  "frozen_v2_2_verdict": "PHASE0S_SOFA_STAGE_C_V2_2_FIXTURE_EXTENT_NOT_CORRECTED",
  "frozen_crossing_event_count": 6,
  "frozen_terminal_mobile_edge_escape": true,
  "effective_contact_distance_m": 0.002,
  "reaction_floor_c0_p99": 0.0,
  "temporal_order": {
    "verdict": "PHASE0S_SOFA_STAGE_C_V2_2L_CROSSING_PRECEDES_FIRST_PERMANENT_NODE_EXIT",
    "reason": "the first completed in-footprint wall-plane crossing precedes the earliest permanent mobile-node fixture exit",
    "formal_comparison": "first_completed_in_footprint_crossing_vs_earliest_permanent_mobile_node_fixture_exit",
    "geometric_first_index": 61,
    "geometric_first_time_s": 0.1240000000000001,
    "geometric_last_index": 145,
    "geometric_last_time_s": 0.2920000000000002,
    "crossing_consistency": {
      "pass": true,
      "failures": [],
      "reconstructed_count": 6,
      "frozen_count": 6
    },
    "crossing_events": [
      {
        "node_id": 6,
        "from_record_index": 70,
        "to_record_index": 71,
        "from_time_s": 0.1420000000000001,
        "to_time_s": 0.1440000000000001,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 1.2301278259588998e-05,
        "to_signed_plane_distance_m": -0.00020900080685008087
      },
      {
        "node_id": 5,
        "from_record_index": 76,
        "to_record_index": 77,
        "from_time_s": 0.1540000000000001,
        "to_time_s": 0.1560000000000001,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 0.00016095192374247246,
        "to_signed_plane_distance_m": -2.4276536499422825e-05
      },
      {
        "node_id": 4,
        "from_record_index": 85,
        "to_record_index": 86,
        "from_time_s": 0.17200000000000013,
        "to_time_s": 0.17400000000000013,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 0.00013136378118970446,
        "to_signed_plane_distance_m": -3.9017465274988163e-05
      },
      {
        "node_id": 3,
        "from_record_index": 95,
        "to_record_index": 96,
        "from_time_s": 0.19200000000000014,
        "to_time_s": 0.19400000000000014,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 0.0001388965471950745,
        "to_signed_plane_distance_m": -1.8495175081744595e-05
      },
      {
        "node_id": 2,
        "from_record_index": 107,
        "to_record_index": 108,
        "from_time_s": 0.21600000000000016,
        "to_time_s": 0.21800000000000017,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 7.798459234670657e-05,
        "to_signed_plane_distance_m": -5.913359911552571e-05
      },
      {
        "node_id": 1,
        "from_record_index": 127,
        "to_record_index": 128,
        "from_time_s": 0.25600000000000017,
        "to_time_s": 0.2580000000000002,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 7.691071045831292e-06,
        "to_signed_plane_distance_m": -8.825787638322558e-05
      }
    ],
    "crossing_event_count": 6,
    "first_crossing": {
      "node_id": 6,
      "from_record_index": 70,
      "to_record_index": 71,
      "from_time_s": 0.1420000000000001,
      "to_time_s": 0.1440000000000001,
      "transition_duration_s": 0.0020000000000000018,
      "from_signed_plane_distance_m": 1.2301278259588998e-05,
      "to_signed_plane_distance_m": -0.00020900080685008087
    },
    "first_crossing_time_s": 0.1440000000000001,
    "first_permanent_node_exit": {
      "node_id": 6,
      "status": "permanently_outside_fixture_footprint",
      "last_inside_index": 278,
      "last_inside_time_s": 0.5580000000000004,
      "permanent_exit_index": 279,
      "permanent_exit_time_s": 0.5600000000000004,
      "boundary": [
        "x_min"
      ]
    },
    "first_permanent_node_exit_time_s": 0.5600000000000004,
    "first_exit_minus_first_crossing_s": 0.41600000000000026,
    "per_node_crossing_exit_table": [
      {
        "node_id": 1,
        "crossing_time_s": 0.2580000000000002,
        "permanent_exit_time_s": 0.5620000000000004,
        "exit_minus_crossing_s": 0.3040000000000002,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      },
      {
        "node_id": 2,
        "crossing_time_s": 0.21800000000000017,
        "permanent_exit_time_s": 0.5620000000000004,
        "exit_minus_crossing_s": 0.3440000000000002,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      },
      {
        "node_id": 3,
        "crossing_time_s": 0.19400000000000014,
        "permanent_exit_time_s": 0.5620000000000004,
        "exit_minus_crossing_s": 0.3680000000000002,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      },
      {
        "node_id": 4,
        "crossing_time_s": 0.17400000000000013,
        "permanent_exit_time_s": 0.5620000000000004,
        "exit_minus_crossing_s": 0.38800000000000023,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      },
      {
        "node_id": 5,
        "crossing_time_s": 0.1560000000000001,
        "permanent_exit_time_s": 0.5620000000000004,
        "exit_minus_crossing_s": 0.40600000000000025,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      },
      {
        "node_id": 6,
        "crossing_time_s": 0.1440000000000001,
        "permanent_exit_time_s": 0.5600000000000004,
        "exit_minus_crossing_s": 0.41600000000000026,
        "exit_boundary": [
          "x_min"
        ],
        "exit_status": "permanently_outside_fixture_footprint"
      }
    ],
    "per_node_permanent_fixture_exits": [
      {
        "node_id": 1,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 279,
        "last_inside_time_s": 0.5600000000000004,
        "permanent_exit_index": 280,
        "permanent_exit_time_s": 0.5620000000000004,
        "boundary": [
          "x_min"
        ]
      },
      {
        "node_id": 2,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 279,
        "last_inside_time_s": 0.5600000000000004,
        "permanent_exit_index": 280,
        "permanent_exit_time_s": 0.5620000000000004,
        "boundary": [
          "x_min"
        ]
      },
      {
        "node_id": 3,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 279,
        "last_inside_time_s": 0.5600000000000004,
        "permanent_exit_index": 280,
        "permanent_exit_time_s": 0.5620000000000004,
        "boundary": [
          "x_min"
        ]
      },
      {
        "node_id": 4,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 279,
        "last_inside_time_s": 0.5600000000000004,
        "permanent_exit_index": 280,
        "permanent_exit_time_s": 0.5620000000000004,
        "boundary": [
          "x_min"
        ]
      },
      {
        "node_id": 5,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 279,
        "last_inside_time_s": 0.5600000000000004,
        "permanent_exit_index": 280,
        "permanent_exit_time_s": 0.5620000000000004,
        "boundary": [
          "x_min"
        ]
      },
      {
        "node_id": 6,
        "status": "permanently_outside_fixture_footprint",
        "last_inside_index": 278,
        "last_inside_time_s": 0.5580000000000004,
        "permanent_exit_index": 279,
        "permanent_exit_time_s": 0.5600000000000004,
        "boundary": [
          "x_min"
        ]
      }
    ],
    "final_all_mobile_permanent_edge_loss_onset_index_diagnostic": 280,
    "final_all_mobile_permanent_edge_loss_onset_time_s_diagnostic": 0.5620000000000004,
    "terminal_mobile_footprint_node_ids": [],
    "terminal_mobile_native_detection_ids": [],
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
    "time_tolerance_s": 1e-10,
    "offline_geometry_is_formal_contact_signal": false,
    "temporal_precedence_is_causal_proof": false
  },
  "verdict": "PHASE0S_SOFA_STAGE_C_V2_2L_CROSSING_PRECEDES_FIRST_PERMANENT_NODE_EXIT",
  "fact": "The first frozen-consistent completed in-footprint wall-plane crossing occurs at 0.144 s, before the earliest permanent mobile-node fixture exit at 0.56 s.",
  "inference": "Cross-through is temporally established before fixture escape begins at the per-node level in the immutable V2.2 trajectory.",
  "unknown": "Temporal precedence does not prove that crossing causes fixture escape.",
  "next_action": "Review this evidence, then verify official SOFA v26.06 radius/contactDistance composition before authorizing exactly one collision-representation correction."
}
```

## Fact

The first frozen-consistent completed in-footprint wall-plane crossing occurs at 0.144 s, before the earliest permanent mobile-node fixture exit at 0.56 s.

## Inference

Cross-through is temporally established before fixture escape begins at the per-node level in the immutable V2.2 trajectory.

## Unknown

Temporal precedence does not prove that crossing causes fixture escape.

## Next action

Review this evidence, then verify official SOFA v26.06 radius/contactDistance composition before authorizing exactly one collision-representation correction.
