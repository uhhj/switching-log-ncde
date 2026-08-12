# Phase 0S-SOFA Stage C V2.1-L Contact-Loss Morphology Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_LOSS_FIXTURE_EDGE_ESCAPE`

## Scope

Offline-only adjudication of the immutable Stage-C V2 C1 trace. No SOFA, Docker, GPU, trace regeneration, physics change, Oracle fitting, or later-stage execution occurred.

## Metrics

~~~json
{
  "stage": "Phase 0S-SOFA Stage C V2.1-L",
  "offline_only": true,
  "sofa_executed": false,
  "docker_executed": false,
  "gpu_used": false,
  "trace_regenerated": false,
  "physics_changed": false,
  "config_changed": false,
  "scene_changed": false,
  "native_bridge_changed": false,
  "stage_d_executed": false,
  "stages_e_f_executed": false,
  "oracle_fitted": false,
  "oracle_frozen": false,
  "model_training_executed": false,
  "offline_geometry_surrogate_is_formal_contact_signal": false,
  "input_provenance": {
    "base_head": "69f6bf95152d8f89678033be7d13515cafc6a39b",
    "implementation_head": "d3909d3533cba530a80fe6131540492f98891171",
    "implementation_parent": "69f6bf95152d8f89678033be7d13515cafc6a39b",
    "implementation_parent_matches_base": true,
    "branch": "phase0s-sofa-beamadapter",
    "required_branch": "phase0s-sofa-beamadapter",
    "working_tree_status_at_audit_start": [],
    "working_tree_clean_at_audit_start": true,
    "implementation_changed_files": [
      "simulator-spikes/sofa-beam-spike/scripts/audit_stage_c_contact_loss_morphology_v2_1l.py",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py",
      "simulator-spikes/sofa-beam-spike/tests/test_stage_c_contact_loss_morphology_v2_1l.py"
    ],
    "required_implementation_files": [
      "simulator-spikes/sofa-beam-spike/scripts/audit_stage_c_contact_loss_morphology_v2_1l.py",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py",
      "simulator-spikes/sofa-beam-spike/tests/test_stage_c_contact_loss_morphology_v2_1l.py"
    ],
    "implementation_scope_exact": true,
    "blob_hashes": {
      "simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2.json": "1aaab0991983b65c2acf7613b173c5920615e86c",
      "simulator-spikes/sofa-beam-spike/src/stage_c_stable_contact_v2_scene.py": "96df7abc3a72bf2221bf5393424203d432ac0c82",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_semantics.py": "a130d83e11ba4702c44b94344f766f44df6abbc7",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/data/stage_c_v2_stable_contact_trace.json": "e1aa1477d5fe70b92a441a55a2225d85a016825e",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_stable_contact_v2_metrics.json": "9f89484d2b862793b2d12790ddde2d80ccef1fe6",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_contact_semantics_v2_1_metrics.json": "f4bac4d185b820fb2ec70900f82312013f748fc9"
    },
    "expected_blob_hashes": {
      "simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2.json": "1aaab0991983b65c2acf7613b173c5920615e86c",
      "simulator-spikes/sofa-beam-spike/src/stage_c_stable_contact_v2_scene.py": "96df7abc3a72bf2221bf5393424203d432ac0c82",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_semantics.py": "a130d83e11ba4702c44b94344f766f44df6abbc7",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/data/stage_c_v2_stable_contact_trace.json": "e1aa1477d5fe70b92a441a55a2225d85a016825e",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_stable_contact_v2_metrics.json": "9f89484d2b862793b2d12790ddde2d80ccef1fe6",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_contact_semantics_v2_1_metrics.json": "f4bac4d185b820fb2ec70900f82312013f748fc9"
    },
    "trace_sha256": "9e4b8680978286cb020aa52a64af1376c748394c39d34d19062929316c294df3",
    "expected_trace_sha256": "9e4b8680978286cb020aa52a64af1376c748394c39d34d19062929316c294df3",
    "pass": true
  },
  "frozen_stage_c_v2_1_verdict": "PHASE0S_SOFA_STAGE_C_CONTACT_CONSTRUCTION_FAIL",
  "trace_integrity": {
    "records": 500,
    "expected_records": 500,
    "dt_s": 0.002,
    "first_end_time_s": 0.002,
    "last_end_time_s": 1.0000000000000007,
    "measurement_frames": 350,
    "node_count": 7,
    "pass": true
  },
  "effective_contact_distance_m": 0.002,
  "reaction_floor_c0_p99": 0.0,
  "formal_measurement_summary": {
    "frames": 350,
    "native_detection_frames": 0,
    "geometric_contact_frames": 0,
    "load_bearing_contact_frames": 0
  },
  "morphology": {
    "verdict": "PHASE0S_SOFA_STAGE_C_LOSS_FIXTURE_EDGE_ESCAPE",
    "selected_mechanism": "fixture_edge_escape",
    "formal_onsets": {
      "diagnostic_onset_index": 61,
      "diagnostic_onset_source": "geometric_contact",
      "geometric_first_index": 61,
      "geometric_first_time_s": 0.1240000000000001,
      "geometric_last_index": 86,
      "geometric_last_time_s": 0.17400000000000013,
      "load_bearing_first_index": 61,
      "load_bearing_first_time_s": 0.1240000000000001,
      "load_bearing_last_index": 85,
      "load_bearing_last_time_s": 0.17200000000000013
    },
    "terminal": {
      "time_s": 1.0000000000000007,
      "native_detection": false,
      "footprint_node_ids": [],
      "footprint_node_count": 0,
      "minimum_abs_plane_distance_m": null,
      "surrogate_gap_m": null,
      "within_effective_envelope": false,
      "all_positive_outside_envelope": false,
      "any_negative_outside_envelope": false,
      "all_negative_outside_envelope": false,
      "minimum_signed_plane_distance_m": null,
      "maximum_signed_plane_distance_m": null
    },
    "crossing_scan_start_index": 60,
    "crossing_scan_start_time_s": 0.1220000000000001,
    "crossing_events": [
      {
        "node_id": 6,
        "from_record_index": 70,
        "to_record_index": 71,
        "from_time_s": 0.1420000000000001,
        "to_time_s": 0.1440000000000001,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 1.2301278259588566e-05,
        "to_signed_plane_distance_m": -0.00020900080685008123
      },
      {
        "node_id": 5,
        "from_record_index": 76,
        "to_record_index": 77,
        "from_time_s": 0.1540000000000001,
        "to_time_s": 0.1560000000000001,
        "transition_duration_s": 0.0020000000000000018,
        "from_signed_plane_distance_m": 0.00015983377672540598,
        "to_signed_plane_distance_m": -2.6241955294856662e-05
      }
    ],
    "crossing_event_count": 2,
    "weak_mechanism_anchors": {
      "migration_terminal_suffix_start_index": null,
      "migration_terminal_suffix_start_time_s": null,
      "migration_terminal_suffix_end_index": null,
      "migration_terminal_suffix_end_time_s": null,
      "reaction_final_geometric_suffix_start_index": 86,
      "reaction_final_geometric_suffix_start_time_s": 0.17400000000000013,
      "reaction_final_geometric_suffix_end_index": 86,
      "reaction_final_geometric_suffix_end_time_s": 0.17400000000000013
    },
    "final_loss_no_native_near_wall_terminal_run_steps": 0,
    "final_loss_no_native_near_wall_terminal_duration_s": 0.0,
    "final_geometric_without_reaction_terminal_run_steps": 1,
    "final_geometric_without_reaction_terminal_duration_s": 0.0,
    "weak_mechanism_duration_semantics": "timestamp_span_end_minus_start",
    "weak_minimum_dwell_s": 0.3,
    "frozen_max_native_lcp_onset_lag_steps_diagnostic_only": 2,
    "candidate_flags": {
      "cross_through": false,
      "rebound_escape": false,
      "fixture_edge_escape": true,
      "contact_migration": false,
      "reaction_collapse": false
    },
    "trajectory_summary": {
      "frames": 500,
      "footprint_node_count_min": 0,
      "footprint_node_count_max": 3,
      "surrogate_gap_min_m": -0.0019876987217404113,
      "surrogate_gap_max_m": 0.08419739928889482,
      "minimum_signed_plane_distance_min_m": -0.08619739928889482,
      "maximum_signed_plane_distance_max_m": 0.0030000125367053545
    },
    "offline_geometry_surrogate_is_formal_contact_signal": false
  },
  "verdict": "PHASE0S_SOFA_STAGE_C_LOSS_FIXTURE_EDGE_ESCAPE",
  "fact": "After formal contact onset, all beam nodes leave the finite fixture x/z footprint before the terminal frame.",
  "inference": "The finite fixture footprint is the dominant Stage-C contact-loss mechanism.",
  "unknown": "Reaction coupling and all later mode/capability questions remain unqualified.",
  "next_action": "STOP. Design one bounded fixture-extent correction only; keep all other frozen physics unchanged."
}
~~~

## Fact

After formal contact onset, all beam nodes leave the finite fixture x/z footprint before the terminal frame.

## Inference

The finite fixture footprint is the dominant Stage-C contact-loss mechanism.

## Unknown

Reaction coupling and all later mode/capability questions remain unqualified.

## Next action

STOP. Design one bounded fixture-extent correction only; keep all other frozen physics unchanged.
