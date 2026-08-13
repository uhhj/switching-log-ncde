# Phase 0S-SOFA Stage C V2.3-R1 Collision Radius Audit

## Verdict

`PHASE0S_SOFA_STAGE_C_V2_3_INPUT_INVALID`

## Metrics

```json
{
  "stage": "Phase 0S-SOFA Stage C V2.3-R1",
  "cpu_only": true,
  "seed": 84000,
  "formal_c1_runs_requested": 1,
  "formal_c1_runs_completed": 0,
  "automatic_retry": false,
  "provenance": {
    "base": "60f542eaa1161c8491a40651c7b012ebd799c85b",
    "implementation_commit": "9a81d0a750464c6ce109635b358a7135b5ad6e82",
    "implementation_parent": "60f542eaa1161c8491a40651c7b012ebd799c85b",
    "implementation_parent_pass": true,
    "implementation_scope_paths": [
      "simulator-spikes/sofa-beam-spike/configs/stage_c_collision_radius_v2_3.json",
      "simulator-spikes/sofa-beam-spike/scripts/audit_stage_c_collision_radius_v2_3.py",
      "simulator-spikes/sofa-beam-spike/src/stage_c_collision_radius_v2_3.py",
      "simulator-spikes/sofa-beam-spike/src/stage_c_collision_radius_v2_3_scene.py",
      "simulator-spikes/sofa-beam-spike/tests/test_stage_c_collision_radius_v2_3.py"
    ],
    "implementation_scope_pass": false,
    "clean_formal_run_tree_pass": true,
    "frozen_blobs": {
      "simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2_2_fixture_extent.json": "09fbe116c9fff2e435cd343cce62ace32cb6b06b",
      "simulator-spikes/sofa-beam-spike/src/stage_c_stable_contact_v2_scene.py": "de903f45d17b70aa2222c0a598e4cb620b551b17",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_semantics.py": "01246a7c09d5f1a183e8f754d292a79eb9b168a0",
      "simulator-spikes/sofa-beam-spike/src/stage_c_contact_loss_morphology.py": "daa07e402d14bcb8101310e424969e8cd1ec8a1b",
      "simulator-spikes/sofa-beam-spike/native-contact-bridge/src/NativeContactBridge/NativeContactBridge.cpp": "4f844321d386c2dc7f58d9a826528bb17326e1ad",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_fixture_extent_v2_2_metrics.json": "5e2ec4bf260e03286ef861a38c1355dd25ab38e1",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_loss_temporal_order_v2_2l_metrics.json": "5f625ba463135b7bac2021d08a0061c03cfd4f9b"
    },
    "frozen_blobs_pass": false
  },
  "existing_evidence_guard": {
    "pass": true,
    "checked_paths": [
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/data/stage_c_v2_3_collision_radius_trace.json",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/stage_c_collision_radius_v2_3_metrics.json",
      "simulator-spikes/sofa-beam-spike/reports/phase0s_sofa/STAGE_C_COLLISION_RADIUS_V2_3.md"
    ]
  },
  "verdict": "PHASE0S_SOFA_STAGE_C_V2_3_INPUT_INVALID",
  "fact": "The single preregistered C1 trace was evaluated only through the first failing ordered gate.",
  "inference": "This result does not authorize any subsequent Stage-C or capability experiment.",
  "unknown": "No conclusion is drawn about later physical regimes.",
  "next_action": "Review this one V2.3-R1 result before any further simulator experiment."
}
```

## Fact

The single preregistered C1 trace was evaluated only through the first failing ordered gate.

## Inference

This result does not authorize any subsequent Stage-C or capability experiment.

## Unknown

No conclusion is drawn about later physical regimes.

## Next action

Review this one V2.3-R1 result before any further simulator experiment.
