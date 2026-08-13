#!/usr/bin/env python3
"""Offline Stage-C V2.3-L crossing-local/global-response audit."""
from __future__ import annotations
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];SPIKE=ROOT/'simulator-spikes'/'sofa-beam-spike';sys.path.insert(0,str(SPIKE/'src'))
from stage_c_crossing_local_audit import V_LOCAL_CONTACT_GLOBAL_ACTIVE,V_LOCAL_CONTACT_GLOBAL_INACTIVE,V_LOCAL_CONTACT_ABSENT,audit_crossings
CONFIG=SPIKE/'configs'/'stage_c_collision_radius_v2_3.json';SOURCE_METRICS=SPIKE/'reports'/'phase0s_sofa'/'stage_c_collision_radius_v2_3_r2_metrics.json';SOURCE_TRACE=SPIKE/'reports'/'phase0s_sofa'/'data'/'stage_c_v2_3_r2_collision_radius_trace.json';OUT_METRICS=SPIKE/'reports'/'phase0s_sofa'/'stage_c_v2_3_crossing_local_metrics.json';OUT_REPORT=SPIKE/'reports'/'phase0s_sofa'/'STAGE_C_V2_3_CROSSING_LOCAL_AUDIT.md';EXPECTED_SOURCE_VERDICT='PHASE0S_SOFA_STAGE_C_V2_3_BARRIER_CROSSING_FAIL';EXPECTED_CROSSING_COUNT=6
def narrative(r):
 v=r['verdict'];flip=r['primary_event']['normal_y_sign_flip_across_crossing']
 if v==V_LOCAL_CONTACT_GLOBAL_ACTIVE:inf='Detection geometry alone is no longer the leading explanation: node-local formal contact still exists while global response remains active at the first barrier violation.';next='Review the frozen first-crossing local-contact/global-response evidence together with the official SOFA barrier/contact-response semantics, then authorize exactly one bounded barrier-topology or response-formulation intervention while keeping Sphere radius, effective envelope, load, material and dt frozen.'
 elif v==V_LOCAL_CONTACT_GLOBAL_INACTIVE:inf='The leading next target is response activation/coupling, because node-local formal contact persists but the global response is not fully active at the first barrier violation.';next='Inspect and repair exactly one contact-response activation/coupling path with Sphere geometry and task physics frozen.'
 elif v==V_LOCAL_CONTACT_ABSENT:inf='The leading next target is contact persistence/detection continuity, because node-local formal geometric contact is absent at crossing completion.';next='Design exactly one bounded contact-persistence/detection intervention with solver response, load, fixture, material and dt otherwise frozen.'
 else:raise ValueError(v)
 normal='unknown: one or both crossing-boundary frames lack node-local geometric normal_y'
 if flip is True:normal='beam-outward normal_y changes sign across the first crossing, consistent with the frozen Triangle/Sphere normal convention as the sphere center changes wall side. This is descriptive only and does not discriminate barrier topology from response enforcement.'
 elif flip is False:normal='beam-outward normal_y does not change sign across the first crossing. Inspect the local multi-contact geometry and normal records; this diagnostic does not alter the formal verdict or choose the next physics intervention.'
 return {'fact':r['reason'],'inference':inf,'normal_diagnostic':normal,'unknown':'Global constraint rows and reaction are not node-mapped; the trace cannot prove the individual crossing node\'s constraint/reaction contribution.','next_action':next}
def fmt(x,digits=6):return '—' if x is None else f'{float(x):.{digits}f}'
def table(rows):
 lines=['| node | crossing s | from y | to y | local geo | normal_y from | normal_y to | flip | global LCP | global reaction | min DetectionOutput | last local geo s | gap s |','|---:|---:|---:|---:|:---:|---:|---:|:---:|:---:|:---:|---:|---:|---:|']
 for r in rows:
  minimum='—' if r['local_min_detection_value_m'] is None else f"{float(r['local_min_detection_value_m']):.9g}"
  lines.append(f"| {r['node_id']} | {r['crossing_time_s']:.6f} | {r['from_signed_center_y_m']:.9g} | {r['completion_signed_center_y_m']:.9g} | {r['local_geometric_at_crossing']} | {fmt(r['from_geometric_normal_y_median'])} | {fmt(r['completion_geometric_normal_y_median'])} | {r['normal_y_sign_flip_across_crossing']} | {r['global_lcp_at_crossing']} | {r['global_reaction_active_at_crossing']} | {minimum} | {fmt(r['last_local_geometric_time_s'])} | {fmt(r['crossing_minus_last_geometric_s'])} |")
 return '\n'.join(lines)
def main():
 if OUT_METRICS.exists() or OUT_REPORT.exists():print('V2.3-L evidence already exists; refusing to overwrite.',file=sys.stderr);return 2
 c=json.loads(CONFIG.read_text(encoding='utf-8'));m=json.loads(SOURCE_METRICS.read_text(encoding='utf-8'));t=json.loads(SOURCE_TRACE.read_text(encoding='utf-8'))
 if m['verdict']!=EXPECTED_SOURCE_VERDICT:raise RuntimeError('source V2.3-R2 verdict is not frozen barrier-crossing failure')
 b=m['barrier']
 if int(b['crossing_event_count'])!=EXPECTED_CROSSING_COUNT:raise RuntimeError('source V2.3-R2 crossing count changed')
 r=audit_crossings(t['records'],b['crossing_events'],wall_y=float(c['contact_wall']['plane_y_m']),reaction_floor=float(m['reaction_floor_c0_p99']),time_tolerance_s=float(c['integrity']['time_tolerance_s']),distance_tolerance_m=float(c['gates']['maximum_fixture_plane_error_m']))
 r.update({'stage':'Phase 0S-SOFA Stage C V2.3-L','execution':'offline-only','source_v2_3_r2_verdict':m['verdict'],'source_crossing_count':int(b['crossing_event_count']),'source_formal_geometric_onset_s':float(b['formal_geometric_onset_time_s']),'sofa_executed':False,'docker_executed':False,'gpu_executed':False,'physics_changed':False,'trace_regenerated':False,'stage_d_e_f_executed':False,'oracle_executed':False,'matched_state_executed':False,'training_executed':False});r.update(narrative(r));OUT_METRICS.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');OUT_REPORT.write_text('# Phase 0S-SOFA Stage C V2.3-L Crossing-Local / Global-Response Audit\n\n## Verdict\n\n`'+r['verdict']+'`\n\n## Scope\n\nOffline-only analysis of the committed V2.3-R2 trace. No SOFA, Docker, GPU, trace regeneration or physics change.\n\nThe formal verdict uses the earliest crossing only. Later crossings are propagation diagnostics.\n\nNode-local DetectionOutput/contact state is available. Constraint-vector and reaction signals are global and are not node-mapped.\n\n## Per-crossing state\n\n'+table(r['per_node_summary'])+'\n\n## Fact\n\n'+r['fact']+'\n\n## Inference\n\n'+r['inference']+'\n\n## normal_y diagnostic\n\n'+r['normal_diagnostic']+'\n\n## Unknown\n\n'+r['unknown']+'\n\n## Next action\n\n'+r['next_action']+'\n',encoding='utf-8');print(r['verdict']);return 0
if __name__=='__main__':raise SystemExit(main())
