#!/usr/bin/env python3
"""Run the single preregistered V2.3 C1 and audit it in gate order."""
from __future__ import annotations
import argparse, hashlib, json, os, subprocess, sys, tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];SPIKE=ROOT/'simulator-spikes'/'sofa-beam-spike';sys.path.insert(0,str(SPIKE/'src'))
from stage_c_collision_radius_v2_3 import *
REPORT_DIR=SPIKE/'reports'/'phase0s_sofa';TRACE=REPORT_DIR/'data'/'stage_c_v2_3_collision_radius_trace.json';METRICS=REPORT_DIR/'stage_c_collision_radius_v2_3_metrics.json';REPORT=REPORT_DIR/'STAGE_C_COLLISION_RADIUS_V2_3.md';PLUGIN=SPIKE/'native-contact-bridge'/'build'/'lib'/'libNativeContactBridge.so'
BASE='60f542eaa1161c8491a40651c7b012ebd799c85b';FROZEN={SPIKE/'configs'/'stage_c_stable_contact_v2_2_fixture_extent.json':'0e7f1bc9a4b82defc602701bdeb14022acc4e543',SPIKE/'src'/'stage_c_stable_contact_v2_scene.py':'96df7abc3a72bf2221bf5393424203d432ac0c82',SPIKE/'src'/'stage_c_contact_semantics.py':'a130d83e11ba4702c44b94344f766f44df6abbc7',SPIKE/'src'/'stage_c_contact_loss_morphology.py':'fc45ae04dbc4e27072cf5301b848ae661014e4ee',SPIKE/'native-contact-bridge'/'src'/'NativeContactBridge'/'NativeContactBridge.cpp':'e6fb41f9b903e53c33583a3cbde3334f9c117dbb',REPORT_DIR/'stage_c_fixture_extent_v2_2_metrics.json':'7c63358435c7108226b5f2eca8a585775cc19388',REPORT_DIR/'stage_c_loss_temporal_order_v2_2l_metrics.json':'97b3e0f4fe21e8524fc7aea1e0e331ef804ef566'}
IMPL={SPIKE/'configs'/'stage_c_collision_radius_v2_3.json',SPIKE/'src'/'stage_c_collision_radius_v2_3.py',SPIKE/'src'/'stage_c_collision_radius_v2_3_scene.py',Path(__file__),SPIKE/'tests'/'test_stage_c_collision_radius_v2_3.py'}
def sha(p):return hashlib.sha1(p.read_bytes()).hexdigest()
def existing_evidence_paths(paths=None):return [p for p in (paths or (TRACE,METRICS,REPORT)) if p.exists()]
def existing_evidence_message(ps):return 'PHASE0S_SOFA_STAGE_C_V2_3_EXISTING_EVIDENCE_REFUSAL: '+', '.join(map(str,ps))
def git(*args):return subprocess.run(['git',*args],cwd=ROOT,text=True,capture_output=True,check=False)
def provenance():
 parent=git('rev-parse','HEAD^').stdout.strip();changed={ROOT/x for x in git('diff','--name-only','HEAD^','HEAD').stdout.splitlines()};dirty=git('status','--porcelain').stdout.splitlines();return {'base':BASE,'implementation_commit':git('rev-parse','HEAD').stdout.strip(),'implementation_parent':parent,'implementation_parent_pass':parent==BASE,'implementation_scope_paths':[str(x.relative_to(ROOT)) for x in sorted(changed)],'implementation_scope_pass':changed==IMPL,'clean_formal_run_tree_pass':not dirty,'frozen_blobs':{str(p.relative_to(ROOT)):sha(p) for p in FROZEN},'frozen_blobs_pass':all(sha(p)==v for p,v in FROZEN.items())}
def instrumentation(payload,cfg):
 rs=payload.get('records',[]); meta=payload.get('metadata',{}); serial=[int(r.get('bridge_frame_serial',-1)) for r in rs];return {'records':len(rs),'expected_records':int(cfg['simulation']['contact_steps']),'material_readback':bool(meta.get('material_snapshot')),'bridge_ready_all':bool(rs) and all(r.get('bridge_ready') for r in rs),'fresh_serial_each_step':len(serial)==len(set(serial)) and all(x>=0 for x in serial),'pass':len(rs)==int(cfg['simulation']['contact_steps']) and bool(meta.get('material_snapshot')) and bool(rs) and all(r.get('bridge_ready') for r in rs) and len(serial)==len(set(serial))}
def skipped(reason):return {'evaluated':False,'reason':reason,'pass':False}
def narrative(v):
 if v==V_PASS:return {'fact':'The one C1 trace satisfied every preregistered Stage-C V2.3 gate.','inference':'Sphere/Triangle contact observability is qualified for Stage C; no later stage was run.','unknown':'Held-out contact-regime capability remains untested.','next_action':'Obtain a new instruction before any later Stage-C or capability work.'}
 return {'fact':'The single preregistered C1 trace was evaluated only through the first failing ordered gate.','inference':'This result does not authorize any subsequent Stage-C or capability experiment.','unknown':'No conclusion is drawn about later physical regimes.','next_action':'Review this one V2.3-R1 result before any further simulator experiment.'}
def write_outputs(m):
 REPORT_DIR.mkdir(parents=True,exist_ok=True);METRICS.write_text(json.dumps(m,indent=2)+'\n');REPORT.write_text('# Phase 0S-SOFA Stage C V2.3-R1 Collision Radius Audit\n\n## Verdict\n\n`'+m['verdict']+'`\n\n## Metrics\n\n```json\n'+json.dumps(m,indent=2)+'\n```\n\n## Fact\n\n'+m['fact']+'\n\n## Inference\n\n'+m['inference']+'\n\n## Unknown\n\n'+m['unknown']+'\n\n## Next action\n\n'+m['next_action']+'\n')
def formal_run(cfg):
 if not PLUGIN.exists():raise RuntimeError('NativeContactBridge artifact missing: '+str(PLUGIN))
 with tempfile.TemporaryDirectory() as td:
  cp=Path(td)/'config.json';cp.write_text(json.dumps(cfg));env=os.environ.copy();env.update({'CUDA_VISIBLE_DEVICES':'','SOFA_STAGE_C_V23_CONFIG':'/work/'+str(cp.relative_to(ROOT)),'SOFA_STAGE_C_V23_TRACE':'/work/'+str(TRACE.relative_to(ROOT))})
  # Mounting the repository makes the temporary config inaccessible, so create it under repo only transiently is disallowed; Docker reads host config through /work existing configuration.
  env['SOFA_STAGE_C_V23_CONFIG']='/work/simulator-spikes/sofa-beam-spike/configs/stage_c_collision_radius_v2_3.json'
  cmd=['docker','run','--rm','--network','none','-e','CUDA_VISIBLE_DEVICES=','-e','SOFA_STAGE_C_V23_CONFIG='+env['SOFA_STAGE_C_V23_CONFIG'],'-e','SOFA_STAGE_C_V23_TRACE='+env['SOFA_STAGE_C_V23_TRACE'],'-e','LD_LIBRARY_PATH=/sofa/lib','-v',f'{ROOT}:/work','-v','/root/workspace/third_party/SOFA_v26.06.00_Linux:/sofa:ro','sofa-v2606-python312-runner:ubuntu24','/sofa/bin/runSofa','-l','SofaPython3','-l','/work/'+str(PLUGIN.relative_to(ROOT)),'-g','batch','-n','/work/simulator-spikes/sofa-beam-spike/src/stage_c_collision_radius_v2_3_scene.py']
  p=subprocess.run(cmd,text=True,capture_output=True,env=env);out=(p.stdout+'\n'+p.stderr).splitlines();bad=[x for x in out if 'Plugin not found: "NativeContactBridge"' not in x]
  if p.returncode or not TRACE.exists():raise RuntimeError('runSofa failed or no trace; last 80 stdout/stderr:\n'+'\n'.join(bad[-80:]))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);a=ap.parse_args();existing=existing_evidence_paths()
 if existing:print(existing_evidence_message(existing),file=sys.stderr);return 2
 cfg=json.loads(Path(a.config).read_text());m={'stage':'Phase 0S-SOFA Stage C V2.3-R1','cpu_only':True,'seed':cfg['seed'],'formal_c1_runs_requested':1,'formal_c1_runs_completed':0,'automatic_retry':False,'provenance':provenance(),'existing_evidence_guard':{'pass':True,'checked_paths':[str(p.relative_to(ROOT)) for p in (TRACE,METRICS,REPORT)]}}
 if not (m['provenance']['implementation_parent_pass'] and m['provenance']['implementation_scope_pass'] and m['provenance']['frozen_blobs_pass']):m['verdict']=V_INPUT;m.update(narrative(m['verdict']));write_outputs(m);print(m['verdict']);return 2
 base=json.loads((SPIKE/'configs'/'stage_c_stable_contact_v2_2_fixture_extent.json').read_text());inter=compensated_intervention_gate(base,cfg);m['compensated_intervention_gate']=inter
 if not inter['pass']:m['verdict']=V_INPUT;m.update(narrative(m['verdict']));write_outputs(m);print(m['verdict']);return 2
 try: formal_run(cfg);m['formal_c1_runs_completed']=1;payload=json.loads(TRACE.read_text())
 except Exception as e:m['formal_run_error']=str(e);m['verdict']=V_INST;m.update(narrative(m['verdict']));write_outputs(m);print(m['verdict']);return 0
 inst=instrumentation(payload,cfg);m['instrumentation']=inst
 if not inst['pass']:m.update({'semantics':skipped('instrumentation failed'),'verdict':V_INST});m.update(narrative(V_INST));write_outputs(m);print(V_INST);return 0
 classes=classify_trace(payload['records'],effective_contact_distance_m=cfg['beam']['radius_m'],reaction_floor=0.);sem=semantics_gate(payload,classes,cfg,base);m['semantics']=sem
 if not sem['pass']:m.update({'zero_load':skipped('semantics failed'),'verdict':V_SEMANTICS});m.update(narrative(V_SEMANTICS));write_outputs(m);print(V_SEMANTICS);return 0
 zero=zero_load_gate(payload['records'],classes);m['zero_load']=zero
 if not zero['pass']:m.update({'barrier':skipped('zero-load failed'),'verdict':V_ZERO});m.update(narrative(V_ZERO));write_outputs(m);print(V_ZERO);return 0
 barrier=barrier_crossing_gate(payload['records'],classes,cfg);m['barrier']=barrier
 if not barrier['pass']:m.update({'mobile_fixture_edge':skipped('barrier failed'),'verdict':V_CROSS});m.update(narrative(V_CROSS));write_outputs(m);print(V_CROSS);return 0
 edge=mobile_fixture_edge_gate(payload['records'],classes,cfg);m['mobile_fixture_edge']=edge
 if not edge['pass']:m.update({'contact':skipped('edge failed'),'verdict':V_EDGE});m.update(narrative(V_EDGE));write_outputs(m);print(V_EDGE);return 0
 con=contact_gate(payload['records'],classes,cfg);m['contact']=con
 if not con['pass']:m.update({'reaction':skipped('contact failed'),'verdict':V_CONTACT});m.update(narrative(V_CONTACT));write_outputs(m);print(V_CONTACT);return 0
 rea=reaction_gate(payload['records'],classes,cfg);m['reaction']=rea
 if not rea['pass']:m.update({'geometry':skipped('reaction failed'),'verdict':V_REACTION});m.update(narrative(V_REACTION));write_outputs(m);print(V_REACTION);return 0
 geo=geometry_gate(payload['records'],classes,cfg);m.update({'geometry':geo,'formal_measurement_relation':relation_table(payload['records'],classes),'verdict':V_PASS if geo['pass'] else V_GEOMETRY});m.update(narrative(m['verdict']));write_outputs(m);print(m['verdict']);return 0
if __name__=='__main__':raise SystemExit(main())
