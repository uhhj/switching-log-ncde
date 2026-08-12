"""Single-run Phase-0S Stage-C V2.2 fixture-extent correction audit."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPIKE = ROOT / "simulator-spikes" / "sofa-beam-spike"
sys.path.insert(0, str(SPIKE / "src"))
from contact_native import contact_episode_stats_timestamps  # noqa: E402
from stage_c_contact_loss_morphology import classify_contact_loss_morphology  # noqa: E402
from stage_c_contact_semantics import classify_record, first_active_time_s, selected_geometric_contacts  # noqa: E402

BASE_HEAD = "86fe72385db067946f91b284339eb1c817e62b75"
BRANCH = "phase0s-sofa-beamadapter"
BASE_CONFIG = SPIKE / "configs" / "stage_c_stable_contact_v2.json"
CONFIG = SPIKE / "configs" / "stage_c_stable_contact_v2_2_fixture_extent.json"
SCENE = SPIKE / "src" / "stage_c_stable_contact_v2_scene.py"
SEMANTICS = SPIKE / "src" / "stage_c_contact_semantics.py"
MORPHOLOGY = SPIKE / "src" / "stage_c_contact_loss_morphology.py"
REPORT_DIR = SPIKE / "reports" / "phase0s_sofa"
C0_METRICS = REPORT_DIR / "stage_c_stable_contact_v2_metrics.json"
V21_METRICS = REPORT_DIR / "stage_c_contact_semantics_v2_1_metrics.json"
LOSS_METRICS = REPORT_DIR / "stage_c_contact_loss_morphology_v2_1l_metrics.json"
PLUGIN = SPIKE / "native-contact-bridge" / "build" / "lib" / "libNativeContactBridge.so"
TRACE = REPORT_DIR / "data" / "stage_c_v2_2_fixture_extent_trace.json"
METRICS = REPORT_DIR / "stage_c_fixture_extent_v2_2_metrics.json"
REPORT = REPORT_DIR / "STAGE_C_FIXTURE_EXTENT_V2_2.md"
V_INPUT="PHASE0S_SOFA_STAGE_C_V2_2_INPUT_INVALID"; V_INST="PHASE0S_SOFA_STAGE_C_V2_2_INSTRUMENTATION_BLOCKED"; V_ZERO="PHASE0S_SOFA_STAGE_C_V2_2_ZERO_LOAD_FAIL"; V_EDGE="PHASE0S_SOFA_STAGE_C_V2_2_FIXTURE_EXTENT_NOT_CORRECTED"; V_CROSS="PHASE0S_SOFA_STAGE_C_V2_2_PLANE_CROSSING_EXPOSED"; V_CONTACT="PHASE0S_SOFA_STAGE_C_V2_2_CONTACT_CONSTRUCTION_FAIL"; V_REACTION="PHASE0S_SOFA_STAGE_C_V2_2_REACTION_PROXY_FAIL"; V_GEOM="PHASE0S_SOFA_STAGE_C_V2_2_GEOMETRY_FAIL"; V_PASS="PHASE0S_SOFA_STAGE_C_V2_2_STABLE_CONTACT_PASS"
IMPLEMENTATION_FILES={"simulator-spikes/sofa-beam-spike/configs/stage_c_stable_contact_v2_2_fixture_extent.json","simulator-spikes/sofa-beam-spike/scripts/audit_stage_c_fixture_extent_v2_2.py","simulator-spikes/sofa-beam-spike/tests/test_stage_c_fixture_extent_v2_2.py"}
EXPECTED_BLOBS={BASE_CONFIG:"1aaab0991983b65c2acf7613b173c5920615e86c",SCENE:"96df7abc3a72bf2221bf5393424203d432ac0c82",SEMANTICS:"a130d83e11ba4702c44b94344f766f44df6abbc7",MORPHOLOGY:"fc45ae04dbc4e27072cf5301b848ae661014e4ee",C0_METRICS:"9f89484d2b862793b2d12790ddde2d80ccef1fe6",V21_METRICS:"f4bac4d185b820fb2ec70900f82312013f748fc9",LOSS_METRICS:"6a8322f915c6648c00735c61e14296eaf4da469b"}

def git(*args): return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
def sha256(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda:f.read(1024*1024),b""): h.update(chunk)
    return h.hexdigest()
def deep_diff_paths(a,b,prefix=""):
    if isinstance(a,Mapping) and isinstance(b,Mapping):
        out=[]
        for key in sorted(set(a)|set(b)):
            path=f"{prefix}.{key}" if prefix else str(key)
            out.extend([path] if key not in a or key not in b else deep_diff_paths(a[key],b[key],path))
        return out
    if isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b): return [prefix]
        return [p for i,(x,y) in enumerate(zip(a,b)) for p in deep_diff_paths(x,y,f"{prefix}[{i}]")]
    return [] if a==b else [prefix]
def config_delta(base,candidate):
    paths=deep_diff_paths(base,candidate); nominal=np.linspace(0.0,float(candidate["beam"]["length_m"]),int(candidate["beam"]["nodes"])); old=float(base["contact_wall"]["x_min_m"]); new=float(candidate["contact_wall"]["x_min_m"])
    return {"diff_paths":paths,"old_x_min_m":old,"new_x_min_m":new,"nominal_beam_x_min_m":float(nominal.min()),"nominal_beam_x_max_m":float(nominal.max()),"pass":bool(paths==["contact_wall.x_min_m"] and old==0.08 and new==0.0 and new==float(nominal.min()))}
def provenance():
    head,parent=git("rev-parse","HEAD"),git("rev-parse","HEAD^"); status=git("status","--short"); changed=set(filter(None,git("diff","--name-only",f"{BASE_HEAD}..{head}").splitlines())); actual={str(p.relative_to(ROOT)):git("hash-object",str(p)) for p in EXPECTED_BLOBS}; expected={str(p.relative_to(ROOT)):v for p,v in EXPECTED_BLOBS.items()}
    return {"base_head":BASE_HEAD,"implementation_head":head,"implementation_parent":parent,"implementation_parent_matches_base":parent==BASE_HEAD,"branch":git("branch","--show-current"),"working_tree_clean_at_audit_start":status=="","implementation_changed_files":sorted(changed),"required_implementation_files":sorted(IMPLEMENTATION_FILES),"implementation_scope_exact":changed==IMPLEMENTATION_FILES,"blob_hashes":actual,"expected_blob_hashes":expected,"pass":bool(parent==BASE_HEAD and git("branch","--show-current")==BRANCH and status=="" and changed==IMPLEMENTATION_FILES and actual==expected)}
def material_gate(payload,config):
    snap=payload["metadata"]["material_snapshot"]; edges=int(config["beam"]["nodes"])-1; requested={"default_young_modulus_pa":float(config["beam"]["young_modulus_pa"]),"default_poisson_ratio":float(config["beam"]["poisson_ratio"]),"radius_m":float(config["beam"]["radius_m"]),"mass_density_kg_m3":float(config["beam"]["mass_density_kg_m3"])}; checks={}
    for key,target in requested.items():
        values=np.asarray(snap[key],dtype=float).reshape(-1); checks[key]={"values":values.tolist(),"requested":target,"pass":bool(values.size==edges and np.allclose(values,target,rtol=0,atol=1e-12))}
    return {"checks":checks,"pass":all(x["pass"] for x in checks.values())}
def bridge_gate(records):
    steps=np.asarray([r["step_index"] for r in records],dtype=int); serials=np.asarray([r["bridge_frame_serial"] for r in records],dtype=int); sources=np.asarray([r["native_contact_source_serial"] for r in records],dtype=int); diffs=np.diff(serials); expected=np.arange(len(records),dtype=int)
    out={"step_indices_exact":bool(np.array_equal(steps,expected)),"serials_exact":bool(np.array_equal(serials,expected+1)),"source_serial_matches":bool(np.array_equal(sources,serials)),"duplicates":int(np.count_nonzero(diffs==0)),"gaps":int(np.count_nonzero(diffs>1)),"regressions":int(np.count_nonzero(diffs<0))}; out["pass"]=bool(out["step_indices_exact"] and out["serials_exact"] and out["source_serial_matches"] and not(out["duplicates"] or out["gaps"] or out["regressions"])); return out
def timing_gate(records,config):
    dt=float(config["simulation"]["dt_s"]); tol=float(config["integrity"]["time_tolerance_s"]); starts=np.asarray([r["step_start_time_s"] for r in records],dtype=float); ends=np.asarray([r["step_end_time_s"] for r in records],dtype=float); i=np.arange(len(records),dtype=float); a=float(np.max(np.abs(starts-i*dt))); b=float(np.max(np.abs(ends-(i+1)*dt))); c=float(np.max(np.abs((ends-starts)-dt))); return {"start_error_max_s":a,"end_error_max_s":b,"interval_error_max_s":c,"pass":bool(a<=tol and b<=tol and c<=tol)}
def force_gate(records,target):
    bad=[]; shape=[]; maximum=0.0
    for r in records:
        step=int(r["step_index"]); schedule=np.asarray(r["scheduled_force_n"],dtype=float).reshape(3); bi=np.asarray(r["force_field_indices_begin"],dtype=int).reshape(-1); ei=np.asarray(r["force_field_indices_end"],dtype=int).reshape(-1); br=np.asarray(r["force_field_force_data_begin_n"],dtype=float); er=np.asarray(r["force_field_force_data_end_n"],dtype=float)
        if br.size%6 or er.size%6: shape.append(step); continue
        br,er=br.reshape(-1,6),er.reshape(-1,6)
        if not np.array_equal(bi,[target]) or not np.array_equal(ei,[target]) or br.shape!=(1,6) or er.shape!=(1,6): shape.append(step); continue
        error=max(float(np.max(np.abs(br[0]-np.r_[schedule,[0.,0.,0.]]))),float(np.max(np.abs(er[0]-np.r_[schedule,[0.,0.,0.]])))); maximum=max(maximum,error)
        if error>1e-15: bad.append(step)
    return {"target_index":int(target),"shape_or_index_mismatch_steps":shape,"force_mismatch_steps":bad,"max_abs_error_n":maximum,"pass":bool(not shape and not bad and maximum<=1e-15)}
def instrumentation(payload,config):
    records=payload["records"]
    if len(records)!=int(config["simulation"]["contact_steps"]): return {"pass":False,"reason":"trace_length_mismatch"}
    if payload.get("metadata",{}).get("mode")!="stable_contact": return {"pass":False,"reason":"mode_mismatch"}
    material,bridge,timing,force=material_gate(payload,config),bridge_gate(records),timing_gate(records,config),force_gate(records,int(config["beam"]["nodes"])-1); ready=all(bool(r["bridge_ready"]) for r in records); tail=max(float(r["tail_translation_displacement_m"]) for r in records); tail_ok=tail<=float(config["integrity"]["tail_translation_max_m"])
    return {"material":material,"bridge":bridge,"timing":timing,"force":force,"bridge_ready_all":bool(ready),"tail_translation_max_m":tail,"tail_anchor_pass":bool(tail_ok),"pass":bool(material["pass"] and bridge["pass"] and timing["pass"] and force["pass"] and ready and tail_ok)}
def classify(records,effective,floor): return [classify_record(r,effective_contact_distance_m=effective,reaction_floor=floor) for r in records]
def zero_load(records,classes):
    pairs=[(r,c) for r,c in zip(records,classes) if bool(r["zero_load_phase"])]; out={"samples":len(pairs),"native_detection_frames":int(sum(c["native_detection"] for _,c in pairs)),"proximity_only_frames":int(sum(c["proximity_only"] for _,c in pairs)),"geometric_contact_frames":int(sum(c["geometric_contact"] for _,c in pairs)),"lcp_row_frames":int(sum(c["lcp_rows_present"] for _,c in pairs)),"reaction_active_frames":int(sum(c["reaction_active"] for _,c in pairs))}; out["pass"]=bool(out["geometric_contact_frames"]==0 and out["reaction_active_frames"]==0); return out
def translation_fixed_node_ids(config):
    tail=config["tail_constraint"]; directions=[int(v) for v in tail["fixed_directions"]]
    if len(directions)<3: raise ValueError("tail fixed_directions must expose xyz translation flags")
    return {int(tail["node_index"])} if directions[:3]==[1,1,1] else set()
def _mobile_node_ids(config):
    all_ids=set(range(int(config["beam"]["nodes"]))); excluded=translation_fixed_node_ids(config)
    if not excluded.issubset(all_ids): raise ValueError("translation-fixed node id outside beam node range")
    return sorted(all_ids-excluded),sorted(excluded)
def _node_inside_fixture_footprint(point,wall,tolerance_m):
    x,_,z=[float(v) for v in point[:3]]; tol=float(tolerance_m); return bool(float(wall["x_min_m"])-tol<=x<=float(wall["x_max_m"])+tol and -float(wall["z_half_extent_m"])-tol<=z<=float(wall["z_half_extent_m"])+tol)
def mobile_fixture_edge_gate(records,classes,config):
    if len(records)!=len(classes): raise ValueError("record/classification length mismatch")
    mobile,excluded=_mobile_node_ids(config); wall=config["contact_wall"]; tol=float(config["gates"]["maximum_fixture_plane_error_m"]); onset=next((i for i,c in enumerate(classes) if bool(c["geometric_contact"])),None)
    def nodes(record,ids):
        pos=np.asarray(record["beam_node_positions_m"],dtype=float)
        if pos.ndim!=2 or pos.shape[0]!=int(config["beam"]["nodes"]): raise ValueError("beam_node_positions_m shape mismatch")
        return [node for node in ids if _node_inside_fixture_footprint(pos[node],wall,tol)]
    terminal=records[-1]; terminal_mobile=nodes(terminal,mobile); terminal_all=nodes(terminal,range(int(config["beam"]["nodes"]))); native=sorted({int(c["beam_element_id"]) for c in terminal["native_contacts"] if int(c["beam_element_id"]) in set(mobile)}); ever=False if onset is None else any(bool(nodes(r,mobile)) for r in records[onset:]); escape=bool(onset is not None and ever and not terminal_mobile and not native)
    return {"evaluated":onset is not None,"formal_onset_source":"geometric_contact","geometric_first_index":onset,"geometric_first_time_s":float(records[onset]["step_end_time_s"]) if onset is not None else None,"excluded_translation_fixed_node_ids":excluded,"mobile_node_ids":mobile,"ever_mobile_footprint_after_onset":bool(ever),"terminal_mobile_footprint_node_ids":terminal_mobile,"terminal_mobile_footprint_node_count":len(terminal_mobile),"terminal_mobile_native_detection_ids":native,"terminal_mobile_native_detection":bool(native),"terminal_all_node_footprint_node_ids_diagnostic":terminal_all,"terminal_all_node_footprint_node_count_diagnostic":len(terminal_all),"terminal_all_native_detection_diagnostic":bool(terminal["native_contact_count"]>0),"fixture_edge_escape":escape,"pass":bool(not escape)}
def contact_gate(records,classes,config):
    pairs=[(r,c) for r,c in zip(records,classes) if bool(r["measurement_active"])]; stats=contact_episode_stats_timestamps([float(r["step_end_time_s"]) for r,_ in pairs],[bool(c["geometric_contact"]) for _,c in pairs]); req_occ=float(config["gates"]["minimum_measurement_contact_occupancy"]); req_dwell=float(config["gates"]["minimum_observed_contact_dwell_ms"])/1000.; tol=float(config["integrity"]["time_tolerance_s"])
    return {"measurement_samples":len(pairs),"episode":stats,"longest_episode":max(stats["episode_records"],key=lambda x:x["duration_s"],default=None),"required_occupancy":req_occ,"required_dwell_s":req_dwell,"time_tolerance_s":tol,"duration_semantics":"timestamp_span_end_minus_start","pass":bool(stats["occupancy"]>=req_occ and stats["longest_duration_s"]+tol>=req_dwell)}
def reaction_gate(records,classes,config,floor):
    pairs=[(r,c) for r,c in zip(records,classes) if bool(r["measurement_active"]) and bool(c["geometric_contact"])]; lcp=float(np.mean([c["lcp_rows_present"] for _,c in pairs])) if pairs else 0.; positive=float(np.mean([c["reaction_active"] for _,c in pairs])) if pairs else 0.; values=np.asarray([float(r["reaction_force_proxy"]) for r,_ in pairs],dtype=float); geometric=first_active_time_s(records,classes,"geometric_contact"); reaction=first_active_time_s(records,classes,"reaction_active"); lag=float(reaction)-float(geometric) if geometric is not None and reaction is not None else None; allowed=int(config["gates"]["maximum_native_lcp_onset_lag_steps"])*float(config["simulation"]["dt_s"]); tol=float(config["integrity"]["time_tolerance_s"]); causal=bool(lag is not None and lag>=-tol); upper=bool(lag is not None and lag<=allowed+tol); pre=int(sum(bool(c["reaction_active"]) for r,c in zip(records,classes) if geometric is not None and float(r["step_end_time_s"])<geometric-tol)) if geometric is not None else 0
    out={"geometric_measurement_frames":len(pairs),"lcp_fraction":lcp,"positive_reaction_fraction":positive,"reaction_floor_c0_p99":float(floor),"p10":float(np.percentile(values,10)) if values.size else None,"p50":float(np.percentile(values,50)) if values.size else None,"p90":float(np.percentile(values,90)) if values.size else None,"max":float(values.max()) if values.size else None,"geometric_onset_s":geometric,"reaction_onset_s":reaction,"reaction_minus_geometric_onset_s":lag,"allowed_positive_onset_lag_s":allowed,"time_tolerance_s":tol,"causal_order_pass":causal,"upper_lag_pass":upper,"pre_geometric_reaction_frames":pre}; out["pass"]=bool(pairs and lcp>=float(config["gates"]["minimum_contact_lcp_fraction"]) and positive>=float(config["gates"]["minimum_contact_positive_reaction_fraction"]) and values.size and float(np.percentile(values,10))>float(floor) and causal and upper and pre==0); return out
def geometry_gate(records,classes,config,effective):
    pairs=[(r,c) for r,c in zip(records,classes) if bool(r["measurement_active"])]; contacts=selected_geometric_contacts([r for r,_ in pairs],[c for _,c in pairs],effective)
    if not contacts:return {"evaluated":True,"geometric_contact_samples":0,"pass":False}
    wall_y=float(config["contact_wall"]["plane_y_m"]); fixture=np.asarray([c["fixture_contact_point_m"] for c in contacts],dtype=float); normals=np.asarray([c["beam_outward_normal"] for c in contacts],dtype=float); unique=[];spans=[]
    for r,c in pairs:
        if c["geometric_contact"]:
            selected=[x for x,g in zip(r["native_contacts"],c["local_min_distance_gaps_m"]) if g<=0]; unique.append(len({int(x["beam_element_id"]) for x in selected})); xs=[float(x["beam_contact_point_m"][0]) for x in selected];spans.append(float(np.ptp(xs)) if xs else 0.)
    plane=float(np.max(np.abs(fixture[:,1]-wall_y))); normal=float(np.percentile(np.abs(normals[:,1]),10)); primitives=float(np.percentile(unique,95)); span=float(np.percentile(spans,95)); g=config["gates"]; return {"evaluated":True,"geometric_contact_samples":len(contacts),"fixture_plane_error_max_m":plane,"normal_alignment_p10":normal,"unique_beam_primitives_p95":primitives,"contact_x_span_p95_m":span,"pass":bool(plane<=float(g["maximum_fixture_plane_error_m"]) and normal>=float(g["minimum_normal_alignment_p10"]) and primitives<=float(g["maximum_unique_beam_primitives_p95"]) and span<=float(g["maximum_contact_x_span_p95_m"]))}
def choose_verdict(inst,zero,edge,morph,contact,reaction,geometry):
    if not inst:return V_INST
    if not zero:return V_ZERO
    if bool(edge.get("fixture_edge_escape")):return V_EDGE
    if int(morph.get("crossing_event_count",0))>0:return V_CROSS
    if not contact:return V_CONTACT
    if not reaction:return V_REACTION
    if not geometry:return V_GEOM
    return V_PASS
def run_once(config):
    if TRACE.exists():raise RuntimeError("V2.2 trace already exists; refusing a second formal run")
    if not PLUGIN.is_file():raise RuntimeError(f"NativeContactBridge missing: {PLUGIN}")
    TRACE.parent.mkdir(parents=True,exist_ok=True); command=["docker","run","--rm","--network","none","-e","CUDA_VISIBLE_DEVICES=","-e","LD_LIBRARY_PATH=/sofa/lib","-e",f"SOFA_STAGE_C_V2_CONFIG=/work/{CONFIG.relative_to(ROOT).as_posix()}","-e","SOFA_STAGE_C_V2_MODE=stable_contact","-e",f"SOFA_STAGE_C_V2_TRACE=/work/{TRACE.relative_to(ROOT).as_posix()}","-v",f"{ROOT}:/work","-v",f"{config['runtime']['sofa_root']}:/sofa:ro",config["runtime"]["image"],"/sofa/bin/runSofa","-l","SofaPython3","-l",f"/work/{PLUGIN.relative_to(ROOT).as_posix()}","-g","batch","-n",str(int(config["simulation"]["contact_steps"])),f"/work/{SCENE.relative_to(ROOT).as_posix()}"]
    done=subprocess.run(command,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT); log=REPORT_DIR/".stage_c_v2_2_run.log";log.write_text(done.stdout,encoding="utf-8"); errors=[line for line in done.stdout.splitlines() if "[ERROR]" in line and 'Plugin not found: "NativeContactBridge"' not in line]
    if done.returncode!=0 or errors or "NaN" in done.stdout or not TRACE.is_file():raise RuntimeError(f"single V2.2 C1 run failed; see {log}")
    log.unlink(missing_ok=True)
def narrative(v):
    m={V_INST:("The single V2.2 trajectory failed instrumentation/provenance validation.","No Stage-C physics conclusion is permitted.","The fixture-only intervention remains scientifically unresolved.","Repair only the failing engineering/instrumentation path; do not change physics."),V_ZERO:("The expanded fixture violates the corrected zero-load geometric/reaction baseline.","The proposed extent does not preserve the intended contact-free initial state.","Loaded stable-contact behavior is unqualified.","STOP and review the fixture-only construction; do not tune load/contact thresholds."),V_EDGE:("Fixture-edge escape still occurs after x_min is moved to the nominal beam-domain boundary.","The single x_min correction is insufficient to remove finite-fixture escape.","Reaction coupling and later stages remain unqualified.","STOP; do not expand x_max/z without a new morphology-based justification."),V_CROSS:("Fixture-edge escape is no longer the first blocker, but post-onset positive-to-negative plane crossing is observed.","The next exposed Stage-C problem is collision/barrier representation rather than fixture extent.","The correct radius-carrying collision representation remains unqualified.","Design exactly one collision-representation correction; keep fixture extent/load/material/solver frozen."),V_CONTACT:("Edge escape and plane crossing are excluded, but the frozen geometric-contact occupancy/dwell gate fails.","Another construction mechanism limits sustained contact.","The next dominant mechanism must come from the emitted morphology evidence.","STOP and select one further correction only from the V2.2 morphology evidence."),V_REACTION:("Stable geometric contact passes, but the frozen reaction-coupling gate fails.","The global LCP reaction proxy is not qualified for this stable regime.","Breakaway and all later capabilities remain unqualified.","Audit one reaction/constraint-coupling path only; do not change geometry or preload."),V_GEOM:("Sustained contact and reaction coupling pass, but the frozen local-contact geometry gate fails.","The expanded fixture produces an unsuitable contact geometry such as extended laydown.","Breakaway and later capabilities remain unqualified.","STOP and review contact distribution; do not relax geometry gates."),V_PASS:("The single fixture x_min correction passes instrumentation, corrected zero-load semantics, timestamp-span stable contact, reaction coupling, local geometry, and zero-crossing exclusion.","Stage C stable normal-contact construction is qualified for this exact V2.2 fixture extent.","Breakaway, stick/slip, jam, Oracle, capability, and passage remain untested.","STOP for review before Stage D; do not run breakaway in this task.")}; return m[v]
def write_outputs(metrics):
    REPORT_DIR.mkdir(parents=True,exist_ok=True);METRICS.write_text(json.dumps(metrics,indent=2)+"\n",encoding="utf-8");REPORT.write_text("# Phase 0S-SOFA Stage C V2.2 Fixture-Extent Correction\n\n## Verdict\n\n`%s`\n\n## Scope\n\nOne physical change only: `contact_wall.x_min_m` 0.08 -> 0.00 m. One CPU-only C1 trajectory; no C0 rerun, sweep, Stage D/E/F, Oracle, or training.\n\n## Metrics\n\n```json\n%s\n```\n\n## Fact\n\n%s\n\n## Inference\n\n%s\n\n## Unknown\n\n%s\n\n## Next action\n\n%s\n"%(metrics["verdict"],json.dumps(metrics,indent=2),metrics["fact"],metrics["inference"],metrics["unknown"],metrics["next_action"]),encoding="utf-8")
def main():
    parser=argparse.ArgumentParser();parser.add_argument("--config",type=Path,default=CONFIG);args=parser.parse_args()
    if args.config.resolve()!=CONFIG.resolve():raise RuntimeError("only the preregistered V2.2 config is allowed")
    if os.environ.get("CUDA_VISIBLE_DEVICES") not in {"",None}:raise RuntimeError("CPU-only")
    prov=provenance();metrics={"stage":"Phase 0S-SOFA Stage C V2.2","cpu_only":True,"formal_c1_runs_requested":1,"formal_c1_runs_completed":0,"c0_rerun":False,"stage_d_executed":False,"stages_e_f_executed":False,"oracle_fitted":False,"oracle_frozen":False,"training_executed":False,"input_provenance":prov}
    if not prov["pass"]:metrics.update(verdict=V_INPUT,fact="Frozen-input or two-commit provenance failed.",inference="No physical run is permitted.",unknown="The fixture-extent correction remains untested.",next_action="Restore the exact frozen base/implementation chain.");write_outputs(metrics);print(metrics["verdict"]);return 2
    base=json.loads(BASE_CONFIG.read_text());candidate=json.loads(CONFIG.read_text());delta=config_delta(base,candidate);metrics["config_delta"]=delta
    if not delta["pass"]:metrics.update(verdict=V_INPUT,fact="Candidate config is not the exact one-field x_min correction.",inference="The proposed run is not a single-variable intervention.",unknown="The intended fixture correction remains untested.",next_action="Restore the candidate config to the exact x_min-only change.");write_outputs(metrics);print(metrics["verdict"]);return 2
    old=json.loads(C0_METRICS.read_text());floor=float(old["c0_baseline"]["reaction_p99"]);metrics["historical_c0"]={"instrumentation_pass":bool(old["c0_instrumentation"]["pass"]),"baseline_pass":bool(old["c0_baseline"]["pass"]),"reaction_p99":floor,"reused_without_rerun":True}
    try:
        if not metrics["historical_c0"]["instrumentation_pass"] or not metrics["historical_c0"]["baseline_pass"]:raise RuntimeError("historical material-matched C0 prerequisite is not PASS")
        run_once(candidate);metrics["formal_c1_runs_completed"]=1;payload=json.loads(TRACE.read_text());inst=instrumentation(payload,candidate);metrics["instrumentation"]=inst
        if not inst["pass"]:verdict=V_INST;zero=contact=reaction=geometry=mobile=morphology={"evaluated":False}
        else:
            records=payload["records"];effective=float(candidate["simulation"]["contact_distance_m"]);classes=classify(records,effective,floor);zero=zero_load(records,classes);contact=contact_gate(records,classes,candidate);reaction=reaction_gate(records,classes,candidate,floor);geometry=geometry_gate(records,classes,candidate,effective);mobile=mobile_fixture_edge_gate(records,classes,candidate);morphology=classify_contact_loss_morphology(records,classes,config=candidate,effective_contact_distance_m=effective);verdict=choose_verdict(inst["pass"],zero["pass"],mobile,morphology,contact["pass"],reaction["pass"],geometry["pass"]);metrics["effective_contact_distance_m"]=effective
        metrics.update(zero_load=zero,contact=contact,reaction=reaction,geometry=geometry,mobile_fixture_edge_gate=mobile,secondary_morphology=morphology,trace={"path":str(TRACE.relative_to(ROOT)),"sha256":sha256(TRACE)},verdict=verdict);fact,inference,unknown,next_action=narrative(verdict);metrics.update(fact=fact,inference=inference,unknown=unknown,next_action=next_action)
    except Exception as exc:metrics.update(verdict=V_INST,error=repr(exc),fact="The single preregistered V2.2 C1 execution did not produce valid evidence.",inference="No physical conclusion or automatic retry is permitted.",unknown="The fixture-extent intervention remains unresolved.",next_action="STOP; repair only the identified engineering blocker before explicitly authorizing another run.")
    write_outputs(metrics);print(metrics["verdict"]);return 0
if __name__=="__main__":raise SystemExit(main())
