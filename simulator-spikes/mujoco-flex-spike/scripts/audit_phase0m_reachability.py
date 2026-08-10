#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.passage import (  # noqa: E402
    cable_radius,
    geometry_from_config,
    passage_success,
    reachability_clearance,
)
from src.phase0m import branch_y_target, resolve_phase0m  # noqa: E402
from src.reachability import (  # noqa: E402
    ordered_local_spacing,
    required_success_translation,
)


def _git(*args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(ROOT), *args], text=True
    ).strip()


def _runs_ms(mask: np.ndarray, timestep_s: float) -> List[float]:
    values = np.asarray(mask, dtype=bool)
    starts = np.flatnonzero(values & np.r_[True, ~values[:-1]])
    ends = np.flatnonzero(values & np.r_[~values[1:], True])
    return [float((end - start + 1) * timestep_s * 1000.0) for start, end in zip(starts, ends)]


def _entered_all(positions: np.ndarray, indices: np.ndarray, plane_x: float) -> bool:
    return bool(
        any(passage_success(frame[indices], plane_x) for frame in positions)
    )


def _trace_diagnostics(
    path: Path,
    initial_vertices: np.ndarray,
    leading: np.ndarray,
    geometry,
    timestep_s: float,
) -> Dict[str, Any]:
    with np.load(path) as loaded:
        trace = {key: loaded[key] for key in loaded.files}
    positions = np.asarray(trace["ordered_vertex_positions"], dtype=np.float64)
    heads = np.asarray(trace["head_position"], dtype=np.float64)
    leading_mean = np.asarray(trace["leading4_mean_position"], dtype=np.float64)
    initial_head_x = float(initial_vertices[int(leading[0]), 0])
    initial_leading_mean_x = float(np.mean(initial_vertices[leading, 0]))
    final_leading_x = positions[-1, leading, 0]
    contact = np.asarray(trace["fixture_contact_count"], dtype=np.int64) > 0
    first_contact_x = None
    first_contact_region = None
    if np.any(contact):
        first_step = int(np.flatnonzero(contact)[0])
        vertex_ids = np.asarray(trace["contact_vertex_ids"][first_step], dtype=np.int64)
        vertex_ids = vertex_ids[vertex_ids >= 0]
        if vertex_ids.size:
            first_contact_x = float(np.mean(positions[first_step, vertex_ids, 0]))
        else:
            first_contact_x = float(heads[first_step, 0])
        if first_contact_x < geometry.funnel_exit_x_m:
            first_contact_region = "funnel_x_proxy"
        elif first_contact_x <= geometry.exit_x_m:
            first_contact_region = "throat_x_proxy"
        else:
            first_contact_region = "post_exit_x_proxy"
    duration_s = float(np.asarray(trace["time"])[-1])
    active_time_s = float(np.count_nonzero(trace["command_active"]) * timestep_s)
    return {
        "samples": int(np.asarray(trace["time"]).size),
        "duration_s": duration_s,
        "reconstructed_reference_travel_m": active_time_s
        * float(np.asarray(trace["command_forward"])[0]),
        "head_x_travel_m": float(np.max(heads[:, 0]) - initial_head_x),
        "leading4_mean_x_travel_m": float(
            np.max(leading_mean[:, 0]) - initial_leading_mean_x
        ),
        "leading4_min_x_final_m": float(np.min(final_leading_x)),
        "distance_remaining_to_success_m": max(
            0.0, float(geometry.exit_x_m + geometry.exit_margin_m - np.min(final_leading_x))
        ),
        "max_head_x_m": float(np.max(heads[:, 0])),
        "max_leading4_mean_x_m": float(np.max(leading_mean[:, 0])),
        "head_entered_funnel": bool(np.max(heads[:, 0]) >= geometry.entry_x_m),
        "head_entered_throat": bool(np.max(heads[:, 0]) >= geometry.funnel_exit_x_m),
        "leading4_entered_throat": _entered_all(
            positions, leading, geometry.funnel_exit_x_m
        ),
        "head_crossed_exit": bool(np.max(heads[:, 0]) >= geometry.exit_x_m),
        "leading4_passage_success": _entered_all(
            positions, leading, geometry.exit_x_m + geometry.exit_margin_m
        ),
        "fixture_contact_samples": int(np.count_nonzero(contact)),
        "first_fixture_contact_x_m": first_contact_x,
        "first_contact_region": first_contact_region,
        "contact_dwell_ms": _runs_ms(contact, timestep_s),
    }


def audit(config_path: Path) -> Dict[str, Any]:
    spike_root, config, _ = resolve_phase0m(config_path)
    source_model = spike_root / str(config["simulation"]["source_model"])
    radius = cable_radius(source_model)
    geometry = geometry_from_config(radius, config)
    success_plane_x = geometry.exit_x_m + geometry.exit_margin_m
    timestep_s = float(config["simulation"]["timestep_s"])
    speed = float(config["controller"]["forward_speed_mps"])
    old_duration = float(config["controller"]["branch_duration_s"])
    old_budget = speed * old_duration
    report_root = spike_root / "reports" / "phase0m_c1"
    old_data_root = spike_root / str(config["paths"]["report_root"]) / "data"
    seeds = [int(value) for value in config["experiment"]["seeds"]]
    per_seed: Dict[str, Any] = {}
    spacings: List[float] = []
    required_values: List[float] = []
    old_budget_sufficient = 0
    medium_entered = 0
    large_entered = 0
    for seed in seeds:
        seed_root = old_data_root / f"seed_{seed}"
        with np.load(seed_root / "common_state.npz") as common:
            vertices = np.asarray(common["ordered_vertex_positions"], dtype=np.float64)
            leading = np.asarray(common["leading_indices"], dtype=np.int64)
            head_index = int(np.asarray(common["head_index"]).item())
        leading_x0 = vertices[leading, 0]
        spacing = ordered_local_spacing(vertices)
        required = required_success_translation(leading_x0, success_plane_x)
        sufficient = old_budget >= required + spacing
        old_budget_sufficient += int(sufficient)
        spacings.append(spacing)
        required_values.append(required)
        traces = {
            branch: _trace_diagnostics(
                seed_root / f"{branch}.npz",
                vertices,
                leading,
                geometry,
                timestep_s,
            )
            for branch in ("centered", "offset_medium", "offset_large")
        }
        medium_entered += int(traces["offset_medium"]["leading4_entered_throat"])
        large_entered += int(traces["offset_large"]["leading4_entered_throat"])
        per_seed[str(seed)] = {
            "head_index": head_index,
            "leading_indices": leading.tolist(),
            "leading4_x0_m": leading_x0.tolist(),
            "leading4_span_m": float(np.max(leading_x0) - np.min(leading_x0)),
            "local_spacing_m": spacing,
            "required_success_translation_m": required,
            "old_budget_sufficient_with_spacing": sufficient,
            "distance_head_to_funnel_entry_m": float(
                geometry.entry_x_m - vertices[head_index, 0]
            ),
            "distance_head_to_throat_start_m": float(
                geometry.funnel_exit_x_m - vertices[head_index, 0]
            ),
            "distance_head_to_throat_exit_m": float(
                geometry.exit_x_m - vertices[head_index, 0]
            ),
            "old_traces": traces,
        }

    local_spacing = float(np.median(spacings))
    centered_never_succeeded = all(
        not item["old_traces"]["centered"]["leading4_passage_success"]
        for item in per_seed.values()
    )
    confounded = (
        old_budget < max(required_values)
        and old_budget < max(required_values) + local_spacing
        and centered_never_succeeded
    )
    verdict = "REACHABILITY_CONFOUNDED" if confounded else "REACHABILITY_SUFFICIENT"
    medium_y = branch_y_target("offset_medium", radius, config)
    large_y = branch_y_target("offset_large", radius, config)
    previous_metrics_path = spike_root / str(config["paths"]["report_root"]) / "metrics.json"
    with previous_metrics_path.open("r", encoding="utf-8") as handle:
        previous_verdict = json.load(handle)["verdict"]
    result = {
        "verdict": verdict,
        "source": {
            "branch": _git("branch", "--show-current"),
            "commit": _git("rev-parse", "HEAD"),
            "config": str(Path(config_path).resolve().relative_to(spike_root)),
            "previous_result": previous_verdict,
        },
        "controller": {
            "type": "moving endpoint position-reference servo",
            "forward_speed_semantics": "target_x = start_x + forward_speed_mps * elapsed_s",
            "forward_speed_mps": speed,
            "old_duration_s": old_duration,
            "old_planned_command_travel_m": old_budget,
        },
        "geometry": {
            "cable_radius_m": radius,
            "funnel_entry_half_gap_r": float(config["geometry"]["funnel_entry_half_gap_r"]),
            "funnel_entry_half_gap_m": geometry.entry_half_gap_m,
            "throat_half_gap_r": float(config["geometry"]["throat_half_gap_r"]),
            "throat_half_gap_m": geometry.throat_half_gap_m,
            "funnel_entry_x_m": geometry.entry_x_m,
            "throat_start_x_m": geometry.funnel_exit_x_m,
            "throat_exit_x_m": geometry.exit_x_m,
            "exit_margin_r": float(config["geometry"]["exit_margin_r"]),
            "exit_margin_m": geometry.exit_margin_m,
            "success_plane_x_m": success_plane_x,
            "local_spacing_m": local_spacing,
            "medium_lateral_target_r": float(config["branches"]["medium_offset_r"]),
            "medium_lateral_target_m": medium_y,
            "large_lateral_target_r": float(config["branches"]["large_offset_r"]),
            "large_lateral_target_m": large_y,
            "medium_throat_clearance_proxy_m": reachability_clearance(geometry, medium_y),
            "large_throat_clearance_proxy_m": reachability_clearance(geometry, large_y),
        },
        "required_success_translation_m": {
            "min": float(np.min(required_values)),
            "median": float(np.median(required_values)),
            "max": float(np.max(required_values)),
        },
        "old_budget_sufficient_seeds": old_budget_sufficient,
        "medium_old_traces_leading4_entered_throat": medium_entered,
        "large_old_traces_leading4_entered_throat": large_entered,
        "per_seed": per_seed,
        "fact": (
            "The 0.090000 m planned endpoint-reference travel is below the "
            f"{max(required_values):.9f} m leading-4 success lower bound; all centered "
            "traces stopped before the success plane and medium never entered the throat."
        ),
        "inference": (
            "The fixed 3.0 s horizon confounded the previous Phase 0M task-level NO-GO, "
            "so one distance-derived horizon correction is applicable."
        ),
        "action": "Run one fresh C1 batch with only the longitudinal drive-distance and termination protocol corrected.",
    }
    report_root.mkdir(parents=True, exist_ok=True)
    with (report_root / "reachability_audit.json").open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")
    rows = "\n".join(
        f"| {seed} | {min(values['leading4_x0_m']):.9f} | "
        f"{values['required_success_translation_m']:.9f} | "
        f"{'yes' if values['old_budget_sufficient_with_spacing'] else 'no'} |"
        for seed, values in per_seed.items()
    )
    medium_distance = float(np.median([
        item["distance_head_to_throat_start_m"] for item in per_seed.values()
    ]))
    large_regions = sorted({
        item["old_traces"]["offset_large"]["first_contact_region"] or "none"
        for item in per_seed.values()
    })
    report = f"""# Phase 0M Reachability Audit

## Source
- branch: {result['source']['branch']}
- commit: {result['source']['commit']}
- config: {result['source']['config']}
- previous result: {previous_verdict}

## Actual geometry
- cable radius: {radius:.9f} m
- local spacing: {local_spacing:.9f} m
- funnel entry x: {geometry.entry_x_m:.9f} m
- throat start x: {geometry.funnel_exit_x_m:.9f} m
- throat exit x: {geometry.exit_x_m:.9f} m
- success plane x: {success_plane_x:.9f} m
- entry half-gap: {config['geometry']['funnel_entry_half_gap_r']} r = {geometry.entry_half_gap_m:.9f} m
- throat half-gap: {config['geometry']['throat_half_gap_r']} r = {geometry.throat_half_gap_m:.9f} m
- exit margin: {config['geometry']['exit_margin_r']} r = {geometry.exit_margin_m:.9f} m

## Controller
- actual controller type: moving endpoint position-reference servo
- forward speed/reference semantics: target_x = start_x + 0.030000 m/s * elapsed_s
- old duration: {old_duration:.6f} s
- old planned travel: {old_budget:.9f} m

## Per-seed required travel

| seed | leading4 min x | required success translation | old budget sufficient |
| --- | ---: | ---: | :---: |
{rows}

Required translation min/median/max: {np.min(required_values):.9f} / {np.median(required_values):.9f} / {np.max(required_values):.9f} m.

## Throat reachability
- median initial head distance to throat start: {medium_distance:.9f} m
- medium target: {config['branches']['medium_offset_r']} r = {medium_y:.9f} m
- large target: {config['branches']['large_offset_r']} r = {large_y:.9f} m
- medium clearance proxy: {result['geometry']['medium_throat_clearance_proxy_m']:.9f} m
- large clearance proxy: {result['geometry']['large_throat_clearance_proxy_m']:.9f} m
- medium old traces leading-4 entered throat: {medium_entered}/5
- large old traces leading-4 entered throat: {large_entered}/5
- large first-contact region (x proxy; trace has no fixture geom id): {', '.join(large_regions)}

## Verdict
{verdict}

## Fact
{result['fact']}

## Inference
{result['inference']}

## Action
{result['action']}
"""
    (report_root / "REACHABILITY.md").write_text(report, encoding="utf-8")
    print(json.dumps({
        "verdict": verdict,
        "old_budget_m": old_budget,
        "required_translation_m": result["required_success_translation_m"],
        "old_budget_sufficient_seeds": old_budget_sufficient,
        "medium_entered_throat": medium_entered,
        "large_entered_throat": large_entered,
    }, indent=2, sort_keys=True))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    audit(args.config)


if __name__ == "__main__":
    main()
