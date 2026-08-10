from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import mujoco
import numpy as np

from .labels import derive_labels, sustained_mask
from .passage import cable_radius, geometry_from_config
from .phase0m import (
    BRANCHES,
    NON_REPEAT_BRANCHES,
    resolve_c1_drive_protocol,
    resolve_phase0m,
)
from .passage import PassageScene
from .probes import PROBES, resolve_setup


def longest_true_run(mask: Sequence[bool]) -> int:
    longest = current = 0
    for value in np.asarray(mask, dtype=bool):
        current = current + 1 if value else 0
        longest = max(longest, current)
    return int(longest)


def _episode_metrics(
    probe: str,
    trace: Mapping[str, np.ndarray],
    thresholds: Mapping[str, Any],
    hz: float,
) -> Dict[str, Any]:
    labels = derive_labels(trace, thresholds, hz)
    raw_contact = np.asarray(trace["contact_count"]) > 0
    contact_dwell_ms = 1000.0 * longest_true_run(raw_contact) / hz
    tangent = np.asarray(trace["tangent_velocity"], dtype=np.float64)[raw_contact]
    tangent_force = np.asarray(trace["tangent_force"], dtype=np.float64)[raw_contact]
    normal = np.asarray(trace["normal_force"], dtype=np.float64)[raw_contact]
    median_tangent = float(np.median(tangent)) if tangent.size else 0.0
    median_tangent_force = float(np.median(tangent_force)) if tangent_force.size else 0.0
    median_normal = float(np.median(normal)) if normal.size else 0.0
    endpoint_x = np.asarray(trace["endpoint_position"], dtype=np.float64)[:, 0]
    final_window = min(endpoint_x.size - 1, int(round(0.3 * hz)))
    final_displacement = abs(float(endpoint_x[-1] - endpoint_x[-1 - final_window]))
    driven_displacement = float(endpoint_x[-1] - endpoint_x[0])
    window = int(np.asarray(labels["jam_window_samples"]).item())
    eligible = raw_contact & (np.arange(raw_contact.size) >= window)
    jam_progress = (
        float(np.median(labels["progress_delta"][eligible]))
        if np.any(eligible)
        else float("inf")
    )
    minimum = int(np.asarray(labels["minimum_samples"]).item())
    minimum_dwell_ms = float(thresholds["minimum_mode_duration_ms"])
    stick_sustained = sustained_mask(labels["stick"], minimum)
    slip_sustained = sustained_mask(labels["slip"], minimum)

    if probe == "free":
        raw_pass = bool(np.mean(~raw_contact) >= 0.90)
        label_pass = bool(np.mean(labels["free"]) >= 0.90)
    elif probe == "stick":
        raw_pass = bool(
            contact_dwell_ms >= minimum_dwell_ms
            and median_tangent <= float(thresholds["stick_tangent_speed_max_mps"])
            and final_displacement <= 0.001
            and median_tangent_force > 1e-9
        )
        label_pass = longest_true_run(stick_sustained) >= minimum
    elif probe == "slip":
        raw_pass = bool(
            contact_dwell_ms >= minimum_dwell_ms
            and median_tangent >= float(thresholds["slip_tangent_speed_min_mps"])
            and driven_displacement >= 0.005
        )
        label_pass = longest_true_run(slip_sustained) >= minimum
    else:
        raw_pass = bool(
            contact_dwell_ms >= minimum_dwell_ms
            and median_normal >= float(thresholds["jam_normal_force_min_n"])
            and np.all(np.linalg.norm(trace["command"], axis=1) >= float(thresholds["command_active_speed_min_mps"]))
            and jam_progress <= float(thresholds["jam_progress_max_m"])
        )
        label_pass = longest_true_run(labels["jam"]) >= minimum
    return {
        "raw_pass": raw_pass,
        "label_pass": bool(label_pass),
        "contact_dwell_ms": contact_dwell_ms,
        "median_tangent_velocity_mps": median_tangent,
        "median_tangent_force_n": median_tangent_force,
        "median_normal_force_n": median_normal,
        "final_300ms_displacement_m": final_displacement,
        "driven_displacement_m": driven_displacement,
        "median_jam_window_progress_m": jam_progress,
    }


def _verdict(matrix: Mapping[str, Mapping[str, bool]]) -> str:
    if any(not bool(matrix[probe]["raw"]) for probe in PROBES):
        return "PHASE0S_MJ_FAIL"
    if any(not bool(matrix[probe]["label"]) for probe in PROBES):
        return "PHASE0S_MJ_LABEL_FAIL"
    return "PHASE0S_MJ_GO"


def analyze(config_path: Path) -> Dict[str, Any]:
    spike_root, config, thresholds = resolve_setup(config_path)
    report_root = spike_root / str(config["outputs"]["report_root"])
    hz = float(config["simulation"]["hz"])
    episodes = []
    grouped: Dict[str, list] = {probe: [] for probe in PROBES}
    for probe in PROBES:
        for path in sorted((report_root / "data").glob(f"{probe}_repeat_*.npz")):
            with np.load(path) as trace:
                result = _episode_metrics(probe, trace, thresholds, hz)
            result.update({"probe": probe, "file": path.name})
            episodes.append(result)
            grouped[probe].append(result)
    repeats = int(config["experiment"]["repeats"])
    if any(len(grouped[probe]) != repeats for probe in PROBES):
        raise RuntimeError("capability data is incomplete")
    regime_metrics: Dict[str, Any] = {}
    matrix: Dict[str, Dict[str, bool]] = {}
    for probe in PROBES:
        items = grouped[probe]
        raw_count = sum(bool(item["raw_pass"]) for item in items)
        label_count = sum(bool(item["label_pass"]) for item in items)
        regime_metrics[probe] = {
            "raw_pass_count": raw_count,
            "label_pass_count": label_count,
            "repeats": repeats,
            "median_contact_dwell_ms": float(np.median([item["contact_dwell_ms"] for item in items])),
            "median_tangent_velocity_mps": float(np.median([item["median_tangent_velocity_mps"] for item in items])),
            "median_tangent_force_n": float(np.median([item["median_tangent_force_n"] for item in items])),
            "median_normal_force_n": float(np.median([item["median_normal_force_n"] for item in items])),
            "median_final_300ms_displacement_m": float(np.median([item["final_300ms_displacement_m"] for item in items])),
            "median_driven_displacement_m": float(np.median([item["driven_displacement_m"] for item in items])),
            "median_jam_window_progress_m": float(np.median([item["median_jam_window_progress_m"] for item in items])),
        }
        matrix[probe] = {"raw": raw_count >= 2, "label": label_count >= 2}
    metrics = {
        "verdict": _verdict(matrix),
        "mujoco_version": mujoco.__version__,
        "cpu_only": True,
        "episodes_completed": len(episodes),
        "thresholds": thresholds,
        "regimes": regime_metrics,
        "capability_matrix": matrix,
        "episodes": episodes,
    }
    report_root.mkdir(parents=True, exist_ok=True)
    with (report_root / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")
    _write_report(report_root / "RESULT.md", metrics)
    return metrics


def _write_report(path: Path, metrics: Mapping[str, Any]) -> None:
    matrix = metrics["capability_matrix"]
    status = lambda value: "PASS" if value else "FAIL"
    if metrics["verdict"] == "PHASE0S_MJ_GO":
        fact = "MuJoCo flex produced stable free, stick, slip, and jam regimes, and the frozen Phase 0C0 labels recognized them."
        inference = "The MuJoCo 1D flex representation is capable of supporting the contact modes required by the later DLO switching-dynamics study."
        scientific = "This representation-only spike supports advancing MuJoCo flex to a benchmark-design phase without carrying forward DeformableRavens as a second main simulator."
        next_action = "Design one minimal MuJoCo flex constrained-passage benchmark while keeping the Phase 0C0 thresholds frozen."
    elif metrics["verdict"] == "PHASE0S_MJ_LABEL_FAIL":
        fact = "Raw MuJoCo regimes formed, but at least one frozen label adapter result failed."
        inference = "The simulator representation is capable; only the MuJoCo-to-Oracle adapter is limiting."
        scientific = "Task or material expansion is premature until the label adapter is corrected."
        next_action = "Fix only the failed MuJoCo label adapter and rerun this same spike."
    else:
        fact = "At least one required raw contact regime did not form in the minimal MuJoCo flex scene."
        inference = "This MuJoCo flex realization does not demonstrate the required representation capability."
        scientific = "The representation should not be promoted to the main DLO simulator path."
        next_action = "Stop this MuJoCo flex representation and evaluate the next simulator candidate."
    rows = "\n".join(
        f"| {probe} | {status(matrix[probe]['raw'])} | {status(matrix[probe]['label'])} |"
        for probe in PROBES
    )
    text = f"""# Phase 0S-MJ MuJoCo Flex Capability Spike

## Verdict
{metrics['verdict']}

## Setup
- MuJoCo: {metrics['mujoco_version']}
- CPU-only: yes
- episodes: {metrics['episodes_completed']}
- representation: 1D flex cable
- direct endpoint force: yes
- robot/camera/policy: none
- thresholds: frozen Phase 0C0/T2 values

## Capability matrix
| Regime | Raw physics | Label |
|---|---|---|
{rows}

## Fact
{fact}

## Inference
{inference}

## Scientific interpretation
{scientific}

## Next action
{next_action}
"""
    path.write_text(text, encoding="utf-8")


def trajectory_rmse_at_ms(
    positions_a: np.ndarray,
    positions_b: np.ndarray,
    hz: float,
    horizon_ms: float,
) -> float:
    first = np.asarray(positions_a, dtype=np.float64)
    second = np.asarray(positions_b, dtype=np.float64)
    if first.ndim != 3 or second.ndim != 3:
        raise ValueError("ordered vertex trajectories must have shape (T, V, 3)")
    index = max(0, int(round(float(horizon_ms) * float(hz) / 1000.0)) - 1)
    index = min(index, first.shape[0] - 1, second.shape[0] - 1)
    return float(np.sqrt(np.mean((first[index] - second[index]) ** 2)))


def phase0m_verdict(
    preparation_pass_count: int,
    preparation_required: int,
    gates: Mapping[str, bool],
) -> str:
    if int(preparation_pass_count) < int(preparation_required):
        return "PHASE0M_ENGINEERING_BLOCKED"
    if all(bool(value) for value in gates.values()):
        return "PHASE0M_GO"
    return "PHASE0M_NO_GO"


def _compress(values: Sequence[str]) -> str:
    output = []
    for value in np.asarray(values).astype(str):
        if not output or output[-1] != value:
            output.append(value)
    if len(output) <= 24:
        return " -> ".join(output)
    return (
        " -> ".join(output[:10])
        + f" -> ... ({len(output)} transitions total) ... -> "
        + " -> ".join(output[-10:])
    )


def _positive_median(values: Sequence[float]) -> float:
    positive = [float(value) for value in values if float(value) > 0.0]
    return float(np.median(positive)) if positive else 0.0


def _phase0m_trajectory_summary(
    trace: Mapping[str, np.ndarray], thresholds: Mapping[str, Any], hz: float
) -> Dict[str, Any]:
    minimum_samples = max(
        1,
        int(
            np.ceil(
                float(thresholds["minimum_mode_duration_ms"])
                * float(hz)
                / 1000.0
            )
        ),
    )
    contact = np.asarray(trace["contact_layer"]).astype(str) == "contact"
    stick = np.asarray(trace["friction_layer"]).astype(str) == "stick"
    slip = np.asarray(trace["friction_layer"]).astype(str) == "slip"
    jam = np.asarray(trace["failure_layer"]).astype(str) == "jam"
    contact_dwell = 1000.0 * longest_true_run(contact) / hz
    stick_dwell = 1000.0 * longest_true_run(stick) / hz
    slip_dwell = 1000.0 * longest_true_run(slip) / hz
    jam_dwell = 1000.0 * longest_true_run(jam) / hz
    funnel_contact = (
        np.asarray(trace["funnel_contact_count"], dtype=np.int64) > 0
        if "funnel_contact_count" in trace
        else np.zeros(contact.shape, dtype=bool)
    )
    throat_contact = (
        np.asarray(trace["throat_contact_count"], dtype=np.int64) > 0
        if "throat_contact_count" in trace
        else np.zeros(contact.shape, dtype=bool)
    )
    events = np.asarray(trace["events"], dtype=bool)
    event_names = np.asarray(trace["event_names"]).astype(str)
    event_counts = {
        name: int(np.count_nonzero(events[:, index]))
        for index, name in enumerate(event_names)
    }
    normal = np.asarray(trace["normal_force_sum"], dtype=np.float64)
    tangent_speed = np.asarray(
        trace["tangential_speed_mean"], dtype=np.float64
    )
    progress_delta = np.asarray(trace["progress_delta"], dtype=np.float64)
    return {
        "success": bool(np.asarray(trace["success"]).item()),
        "final_progress_m": float(np.asarray(trace["progress"])[-1]),
        "contact_present": bool(np.any(contact)),
        "contact_sustained": longest_true_run(contact) >= minimum_samples,
        "stick_sustained": longest_true_run(stick) >= minimum_samples,
        "slip_sustained": longest_true_run(slip) >= minimum_samples,
        "jam_sustained": longest_true_run(jam) >= minimum_samples,
        "contact_dwell_ms": contact_dwell,
        "stick_dwell_ms": stick_dwell,
        "slip_dwell_ms": slip_dwell,
        "jam_dwell_ms": jam_dwell,
        "funnel_contact_dwell_ms": 1000.0
        * longest_true_run(funnel_contact)
        / hz,
        "throat_contact_dwell_ms": 1000.0
        * longest_true_run(throat_contact)
        / hz,
        "contact_occupancy": float(np.mean(contact)),
        "stick_occupancy": float(np.mean(stick)),
        "slip_occupancy": float(np.mean(slip)),
        "jam_occupancy": float(np.mean(jam)),
        "median_tangent_speed_contact_mps": (
            float(np.median(tangent_speed[contact])) if np.any(contact) else 0.0
        ),
        "median_tangent_speed_slip_mps": (
            float(np.median(tangent_speed[slip])) if np.any(slip) else 0.0
        ),
        "median_normal_force_jam_n": (
            float(np.median(normal[jam])) if np.any(jam) else 0.0
        ),
        "median_jam_window_progress_m": (
            float(np.median(progress_delta[jam])) if np.any(jam) else 0.0
        ),
        "events": event_counts,
        "contact_sequence": _compress(trace["contact_layer"]),
        "friction_sequence": _compress(trace["friction_layer"]),
        "failure_sequence": _compress(trace["failure_layer"]),
        "samples": int(np.asarray(trace["time"]).size),
        **(
            {
                "required_translation_m": float(
                    np.asarray(trace["required_translation_m"]).item()
                ),
                "command_distance_m": float(
                    np.asarray(trace["command_distance_m"]).item()
                ),
                "max_duration_s": float(
                    np.asarray(trace["max_duration_s"]).item()
                ),
                "actual_reference_travel_m": float(
                    np.asarray(trace["actual_reference_travel_m"]).item()
                ),
                "actual_episode_duration_s": float(
                    np.asarray(trace["actual_episode_duration_s"]).item()
                ),
                "success_time_s": (
                    None
                    if np.isnan(float(np.asarray(trace["success_time_s"]).item()))
                    else float(np.asarray(trace["success_time_s"]).item())
                ),
                "head_entered_funnel": bool(
                    np.asarray(trace["head_entered_funnel"]).item()
                ),
                "head_entered_throat": bool(
                    np.asarray(trace["head_entered_throat"]).item()
                ),
                "leading4_entered_throat": bool(
                    np.asarray(trace["leading4_entered_throat"]).item()
                ),
                "head_crossed_exit": bool(
                    np.asarray(trace["head_crossed_exit"]).item()
                ),
                "leading4_passage_success": bool(
                    np.asarray(trace["leading4_passage_success"]).item()
                ),
            }
            if "command_distance_m" in trace
            else {}
        ),
    }


def analyze_phase0m(config_path: Path) -> Dict[str, Any]:
    spike_root, config, thresholds = resolve_phase0m(config_path)
    report_root = spike_root / str(config["paths"]["report_root"])
    data_root = report_root / "data"
    hz = 1.0 / float(config["simulation"]["timestep_s"])
    source_model = spike_root / str(config["simulation"]["source_model"])
    radius_m = cable_radius(source_model)
    geometry = geometry_from_config(radius_m, config)
    seeds = [int(value) for value in config["experiment"]["seeds"]]
    per_seed: Dict[str, Any] = {}
    preparations = []
    all_summaries = []
    repeat_values = {100: [], 250: [], 500: []}
    for seed in seeds:
        seed_root = data_root / f"seed_{seed}"
        common_path = seed_root / "common_state.npz"
        branch_paths = {branch: seed_root / f"{branch}.npz" for branch in BRANCHES}
        if not common_path.is_file() or not all(path.is_file() for path in branch_paths.values()):
            continue
        with np.load(common_path) as common:
            preparation = {
                "pass": bool(np.asarray(common["preparation_pass"]).item()),
                "max_node_speed_mps": float(
                    np.asarray(common["max_node_speed_mps"]).item()
                ),
                "fixture_contact_free": bool(
                    np.asarray(common["fixture_contact_free"]).item()
                ),
                "initial_y_m": float(np.asarray(common["initial_y_m"]).item()),
            }
        preparations.append(preparation)
        branch_summaries = {}
        traces = {}
        for branch, path in branch_paths.items():
            with np.load(path) as loaded:
                trace = {key: loaded[key] for key in loaded.files}
            traces[branch] = trace
            summary = _phase0m_trajectory_summary(trace, thresholds, hz)
            branch_summaries[branch] = summary
            all_summaries.append((seed, branch, summary))
        for horizon in repeat_values:
            repeat_values[horizon].append(
                trajectory_rmse_at_ms(
                    traces["centered"]["ordered_vertex_positions"],
                    traces["centered_repeat"]["ordered_vertex_positions"],
                    hz,
                    horizon,
                )
            )
        command_fields = (
            "command_forward",
            "command_lateral",
            "command_active",
        )
        commands_identical = all(
            np.array_equal(traces["centered"][key], traces["centered_repeat"][key])
            for key in command_fields
        )
        per_seed[str(seed)] = {
            "preparation": preparation,
            "branches": branch_summaries,
            "repeat_rmse_m": {
                str(horizon): repeat_values[horizon][-1]
                for horizon in repeat_values
            },
            "centered_repeat_commands_identical": commands_identical,
        }

    preparation_pass = sum(bool(item["pass"]) for item in preparations)
    centered = [item[2] for item in all_summaries if item[1] == "centered"]
    medium = [item[2] for item in all_summaries if item[1] == "offset_medium"]
    large = [item[2] for item in all_summaries if item[1] == "offset_large"]
    non_repeat = [item[2] for item in all_summaries if item[1] in NON_REPEAT_BRANCHES]
    centered_success = sum(bool(item["success"]) for item in centered)
    centered_contact = sum(bool(item["contact_present"]) for item in centered)
    medium_contact = sum(bool(item["contact_present"]) for item in medium)
    large_contact = sum(bool(item["contact_present"]) for item in large)
    medium_slip = sum(bool(item["slip_sustained"]) for item in medium)
    large_jam = sum(bool(item["jam_sustained"]) for item in large)
    stick_episodes = sum(bool(item["stick_sustained"]) for item in non_repeat)
    medium_touch = sum(item["events"].get("touch", 0) > 0 for item in medium)
    large_touch = sum(item["events"].get("touch", 0) > 0 for item in large)
    large_jam_onset = sum(item["events"].get("jam_onset", 0) > 0 for item in large)
    repeat_500 = repeat_values[500]
    gates_config = config["gates"]
    gates = {
        "repeat": bool(repeat_500)
        and float(np.median(repeat_500))
        <= float(gates_config["repeat_median_rmse_500ms_max_m"])
        and sum(
            value <= float(gates_config["repeat_seed_rmse_500ms_max_m"])
            for value in repeat_500
        )
        >= int(gates_config["repeat_seed_pass_count"]),
        "centered_passage": centered_success
        >= int(gates_config["centered_success_count"]),
        "medium_contact": medium_contact
        >= int(gates_config["medium_contact_count"]),
        "large_contact": large_contact
        >= int(gates_config["large_contact_count"]),
        "medium_slip": medium_slip >= int(gates_config["medium_slip_count"]),
        "large_jam": large_jam >= int(gates_config["large_jam_count"]),
        "dataset_stick": stick_episodes
        >= int(gates_config["dataset_stick_episode_count"]),
        "meaningful_transitions": medium_touch >= 3
        and medium_slip >= 3
        and large_touch >= 3
        and large_jam_onset >= 3,
    }
    event_totals = {
        name: int(
            sum(summary["events"].get(name, 0) for _, _, summary in all_summaries)
        )
        for name in (
            "touch",
            "release",
            "stick_to_slip",
            "slip_to_stick",
            "jam_onset",
            "jam_release",
        )
    }
    occupancy = {
        name: float(
            np.average(
                [summary[f"{name}_occupancy"] for _, _, summary in all_summaries],
                weights=[summary["samples"] for _, _, summary in all_summaries],
            )
        )
        if all_summaries
        else 0.0
        for name in ("contact", "stick", "slip", "jam")
    }
    episode_coverage = {
        "contact": int(
            sum(summary["contact_present"] for _, _, summary in all_summaries)
        ),
        **{
            name: int(
                sum(
                    summary[f"{name}_sustained"]
                    for _, _, summary in all_summaries
                )
            )
            for name in ("stick", "slip", "jam")
        },
    }
    dwell = {
        name: _positive_median(
            [summary[f"{name}_dwell_ms"] for _, _, summary in all_summaries]
        )
        for name in ("contact", "stick", "slip", "jam")
    }
    verdict = phase0m_verdict(
        preparation_pass,
        int(gates_config["preparation_required"]),
        gates,
    )
    representative = {
        "offset_medium": {
            key: medium[0][key] if medium else "n/a"
            for key in (
                "contact_sequence",
                "friction_sequence",
                "failure_sequence",
            )
        },
        "offset_large": {
            key: large[0][key] if large else "n/a"
            for key in (
                "contact_sequence",
                "friction_sequence",
                "failure_sequence",
            )
        },
    }
    metrics = {
        "verdict": verdict,
        "setup": {
            "mujoco_version": mujoco.__version__,
            "cpu_only": True,
            "seeds": seeds,
            "cable_radius_m": radius_m,
            "fixture_dimensions_m": {
                "entry_full_gap": 2.0 * geometry.entry_half_gap_m,
                "throat_full_gap": 2.0 * geometry.throat_half_gap_m,
                "funnel_length": geometry.funnel_exit_x_m
                - geometry.entry_x_m,
                "throat_length": geometry.exit_x_m
                - geometry.funnel_exit_x_m,
            },
            "frozen_oracle_source": config["oracle"]["source_config"],
            "completed_trajectories": len(all_summaries),
        },
        "preparation": {
            "pass_count": preparation_pass,
            "attempted": len(preparations),
            "median_max_node_speed_mps": float(
                np.median([item["max_node_speed_mps"] for item in preparations])
            )
            if preparations
            else 0.0,
        },
        "repeat": {
            "median_rmse_100ms_m": float(np.median(repeat_values[100])) if repeat_values[100] else 0.0,
            "median_rmse_250ms_m": float(np.median(repeat_values[250])) if repeat_values[250] else 0.0,
            "median_rmse_500ms_m": float(np.median(repeat_500)) if repeat_500 else 0.0,
            "seeds_le_1_5mm": sum(value <= 0.0015 for value in repeat_500),
        },
        "passage": {
            "centered_success_count": centered_success,
            "median_centered_final_progress_m": float(
                np.median([item["final_progress_m"] for item in centered])
            )
            if centered
            else 0.0,
            "leading4_exit_success": centered_success,
        },
        "contact": {
            "centered_count": centered_contact,
            "medium_count": medium_contact,
            "large_count": large_contact,
            "median_dwell_ms": dwell["contact"],
        },
        "friction": {
            "non_repeat_stick_episode_count": stick_episodes,
            "medium_slip_count": medium_slip,
            "median_stick_dwell_ms": dwell["stick"],
            "median_slip_dwell_ms": dwell["slip"],
            "median_medium_slip_tangent_speed_mps": _positive_median(
                [item["median_tangent_speed_slip_mps"] for item in medium]
            ),
        },
        "failure": {
            "large_jam_count": large_jam,
            "median_jam_dwell_ms": dwell["jam"],
            "median_large_jam_normal_force_n": _positive_median(
                [item["median_normal_force_jam_n"] for item in large]
            ),
            "median_large_jam_window_progress_m": float(
                np.median(
                    [
                        item["median_jam_window_progress_m"]
                        for item in large
                        if item["jam_sustained"]
                    ]
                )
            )
            if any(item["jam_sustained"] for item in large)
            else 0.0,
        },
        "events": event_totals,
        "representative_sequences": representative,
        "mode_coverage": {
            "occupancy": occupancy,
            "episode_coverage": episode_coverage,
            "median_dwell_ms": dwell,
        },
        "gates": gates,
        "per_seed": per_seed,
    }
    is_c1 = config.get("protocol", {}).get("mode") == "geometric_distance_budget"
    if is_c1:
        scene = PassageScene(source_model, config)
        protocol = resolve_c1_drive_protocol(spike_root, scene, config)
        centered_sufficient = sum(
            bool(item["success"])
            or item["actual_reference_travel_m"]
            >= item["command_distance_m"] - 1e-12
            for item in centered
        )
        medium_entered = sum(
            bool(item["leading4_entered_throat"]) for item in medium
        )
        large_entered = sum(
            bool(item["leading4_entered_throat"]) for item in large
        )
        protocol_reachability = {
            "centered_sufficient_travel_count": centered_sufficient,
            "medium_entered_throat_count": medium_entered,
            "large_entered_throat_count": large_entered,
            "centered_sufficient_travel_pass": centered_sufficient == len(seeds),
            "medium_entered_throat_pass": medium_entered >= 4,
            "large_entered_throat_pass": large_entered >= 4,
        }
        protocol_pass = all(
            value
            for key, value in protocol_reachability.items()
            if key.endswith("_pass")
        )
        if preparation_pass < int(gates_config["preparation_required"]):
            verdict = "PHASE0M_C1_ENGINEERING_BLOCKED"
        elif protocol_pass and all(gates.values()):
            verdict = "PHASE0M_C1_GO"
        elif protocol_pass:
            verdict = "PHASE0M_C1_TASK_NO_GO"
        else:
            verdict = "PHASE0M_C1_ENGINEERING_BLOCKED"
        with (spike_root / "reports" / "phase0m_c1" / "reachability_audit.json").open(
            "r", encoding="utf-8"
        ) as handle:
            reachability_audit = json.load(handle)
        with (spike_root / "reports" / "phase0m" / "metrics.json").open(
            "r", encoding="utf-8"
        ) as handle:
            old_metrics = json.load(handle)
        funnel_dwell = _positive_median(
            [item[2]["funnel_contact_dwell_ms"] for item in all_summaries]
        )
        throat_dwell = _positive_median(
            [item[2]["throat_contact_dwell_ms"] for item in all_summaries]
        )
        metrics.update(
            {
                "verdict": verdict,
                "reachability_audit": reachability_audit,
                "protocol": protocol,
                "protocol_reachability": protocol_reachability,
                "contact": {
                    **metrics["contact"],
                    "funnel_median_dwell_ms": funnel_dwell,
                    "throat_median_dwell_ms": throat_dwell,
                },
                "comparison_to_phase0m": {
                    "centered_success": {
                        "phase0m": old_metrics["passage"]["centered_success_count"],
                        "c1": centered_success,
                    },
                    "medium_contact": {
                        "phase0m": old_metrics["contact"]["medium_count"],
                        "c1": medium_contact,
                    },
                    "large_contact": {
                        "phase0m": old_metrics["contact"]["large_count"],
                        "c1": large_contact,
                    },
                    "stick": {
                        "phase0m": old_metrics["friction"][
                            "non_repeat_stick_episode_count"
                        ],
                        "c1": stick_episodes,
                    },
                    "slip": {
                        "phase0m": old_metrics["friction"]["medium_slip_count"],
                        "c1": medium_slip,
                    },
                    "jam": {
                        "phase0m": old_metrics["failure"]["large_jam_count"],
                        "c1": large_jam,
                    },
                },
            }
        )
    report_root.mkdir(parents=True, exist_ok=True)
    with (report_root / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")
    if is_c1:
        _write_phase0m_c1_report(report_root / "RESULT.md", metrics)
    else:
        _write_phase0m_report(report_root / "RESULT.md", metrics)
    return metrics


def _write_phase0m_c1_report(path: Path, metrics: Mapping[str, Any]) -> None:
    failed = ", ".join(
        name for name, value in metrics["gates"].items() if not value
    ) or "none"
    if metrics["verdict"] == "PHASE0M_C1_GO":
        fact = (
            "The distance-derived C1 protocol removed the old reachability confound "
            "and all original Phase 0M scientific gates passed."
        )
        inference = "The unified MuJoCo passage now supports the required contact regimes."
        scientific = (
            "The task is qualified for the matched-state/matched-action necessity audit, "
            "but not yet for dynamics-model training."
        )
        next_action = "Run the Phase 0M2 matched-state/matched-action mode-necessity audit."
    elif metrics["verdict"] == "PHASE0M_C1_ENGINEERING_BLOCKED":
        fact = "The corrected C1 pipeline did not establish all required protocol-reachability gates."
        inference = "The scientific task result cannot be interpreted from this incomplete correction."
        scientific = "This is an engineering qualification block, not a task-level scientific result."
        next_action = "Repair only the C1 runner or trace pipeline and rerun the same fixed protocol."
    else:
        fact = (
            "The old horizon was confounded; under the corrected distance-derived protocol, "
            f"all branches reached the intended region but these scientific gates failed: {failed}."
        )
        inference = (
            "The horizon confound is removed, so the remaining failure belongs to the frozen "
            "task, controller coupling, or contact realization."
        )
        scientific = (
            "The current passage does not naturally supply all required sustained regimes; "
            "contact chatter after throat entry is not merely a truncated approach."
        )
        next_action = "Choose one minimal task correction versus switching to SOFA BeamAdapter."
    protocol = metrics["protocol"]
    preparation = metrics["preparation"]
    reach = metrics["protocol_reachability"]
    repeat = metrics["repeat"]
    passage = metrics["passage"]
    contact = metrics["contact"]
    friction = metrics["friction"]
    failure = metrics["failure"]
    events = metrics["events"]
    coverage = metrics["mode_coverage"]
    comparison = metrics["comparison_to_phase0m"]
    text = f"""# Phase 0M-C1 Reachable-Horizon Correction

## Verdict
{metrics['verdict']}

## Protocol correction
- old command budget: {protocol['old_command_budget_m']:.9f} m
- required geometric translation: {protocol['required_translation_m']:.9f} m
- new command distance: {protocol['command_distance_m']:.9f} m
- new nominal drive time: {protocol['nominal_drive_time_s']:.9f} s
- new max duration: {protocol['max_duration_s']:.9f} s
- only changed variable: longitudinal command distance / drive horizon and its termination logic

## Preparation
- PASS: {preparation['pass_count']}/5

## Protocol reachability
- centered sufficient travel: {reach['centered_sufficient_travel_count']}/5
- medium entered throat: {reach['medium_entered_throat_count']}/5
- large entered throat: {reach['large_entered_throat_count']}/5

## Repeat
- RMSE @100 ms: {repeat['median_rmse_100ms_m']:.9f} m
- RMSE @250 ms: {repeat['median_rmse_250ms_m']:.9f} m
- RMSE @500 ms: {repeat['median_rmse_500ms_m']:.9f} m
- seeds <=1.5 mm: {repeat['seeds_le_1_5mm']}/5

## Passage
- centered success: {passage['centered_success_count']}/5
- median centered progress: {passage['median_centered_final_progress_m']:.9f} m

## Contact
- centered contact: {contact['centered_count']}/5
- medium contact: {contact['medium_count']}/5
- large contact: {contact['large_count']}/5
- median contact dwell: {contact['median_dwell_ms']:.3f} ms

## Friction
- sustained stick episodes: {friction['non_repeat_stick_episode_count']}/15 non-repeat
- medium sustained slip: {friction['medium_slip_count']}/5
- median stick dwell: {friction['median_stick_dwell_ms']:.3f} ms
- median slip dwell: {friction['median_slip_dwell_ms']:.3f} ms

## Failure
- large sustained jam: {failure['large_jam_count']}/5
- median jam dwell: {failure['median_jam_dwell_ms']:.3f} ms
- median jam normal force: {failure['median_large_jam_normal_force_n']:.9f} N
- median jam-window progress: {failure['median_large_jam_window_progress_m']:.9f} m

## Events
- touch: {events['touch']}
- release: {events['release']}
- stick->slip: {events['stick_to_slip']}
- slip->stick: {events['slip_to_stick']}
- jam onset: {events['jam_onset']}
- jam release: {events['jam_release']}

## Region dwell
- funnel contact dwell: {contact['funnel_median_dwell_ms']:.3f} ms
- throat contact dwell: {contact['throat_median_dwell_ms']:.3f} ms

## Mode coverage
- occupancy: {coverage['occupancy']}
- episode coverage: {coverage['episode_coverage']}
- dwell: {coverage['median_dwell_ms']}

## Phase0M vs C1
- centered success: {comparison['centered_success']['phase0m']} -> {comparison['centered_success']['c1']}
- medium contact: {comparison['medium_contact']['phase0m']} -> {comparison['medium_contact']['c1']}
- large contact: {comparison['large_contact']['phase0m']} -> {comparison['large_contact']['c1']}
- stick: {comparison['stick']['phase0m']} -> {comparison['stick']['c1']}
- slip: {comparison['slip']['phase0m']} -> {comparison['slip']['c1']}
- jam: {comparison['jam']['phase0m']} -> {comparison['jam']['c1']}

## Fact
{fact}

## Inference
{inference}

## Scientific interpretation
{scientific}

## Next action
{next_action}
"""
    path.write_text(text, encoding="utf-8")


def _write_phase0m_report(path: Path, metrics: Mapping[str, Any]) -> None:
    if metrics["verdict"] == "PHASE0M_GO":
        fact = "The single MuJoCo flex funnel-and-throat task passed preparation, repeat, passage, contact, friction, jam, and transition gates."
        inference = "One unified task naturally and reproducibly realizes the hierarchical hybrid regimes required for the scientific benchmark."
        scientific = "The benchmark is qualified for a matched-state/matched-action mode-necessity audit, but not yet for model training."
        next_action = "Run Phase 0M2 matched-state/matched-action mode-necessity audit."
    elif metrics["verdict"] == "PHASE0M_ENGINEERING_BLOCKED":
        fact = "The canonical common-state preparation gate failed before scientific mode coverage could be interpreted."
        inference = "The current result is an engineering qualification failure rather than evidence about unified-task contact regimes."
        scientific = "The passage benchmark cannot be promoted or rejected scientifically from this batch."
        next_action = "Repair only the common-state preparation pipeline and rerun the same Phase 0M batch."
    else:
        failed = ", ".join(
            name for name, value in metrics["gates"].items() if not value
        )
        fact = f"Preparation completed, but the following qualification gates failed: {failed}."
        inference = "The first unified MuJoCo flex passage task does not yet provide the required natural and repeatable regime coverage."
        scientific = "This is a task-level NO-GO; no geometry, controller, friction, or Oracle parameter was tuned after the formal batch began."
        next_action = "Review the Phase 0M contact, progress, and mode traces to choose between one minimal task correction and SOFA BeamAdapter."
    setup = metrics["setup"]
    repeat = metrics["repeat"]
    passage = metrics["passage"]
    contact = metrics["contact"]
    friction = metrics["friction"]
    failure = metrics["failure"]
    events = metrics["events"]
    sequences = metrics["representative_sequences"]
    coverage = metrics["mode_coverage"]
    text = f"""# Phase 0M MuJoCo Flex Constrained-Passage Qualification

## Verdict
{metrics['verdict']}

## Setup
- MuJoCo version: {setup['mujoco_version']}
- CPU-only: yes
- cable radius: {setup['cable_radius_m']} m
- fixture dimensions in meters: {setup['fixture_dimensions_m']}
- frozen Oracle source: {setup['frozen_oracle_source']}
- seeds: {setup['seeds']}

## Preparation
- PASS {metrics['preparation']['pass_count']}/5

## Repeat
- RMSE @100 ms: {repeat['median_rmse_100ms_m']:.9f} m
- RMSE @250 ms: {repeat['median_rmse_250ms_m']:.9f} m
- RMSE @500 ms: {repeat['median_rmse_500ms_m']:.9f} m
- per-seed threshold pass: {repeat['seeds_le_1_5mm']}/5

## Passage
- centered success: {passage['centered_success_count']}/5
- median progress: {passage['median_centered_final_progress_m']:.9f} m
- leading-4 exit success: {passage['leading4_exit_success']}/5

## Contact
- centered contact: {contact['centered_count']}/5
- medium contact: {contact['medium_count']}/5
- large contact: {contact['large_count']}/5
- contact dwell: {contact['median_dwell_ms']:.3f} ms

## Friction regimes
- sustained stick episodes: {friction['non_repeat_stick_episode_count']}/15 non-repeat
- medium sustained slip: {friction['medium_slip_count']}/5
- stick dwell: {friction['median_stick_dwell_ms']:.3f} ms
- slip dwell: {friction['median_slip_dwell_ms']:.3f} ms
- medium slip tangent speed: {friction['median_medium_slip_tangent_speed_mps']:.9f} m/s

## Failure regime
- large sustained jam: {failure['large_jam_count']}/5
- jam dwell: {failure['median_jam_dwell_ms']:.3f} ms
- normal force: {failure['median_large_jam_normal_force_n']:.9f} N
- jam-window progress: {failure['median_large_jam_window_progress_m']:.9f} m

## Events
- touch: {events['touch']}
- release: {events['release']}
- stick_to_slip: {events['stick_to_slip']}
- slip_to_stick: {events['slip_to_stick']}
- jam_onset: {events['jam_onset']}
- jam_release: {events['jam_release']}

## Representative sequences
- medium contact: {sequences['offset_medium']['contact_sequence']}
- medium friction: {sequences['offset_medium']['friction_sequence']}
- medium failure: {sequences['offset_medium']['failure_sequence']}
- large contact: {sequences['offset_large']['contact_sequence']}
- large friction: {sequences['offset_large']['friction_sequence']}
- large failure: {sequences['offset_large']['failure_sequence']}

## Mode coverage
- occupancy: {coverage['occupancy']}
- episode coverage: {coverage['episode_coverage']}
- dwell: {coverage['median_dwell_ms']}

## Fact
{fact}

## Inference
{inference}

## Scientific interpretation
{scientific}

## Next action
{next_action}
"""
    path.write_text(text, encoding="utf-8")
