from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from slncde.phase0a.metrics import select_horizon_step
from slncde.phase0a.snapshot import cable_rmse
from slncde.phase0b.labels import (
    compress_transitions,
    derive_labels,
    longest_true_run_samples,
    sustained_mask,
)


BRANCHES = ("nominal", "nominal_repeat", "slide_probe", "jam_probe")


def repeat_gate(values: Sequence[float], config: Mapping[str, Any]) -> bool:
    repeat = np.asarray(values, dtype=np.float64)
    if repeat.size < len(config["experiment"]["seeds"]):
        return False
    median_ok = float(np.median(repeat)) <= float(
        config["analysis"]["repeat_rmse_500ms_max_m"]
    )
    seeds_ok = int(np.count_nonzero(repeat <= 0.0020)) >= 4
    return bool(median_ok and seeds_ok)


def experiment_verdict(
    aggregate: Mapping[str, Any], config: Mapping[str, Any]
) -> str:
    if int(aggregate["completed_seeds"]) < len(config["experiment"]["seeds"]):
        return "PHASE0B_ENGINEERING_BLOCKED"
    gates = aggregate["gates"]
    if all(bool(gates[name]) for name in ("A", "B", "C", "D", "E", "F")):
        return "PHASE0B_FIXTURE_GO"
    slip_count = int(aggregate["sustained_slip_branches"])
    jam_count = int(aggregate["sustained_jam_branches"])
    weak_mode_count = (1 <= slip_count <= 2) or (1 <= jam_count <= 2)
    if gates["A"] and gates["B"] and gates["C"] and weak_mode_count:
        return "PHASE0B_FIXTURE_WEAK"
    return "PHASE0B_FIXTURE_NO_GO"


def _load_seed(seed_root: Path):
    with (seed_root / "metadata.json").open("r", encoding="utf-8") as handle:
        metadata = json.load(handle)
    branches = {}
    for branch in BRANCHES:
        with np.load(
            seed_root / branch / "trajectory.npz", allow_pickle=False
        ) as archive:
            branches[branch] = {name: archive[name] for name in archive.files}
    return metadata, branches


def _aligned_steps(*branches: Mapping[str, np.ndarray]) -> np.ndarray:
    common = np.asarray(branches[0]["physics_step"], dtype=np.int64)
    for branch in branches[1:]:
        common = np.intersect1d(
            common, np.asarray(branch["physics_step"], dtype=np.int64)
        )
    return common


def _row_at(branch: Mapping[str, np.ndarray], step: int) -> np.ndarray:
    index = np.flatnonzero(np.asarray(branch["physics_step"]) == int(step))
    if index.size != 1:
        raise ValueError(f"expected one trajectory row at physics step {step}")
    return np.asarray(branch["bead_positions"][int(index[0])])


def _repeat_metrics(
    nominal: Mapping[str, np.ndarray],
    repeat: Mapping[str, np.ndarray],
    config: Mapping[str, Any],
) -> Dict[str, float]:
    common = _aligned_steps(nominal, repeat)
    nominal_insertion = np.asarray(nominal["physics_step"])[
        np.asarray(nominal["phase"]).astype(str) == "insertion"
    ]
    repeat_insertion = np.asarray(repeat["physics_step"])[
        np.asarray(repeat["phase"]).astype(str) == "insertion"
    ]
    if nominal_insertion.size == 0 or repeat_insertion.size == 0:
        raise ValueError("nominal repeat lacks insertion samples")
    onset = int(max(np.min(nominal_insertion), np.min(repeat_insertion)))
    metrics = {}
    for horizon in config["analysis"]["horizons_ms"]:
        step = select_horizon_step(
            common,
            onset,
            float(horizon),
            float(config["simulator"]["hz"]),
        )
        metrics[f"repeat_rmse_{int(horizon)}ms"] = cable_rmse(
            _row_at(nominal, step), _row_at(repeat, step)
        )
        metrics[f"repeat_aligned_step_{int(horizon)}ms"] = int(step)
    return metrics


def _duration_ms(mask: np.ndarray, effective_trace_hz: float) -> float:
    return float(
        1000.0 * longest_true_run_samples(mask) / effective_trace_hz
    )


def _branch_summary(
    trace: Mapping[str, np.ndarray], config: Mapping[str, Any]
) -> Dict[str, Any]:
    derived = derive_labels(trace, config)
    effective_trace_hz = float(config["simulator"]["hz"]) / float(
        config["simulator"]["trace_stride"]
    )
    minimum_samples = int(derived["minimum_mode_samples"])
    contact_sustained = sustained_mask(derived["contact"], minimum_samples)
    slip_sustained = sustained_mask(derived["slip"], minimum_samples)
    stick_sustained = sustained_mask(derived["stick"], minimum_samples)
    insertion = np.asarray(trace["phase"]).astype(str) == "insertion"
    denominator = max(1, int(np.count_nonzero(insertion)))
    contact_force = np.asarray(trace["normal_force_sum"], dtype=np.float64)
    tangent_speed = np.asarray(
        trace["tangential_speed_mean"], dtype=np.float64
    )
    contact_values = derived["contact"]
    modes = np.asarray(derived["mode"]).astype(str)

    slip_indices = np.flatnonzero(slip_sustained)
    jam_indices = np.flatnonzero(derived["jam"])
    slide_transition = False
    if slip_indices.size:
        first = int(slip_indices[0])
        slide_transition = bool(
            np.any(modes[:first] == "free")
            and np.any(derived["touch"][: first + 1])
        )
    jam_transition = False
    if jam_indices.size:
        first = int(jam_indices[0])
        jam_transition = bool(
            np.any(modes[:first] == "free")
            and np.any(derived["touch"][: first + 1])
        )

    progress = np.asarray(trace["progress_m"], dtype=np.float64)
    progress_before_jam = (
        float(progress[int(jam_indices[0])]) if jam_indices.size else None
    )
    return {
        "motion_completed": bool(np.asarray(trace["motion_completed"]).item()),
        "final_progress_m": float(progress[-1]),
        "contact_duration_ms": _duration_ms(
            contact_sustained, effective_trace_hz
        ),
        "sustained_slip_duration_ms": _duration_ms(
            slip_sustained, effective_trace_hz
        ),
        "sustained_stick_duration_ms": _duration_ms(
            stick_sustained, effective_trace_hz
        ),
        "sustained_jam_duration_ms": _duration_ms(
            derived["jam"], effective_trace_hz
        ),
        "slip_fraction": float(
            np.count_nonzero(slip_sustained & insertion) / denominator
        ),
        "stick_fraction": float(
            np.count_nonzero(stick_sustained & insertion) / denominator
        ),
        "jam_fraction": float(
            np.count_nonzero(derived["jam"] & insertion) / denominator
        ),
        "max_normal_force_n": float(np.max(contact_force)),
        "median_normal_force_during_contact_n": (
            float(np.median(contact_force[contact_values]))
            if np.any(contact_values)
            else 0.0
        ),
        "median_tangent_speed_during_contact_mps": (
            float(np.median(tangent_speed[contact_values]))
            if np.any(contact_values)
            else 0.0
        ),
        "progress_before_jam_m": progress_before_jam,
        "transition_sequence": compress_transitions(modes),
        "free_contact_slip_transition": slide_transition,
        "free_contact_jam_transition": jam_transition,
    }


def analyze_seed(
    seed_root: Path, config: Mapping[str, Any], seed: int
) -> Dict[str, Any]:
    metadata, branches = _load_seed(seed_root)
    repeat = _repeat_metrics(
        branches["nominal"], branches["nominal_repeat"], config
    )
    summaries = {
        branch: _branch_summary(branches[branch], config) for branch in BRANCHES
    }
    nominal = summaries["nominal"]
    slide = summaries["slide_probe"]
    jam = summaries["jam_probe"]
    minimum_duration = float(config["labels"]["minimum_mode_duration_ms"])
    nominal_success = bool(
        nominal["final_progress_m"]
        >= float(config["analysis"]["nominal_progress_min_m"])
        and nominal["sustained_jam_duration_ms"] < minimum_duration
    )
    return {
        "seed": int(seed),
        **repeat,
        "nominal_motion_completed": nominal["motion_completed"],
        "nominal_final_progress_m": nominal["final_progress_m"],
        "nominal_jam_duration_ms": nominal["sustained_jam_duration_ms"],
        "nominal_success": nominal_success,
        "slide_contact_duration_ms": slide["contact_duration_ms"],
        "slide_contact_sustained": slide["contact_duration_ms"]
        >= minimum_duration,
        "slide_max_normal_force_n": slide["max_normal_force_n"],
        "slide_median_normal_force_n": slide[
            "median_normal_force_during_contact_n"
        ],
        "slide_slip_duration_ms": slide["sustained_slip_duration_ms"],
        "slide_slip_fraction": slide["slip_fraction"],
        "slide_stick_duration_ms": slide["sustained_stick_duration_ms"],
        "slide_median_tangent_speed_mps": slide[
            "median_tangent_speed_during_contact_mps"
        ],
        "slide_sequence": slide["transition_sequence"],
        "slide_meaningful_transition": slide[
            "free_contact_slip_transition"
        ],
        "jam_contact_duration_ms": jam["contact_duration_ms"],
        "jam_contact_sustained": jam["contact_duration_ms"]
        >= minimum_duration,
        "jam_duration_ms": jam["sustained_jam_duration_ms"],
        "jam_fraction": jam["jam_fraction"],
        "jam_normal_force_peak_n": jam["max_normal_force_n"],
        "jam_progress_before_jam_m": jam["progress_before_jam_m"],
        "jam_stick_duration_ms": jam["sustained_stick_duration_ms"],
        "jam_sequence": jam["transition_sequence"],
        "jam_meaningful_transition": jam["free_contact_jam_transition"],
        "sustained_stick": bool(
            slide["sustained_stick_duration_ms"] >= minimum_duration
            or jam["sustained_stick_duration_ms"] >= minimum_duration
        ),
        "meaningful_transition": bool(
            slide["free_contact_slip_transition"]
            and jam["free_contact_jam_transition"]
        ),
        "branch_motion_completed": {
            name: summary["motion_completed"]
            for name, summary in summaries.items()
        },
        "targets_identical": bool(
            metadata["branches"]["nominal"]["insert_pose_target"]
            == metadata["branches"]["nominal_repeat"]["insert_pose_target"]
        ),
    }


def _median(pairs: Sequence[Mapping[str, Any]], key: str) -> float:
    return float(np.median([float(pair[key]) for pair in pairs]))


def analyze_experiment(
    repo_root: Path, config: Mapping[str, Any]
) -> Dict[str, Any]:
    data_root = repo_root / str(config["paths"]["data_root"])
    pairs = []
    for seed in config["experiment"]["seeds"]:
        seed_root = data_root / f"seed_{int(seed)}"
        required = [seed_root / "metadata.json"] + [
            seed_root / branch / "trajectory.npz" for branch in BRANCHES
        ]
        if all(path.is_file() for path in required):
            pairs.append(analyze_seed(seed_root, config, int(seed)))

    minimum_duration = float(config["labels"]["minimum_mode_duration_ms"])
    repeat_500 = [pair["repeat_rmse_500ms"] for pair in pairs]
    slide_contact = sum(bool(pair["slide_contact_sustained"]) for pair in pairs)
    jam_contact = sum(bool(pair["jam_contact_sustained"]) for pair in pairs)
    nominal_success = sum(bool(pair["nominal_success"]) for pair in pairs)
    slip_count = sum(
        pair["slide_slip_duration_ms"] >= minimum_duration for pair in pairs
    )
    jam_count = sum(pair["jam_duration_ms"] >= minimum_duration for pair in pairs)
    stick_count = sum(bool(pair["sustained_stick"]) for pair in pairs)
    transition_count = sum(bool(pair["meaningful_transition"]) for pair in pairs)
    aggregate: Dict[str, Any] = {
        "completed_seeds": len(pairs),
        "median_repeat_rmse_100ms": _median(pairs, "repeat_rmse_100ms")
        if pairs
        else float("nan"),
        "median_repeat_rmse_250ms": _median(pairs, "repeat_rmse_250ms")
        if pairs
        else float("nan"),
        "median_repeat_rmse_500ms": _median(pairs, "repeat_rmse_500ms")
        if pairs
        else float("nan"),
        "repeat_seeds_le_2mm": int(
            np.count_nonzero(np.asarray(repeat_500) <= 0.0020)
        ),
        "nominal_successful_branches": nominal_success,
        "median_nominal_final_progress_m": _median(
            pairs, "nominal_final_progress_m"
        )
        if pairs
        else float("nan"),
        "nominal_jam_count": sum(
            pair["nominal_jam_duration_ms"] >= minimum_duration for pair in pairs
        ),
        "slide_contact_branches": slide_contact,
        "jam_contact_branches": jam_contact,
        "median_slide_contact_duration_ms": _median(
            pairs, "slide_contact_duration_ms"
        )
        if pairs
        else float("nan"),
        "median_jam_contact_duration_ms": _median(
            pairs, "jam_contact_duration_ms"
        )
        if pairs
        else float("nan"),
        "sustained_slip_branches": slip_count,
        "sustained_stick_seeds": stick_count,
        "sustained_jam_branches": jam_count,
        "meaningful_transition_seeds": transition_count,
        "representative_slide_sequence": (
            next(
                (
                    pair["slide_sequence"]
                    for pair in pairs
                    if pair["slide_slip_duration_ms"] >= minimum_duration
                ),
                pairs[0]["slide_sequence"] if pairs else "n/a",
            )
        ),
        "representative_jam_sequence": (
            next(
                (
                    pair["jam_sequence"]
                    for pair in pairs
                    if pair["jam_duration_ms"] >= minimum_duration
                ),
                pairs[0]["jam_sequence"] if pairs else "n/a",
            )
        ),
    }
    analysis = config["analysis"]
    aggregate["gates"] = {
        "A": repeat_gate(repeat_500, config),
        "B": nominal_success >= 4,
        "C": slide_contact >= int(analysis["minimum_contact_seeds"])
        and jam_contact >= int(analysis["minimum_contact_seeds"]),
        "D": slip_count >= int(analysis["minimum_slip_seeds"]),
        "E": jam_count >= int(analysis["minimum_jam_seeds"]),
        "F": transition_count >= int(analysis["minimum_transition_seeds"]),
    }
    verdict = experiment_verdict(aggregate, config)
    return {"verdict": verdict, "aggregate": aggregate, "seeds": pairs}
