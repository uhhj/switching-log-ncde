from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np

from .snapshot import cable_rmse


BRANCHES = ("free", "high_friction", "free_repeat")


def repeat_ratio(cross_rmse: float, repeat_rmse: float, floor: float) -> float:
    return float(cross_rmse) / max(float(repeat_rmse), float(floor))


def select_horizon_step(
    physics_steps: Sequence[int], onset_step: int, horizon_ms: float, hz: float
) -> int:
    steps = np.asarray(physics_steps, dtype=np.int64)
    steps = steps[steps >= int(onset_step)]
    if steps.size == 0:
        raise ValueError("no aligned sample at or after main-pull onset")
    target = float(onset_step) + float(horizon_ms) * float(hz) / 1000.0
    return int(steps[np.argmin(np.abs(steps.astype(np.float64) - target))])


def sustained_mask(
    mask: Sequence[bool],
    physics_steps: Sequence[int],
    minimum_duration_ms: float,
    hz: float,
) -> np.ndarray:
    values = np.asarray(mask, dtype=bool)
    steps = np.asarray(physics_steps, dtype=np.int64)
    if values.shape != steps.shape:
        raise ValueError("mask and physics_steps must have identical shapes")
    output = np.zeros(values.shape, dtype=bool)
    if values.size == 0:
        return output
    positive_diffs = np.diff(steps)
    positive_diffs = positive_diffs[positive_diffs > 0]
    stride = int(np.median(positive_diffs)) if positive_diffs.size else 1
    required_steps = int(np.ceil(float(minimum_duration_ms) * float(hz) / 1000.0))
    start = 0
    while start < values.size:
        if not values[start]:
            start += 1
            continue
        end = start + 1
        while end < values.size and values[end]:
            end += 1
        covered_steps = int(steps[end - 1] - steps[start] + stride)
        if covered_steps >= required_steps:
            output[start:end] = True
        start = end
    return output


def _load_pair(pair_root: Path) -> Dict[str, Dict[str, np.ndarray]]:
    loaded: Dict[str, Dict[str, np.ndarray]] = {}
    for branch in BRANCHES:
        path = pair_root / branch / "trajectory.npz"
        with np.load(path, allow_pickle=False) as archive:
            loaded[branch] = {name: archive[name] for name in archive.files}
    return loaded


def _aligned_steps(*branches: Mapping[str, np.ndarray]) -> np.ndarray:
    common = np.asarray(branches[0]["physics_step"], dtype=np.int64)
    for branch in branches[1:]:
        common = np.intersect1d(
            common, np.asarray(branch["physics_step"], dtype=np.int64)
        )
    return common


def _row_at(branch: Mapping[str, np.ndarray], step: int, field: str) -> np.ndarray:
    indices = np.flatnonzero(np.asarray(branch["physics_step"]) == int(step))
    if indices.size != 1:
        raise ValueError(f"expected exactly one row for physics step {step}")
    return np.asarray(branch[field][int(indices[0])])


def _main_pull(branch: Mapping[str, np.ndarray]) -> Dict[str, np.ndarray]:
    selected = np.asarray(branch["phase"]).astype(str) == "main_pull"
    return {name: np.asarray(value)[selected] for name, value in branch.items() if np.asarray(value).ndim > 0 and np.asarray(value).shape[0] == selected.shape[0]}


def _regime_summary(
    branch: Mapping[str, np.ndarray], analysis: Mapping[str, Any], hz: float
) -> Dict[str, Any]:
    main = _main_pull(branch)
    steps = np.asarray(main["physics_step"], dtype=np.int64)
    active = np.asarray(main["contact_active_beads"], dtype=np.int64) > 0
    speed = np.asarray(main["contact_mean_speed"], dtype=np.float64)
    force = np.asarray(main["contact_force_norm"], dtype=np.float64)
    stick = active & (speed <= float(analysis["stick_speed_max_mps"]))
    slip = active & (speed >= float(analysis["slip_speed_min_mps"]))
    minimum_ms = float(analysis["minimum_regime_duration_ms"])
    sustained_stick = sustained_mask(stick, steps, minimum_ms, hz)
    sustained_slip = sustained_mask(slip, steps, minimum_ms, hz)
    contact_speed = speed[active]
    contact_force = force[active]
    denominator = max(1, int(steps.size))
    return {
        "median_contact_speed_mps": float(np.median(contact_speed)) if contact_speed.size else 0.0,
        "median_contact_force_n": float(np.median(contact_force)) if contact_force.size else 0.0,
        "stick_fraction": float(np.count_nonzero(stick) / denominator),
        "slip_fraction": float(np.count_nonzero(slip) / denominator),
        "has_sustained_stick": bool(np.any(sustained_stick)),
        "has_sustained_slip": bool(np.any(sustained_slip)),
        "active_fraction": float(np.count_nonzero(active) / denominator),
    }


def pair_gate(metrics: Mapping[str, Any], config: Mapping[str, Any]) -> bool:
    analysis = config["analysis"]
    return bool(
        float(metrics["initial_cross_rmse"])
        <= float(analysis["initial_state_rmse_max_m"])
        and float(metrics["future_cross_rmse_500ms"])
        >= float(analysis["future_divergence_min_m"])
        and float(metrics["cross_to_repeat_ratio_500ms"])
        >= float(analysis["future_vs_repeat_multiplier"])
        and bool(metrics["friction_response_contrast"])
    )


def analyze_pair(
    pair_root: Path, config: Mapping[str, Any], seed: int
) -> Dict[str, Any]:
    branches = _load_pair(pair_root)
    free = branches["free"]
    high = branches["high_friction"]
    repeat = branches["free_repeat"]
    analysis = config["analysis"]
    hz = float(config["simulator"]["hz"])

    metrics: Dict[str, Any] = {
        "seed": int(seed),
        "initial_cross_rmse": cable_rmse(
            free["initial_bead_positions"], high["initial_bead_positions"]
        ),
        "initial_repeat_rmse": cable_rmse(
            free["initial_bead_positions"], repeat["initial_bead_positions"]
        ),
    }
    common = _aligned_steps(free, high, repeat)
    free_main = np.asarray(free["physics_step"])[
        np.asarray(free["phase"]).astype(str) == "main_pull"
    ]
    high_main = np.asarray(high["physics_step"])[
        np.asarray(high["phase"]).astype(str) == "main_pull"
    ]
    repeat_main = np.asarray(repeat["physics_step"])[
        np.asarray(repeat["phase"]).astype(str) == "main_pull"
    ]
    onset = int(max(np.min(free_main), np.min(high_main), np.min(repeat_main)))
    for horizon in analysis["horizons_ms"]:
        horizon_key = f"{int(horizon)}ms"
        step = select_horizon_step(common, onset, float(horizon), hz)
        cross = cable_rmse(
            _row_at(free, step, "bead_positions"),
            _row_at(high, step, "bead_positions"),
        )
        repeat_value = cable_rmse(
            _row_at(free, step, "bead_positions"),
            _row_at(repeat, step, "bead_positions"),
        )
        metrics[f"future_cross_rmse_{horizon_key}"] = cross
        metrics[f"future_repeat_rmse_{horizon_key}"] = repeat_value
        metrics[f"cross_to_repeat_ratio_{horizon_key}"] = repeat_ratio(
            cross, repeat_value, float(analysis["repeat_noise_floor_m"])
        )
        metrics[f"aligned_step_{horizon_key}"] = step

    free_regime = _regime_summary(free, analysis, hz)
    high_regime = _regime_summary(high, analysis, hz)
    for prefix, summary in (("free", free_regime), ("high", high_regime)):
        for name, value in summary.items():
            metrics[f"{prefix}_{name}"] = value

    sustained_contrast = bool(
        (
            free_regime["has_sustained_stick"]
            and high_regime["has_sustained_slip"]
        )
        or (
            free_regime["has_sustained_slip"]
            and high_regime["has_sustained_stick"]
        )
    )
    speed_gap = float(analysis["slip_speed_min_mps"]) - float(
        analysis["stick_speed_max_mps"]
    )
    continuous_speed_contrast = bool(
        free_regime["active_fraction"] > 0.0
        and high_regime["active_fraction"] > 0.0
        and abs(
            free_regime["median_contact_speed_mps"]
            - high_regime["median_contact_speed_mps"]
        )
        >= speed_gap
    )
    metrics["sustained_regime_contrast"] = sustained_contrast
    metrics["continuous_speed_contrast"] = continuous_speed_contrast
    metrics["friction_response_contrast"] = bool(
        sustained_contrast or continuous_speed_contrast
    )
    metrics["initial_matched"] = bool(
        metrics["initial_cross_rmse"]
        <= float(analysis["initial_state_rmse_max_m"])
    )
    metrics["valid_signal_pair"] = pair_gate(metrics, config)
    return metrics


def experiment_gate(
    pairs: Sequence[Mapping[str, Any]], config: Mapping[str, Any]
) -> str:
    minimum_total = int(config["gate"]["minimum_total_pairs"])
    minimum_valid = int(config["gate"]["minimum_valid_pairs"])
    if len(pairs) < minimum_total:
        return "PHASE0A_ENGINEERING_BLOCKED"
    valid = sum(bool(pair["valid_signal_pair"]) for pair in pairs)
    if valid >= minimum_valid:
        return "PHASE0A_SMOKE_GO"
    if valid > 0:
        return "PHASE0A_SMOKE_WEAK"
    analysis = config["analysis"]
    near = any(
        bool(pair["friction_response_contrast"])
        and float(pair["future_cross_rmse_500ms"])
        >= 0.75 * float(analysis["future_divergence_min_m"])
        and float(pair["cross_to_repeat_ratio_500ms"])
        >= 0.75 * float(analysis["future_vs_repeat_multiplier"])
        for pair in pairs
    )
    return "PHASE0A_SMOKE_WEAK" if near else "PHASE0A_SMOKE_NO_GO"


def _median(pairs: Sequence[Mapping[str, Any]], key: str) -> float:
    if not pairs:
        return float("nan")
    return float(np.median([float(pair[key]) for pair in pairs]))


def analyze_experiment(
    repo_root: Path, config: Mapping[str, Any]
) -> Dict[str, Any]:
    data_root = repo_root / str(config["paths"]["data_root"])
    pairs = []
    for seed in config["experiment"]["seeds"]:
        pair_root = data_root / f"pair_{int(seed)}"
        required = [pair_root / name / "trajectory.npz" for name in BRANCHES]
        if all(path.is_file() for path in required):
            pairs.append(analyze_pair(pair_root, config, int(seed)))
    verdict = experiment_gate(pairs, config)
    aggregate = {
        "completed_pairs": len(pairs),
        "valid_pairs": sum(bool(pair["valid_signal_pair"]) for pair in pairs),
        "regime_contrast_pairs": sum(
            bool(pair["friction_response_contrast"]) for pair in pairs
        ),
        "median_initial_cross_rmse": _median(pairs, "initial_cross_rmse"),
        "median_future_cross_rmse_100ms": _median(
            pairs, "future_cross_rmse_100ms"
        ),
        "median_future_cross_rmse_250ms": _median(
            pairs, "future_cross_rmse_250ms"
        ),
        "median_future_cross_rmse_500ms": _median(
            pairs, "future_cross_rmse_500ms"
        ),
        "median_future_repeat_rmse_500ms": _median(
            pairs, "future_repeat_rmse_500ms"
        ),
        "median_cross_to_repeat_ratio_500ms": _median(
            pairs, "cross_to_repeat_ratio_500ms"
        ),
        "median_free_contact_speed_mps": _median(
            pairs, "free_median_contact_speed_mps"
        ),
        "median_high_contact_speed_mps": _median(
            pairs, "high_median_contact_speed_mps"
        ),
        "median_free_contact_force_n": _median(
            pairs, "free_median_contact_force_n"
        ),
        "median_high_contact_force_n": _median(
            pairs, "high_median_contact_force_n"
        ),
    }
    return {"verdict": verdict, "aggregate": aggregate, "pairs": pairs}
