from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping


def _format(value: Any) -> str:
    if isinstance(value, float):
        if math.isnan(value):
            return "n/a"
        return f"{value:.6g}"
    return str(value)


def _interpretation(result: Mapping[str, Any], config: Mapping[str, Any]):
    verdict = result["verdict"]
    aggregate = result["aggregate"]
    fact = (
        f"{aggregate['completed_pairs']} fixed-seed pairs completed; "
        f"{aggregate['valid_pairs']} passed every pair gate and "
        f"{aggregate['regime_contrast_pairs']} showed observed friction-response contrast."
    )
    correction = config["experiment"].get("single_allowed_correction", {})
    if correction.get("applied"):
        fact += (
            " This is the single permitted correction run: hidden lateral "
            f"friction changed from {correction['previous_hidden_lateral_friction']} "
            f"to {config['friction']['hidden_lateral_friction']} after the initial "
            "five-pair pilot had matched initial states but no observed regime contrast."
        )
    if verdict == "PHASE0A_SMOKE_GO":
        inference = (
            "Under this controlled simulator intervention, the contact regime "
            "contains predictive information beyond the matched current state and repeat noise."
        )
        scientific = (
            "This supports testing a small oracle-mode predictor; it does not establish "
            "that Switching Log-NCDE is the right model."
        )
        next_action = (
            "Run a tiny no-mode predictor versus oracle-mode predictor on the frozen Phase 0A dataset."
        )
    elif verdict == "PHASE0A_SMOKE_WEAK":
        inference = (
            "The intervention produced some consistent signal, but the five-pair smoke gate was not met."
        )
        scientific = "The evidence is insufficient to begin complex model development."
        next_action = "Run one exact replication with the same configuration and fixed seeds."
    elif verdict == "PHASE0A_SMOKE_NO_GO":
        inference = (
            "The configured hidden-friction intervention did not produce future divergence "
            "that was both regime-grounded and clearly above repeat noise."
        )
        scientific = "Do not proceed to NCDE, mode-classifier, MPC, or RL work."
        no_contrast = aggregate["regime_contrast_pairs"] == 0
        initial_friction = float(config["friction"]["hidden_lateral_friction"])
        if no_contrast and initial_friction == 1.2:
            next_action = (
                "Repeat the five fixed pairs once with hidden lateral friction increased to 2.0 and every other setting unchanged."
            )
        else:
            next_action = "Stop complex-model development for this contact-friction mechanism."
    else:
        inference = "The prescribed five-pair scientific gate cannot yet be evaluated."
        scientific = "No scientific conclusion is available from an incomplete engineering run."
        next_action = "Restore simulator execution and complete all five prescribed pairs."
    return fact, inference, scientific, next_action


def write_reports(
    result: Mapping[str, Any], config: Mapping[str, Any], repo_root: Path
) -> Path:
    report_root = repo_root / str(config["paths"]["report_root"])
    report_root.mkdir(parents=True, exist_ok=True)
    metrics_path = report_root / "metrics.json"
    with metrics_path.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True, allow_nan=True)
        handle.write("\n")

    aggregate = result["aggregate"]
    fact, inference, scientific, next_action = _interpretation(result, config)
    text = f"""# Phase 0A Smoke Result

## Verdict
{result['verdict']}

## Key metrics
- valid pairs: {aggregate['valid_pairs']} / {config['gate']['minimum_total_pairs']}
- median initial cross RMSE: {_format(aggregate['median_initial_cross_rmse'])} m
- median RMSE @100 ms: {_format(aggregate['median_future_cross_rmse_100ms'])} m
- median RMSE @250 ms: {_format(aggregate['median_future_cross_rmse_250ms'])} m
- median RMSE @500 ms: {_format(aggregate['median_future_cross_rmse_500ms'])} m
- median repeat RMSE @500 ms: {_format(aggregate['median_future_repeat_rmse_500ms'])} m
- median cross/repeat ratio @500 ms: {_format(aggregate['median_cross_to_repeat_ratio_500ms'])}
- free median contact speed: {_format(aggregate['median_free_contact_speed_mps'])} m/s
- high-friction median contact speed: {_format(aggregate['median_high_contact_speed_mps'])} m/s
- free median contact force: {_format(aggregate['median_free_contact_force_n'])} N
- high-friction median contact force: {_format(aggregate['median_high_contact_force_n'])} N
- regime-contrast pairs: {aggregate['regime_contrast_pairs']}

## Fact
{fact}

## Inference
{inference}

## Scientific interpretation
{scientific}

## Next action
{next_action}
"""
    result_path = report_root / "RESULT.md"
    result_path.write_text(text, encoding="utf-8")
    return result_path
