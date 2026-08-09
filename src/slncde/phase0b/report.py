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
    aggregate = result["aggregate"]
    verdict = result["verdict"]
    fact = (
        f"{aggregate['completed_seeds']} fixed seeds completed; "
        f"nominal succeeded in {aggregate['nominal_successful_branches']}, "
        f"slide contact occurred in {aggregate['slide_contact_branches']}, "
        f"jam contact occurred in {aggregate['jam_contact_branches']}, "
        f"sustained slip occurred in {aggregate['sustained_slip_branches']}, "
        f"and sustained jam occurred in {aggregate['sustained_jam_branches']}."
    )
    completed = {int(seed["seed"]) for seed in result.get("seeds", [])}
    missing = [
        int(seed)
        for seed in config["experiment"]["seeds"]
        if int(seed) not in completed
    ]
    if missing:
        fact += f" Prescribed results are missing for seeds {missing}."
    if verdict == "PHASE0B_FIXTURE_GO":
        inference = (
            "The task naturally realizes repeatable fixture-contact switching "
            "with sufficiently low same-condition drift."
        )
        scientific = (
            "The fixture task is qualified for an Oracle-mode necessity smoke; "
            "no predictive model has yet been validated."
        )
        next_action = "Run the Phase 0C matched-state/action Oracle-mode necessity smoke."
    elif verdict == "PHASE0B_FIXTURE_WEAK":
        inference = (
            "Repeat stability and fixture contact are adequate, but one intended "
            "natural mode occurs in only one or two fixed seeds."
        )
        scientific = (
            "The task is not yet qualified, but the protocol permits one geometry correction."
        )
        if aggregate["sustained_slip_branches"] < int(
            config["analysis"]["minimum_slip_seeds"]
        ):
            next_action = (
                "Increase slide lateral offset from 0.0035 m to 0.0055 m and rerun the same five seeds once."
            )
        else:
            next_action = (
                "Increase jam lateral offset from 0.0080 m to 0.0100 m and rerun the same five seeds once."
            )
    elif verdict == "PHASE0B_FIXTURE_NO_GO":
        inference = (
            "At least one core qualification gate failed, so branch intention "
            "cannot be treated as evidence of hybrid contact dynamics."
        )
        scientific = "Do not proceed to Phase 0C or any model training."
        if not aggregate["gates"]["A"]:
            next_action = (
                "Diagnose nominal/repeat snapshot and suction-constraint restoration before any geometry change."
            )
        else:
            next_action = "Stop Phase 0C work for this fixture configuration."
    else:
        inference = "The simulator pipeline did not produce all prescribed scientific data."
        scientific = "No fixture-task qualification conclusion is available."
        next_action = "Restore simulator execution and complete all five fixed seeds."
    return fact, inference, scientific, next_action


def write_reports(
    result: Mapping[str, Any], config: Mapping[str, Any], repo_root: Path
) -> Path:
    report_root = repo_root / str(config["paths"]["report_root"])
    report_root.mkdir(parents=True, exist_ok=True)
    with (report_root / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True, allow_nan=True)
        handle.write("\n")

    aggregate = result["aggregate"]
    fact, inference, scientific, next_action = _interpretation(result, config)
    fixture = config["fixture"]
    text = f"""# Phase 0B Fixture Task Qualification

## Verdict
{result['verdict']}

## Setup
- task: {config['simulator']['task_name']}
- fixture: two-wall channel, gap {_format(fixture['channel_gap_m'])} m, length {_format(fixture['channel_length_m'])} m
- seeds: {config['experiment']['seeds']}
- CPU-only: yes

## Repeat stability
- median repeat RMSE @100ms: {_format(aggregate['median_repeat_rmse_100ms'])} m
- median repeat RMSE @250ms: {_format(aggregate['median_repeat_rmse_250ms'])} m
- median repeat RMSE @500ms: {_format(aggregate['median_repeat_rmse_500ms'])} m
- seeds <= 2mm: {aggregate['repeat_seeds_le_2mm']} / 5

## Nominal
- successful nominal branches: {aggregate['nominal_successful_branches']} / 5
- median final progress: {_format(aggregate['median_nominal_final_progress_m'])} m
- nominal jam count: {aggregate['nominal_jam_count']} / 5

## Contact
- slide contact branches: {aggregate['slide_contact_branches']} / 5
- jam contact branches: {aggregate['jam_contact_branches']} / 5
- median slide contact duration: {_format(aggregate['median_slide_contact_duration_ms'])} ms
- median jam contact duration: {_format(aggregate['median_jam_contact_duration_ms'])} ms

## Mode realization
- sustained slip branches: {aggregate['sustained_slip_branches']} / 5
- sustained stick seeds: {aggregate['sustained_stick_seeds']} / 5
- sustained jam branches: {aggregate['sustained_jam_branches']} / 5
- representative slide sequence: {aggregate['representative_slide_sequence']}
- representative jam sequence: {aggregate['representative_jam_sequence']}

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
