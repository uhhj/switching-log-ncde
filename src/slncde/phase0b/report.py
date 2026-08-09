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


def _r1_interpretation(result: Mapping[str, Any]):
    aggregate = result["aggregate"]
    verdict = result["verdict"]
    fact = (
        f"Canonical preparation completed for "
        f"{aggregate['preparation_successful_seeds']} / 5 seeds; "
        f"{aggregate['contact_free_staging_seeds']} / 5 staging states were "
        f"fixture-contact-free. Repeat, nominal, contact, and mode counts are "
        f"reported from {aggregate['completed_seeds']} complete paired rollouts."
    )
    if verdict == "PHASE0B_R1_GO":
        inference = (
            "The canonical frame is seed-robust and preserves low repeat noise "
            "while realizing the required fixture-contact modes."
        )
        scientific = (
            "Phase 0B task qualification is complete; predictive mode necessity "
            "has not yet been tested."
        )
        next_action = "Run the Phase 0C matched-state/action Oracle-mode necessity smoke."
    elif verdict == "PHASE0B_R1_WEAK_JAM":
        inference = (
            "The canonical task is engineering-qualified for free/contact/slip, "
            "but the unchanged jam geometry is not strong enough."
        )
        scientific = (
            "The coordinate-frame failure is resolved; jam remains a separate "
            "geometry qualification question."
        )
        next_action = (
            "Increase jam_lateral_offset_m by 0.002 m in a separate Phase 0B-R2 run."
        )
    elif verdict == "PHASE0B_R1_NO_GO":
        inference = (
            "All canonical preparations ran, but repeatability, nominal motion, "
            "fixture contact, or slip failed a core qualification gate."
        )
        scientific = "The current task realization is not qualified for Phase 0C."
        next_action = "Reassess task realization before changing this fixture geometry."
    else:
        inference = (
            "The fixed frame was placeable, but adjacent cable beads contacted "
            "the walls during staging in four seeds, so the frame alone did not "
            "produce five comparable common states."
        )
        scientific = "No R1 task-qualification conclusion is available."
        next_action = (
            "Redesign canonical staging preparation so the trailing cable remains "
            "outside the channel, then rerun all five fixed seeds."
        )
    return fact, inference, scientific, next_action


def _r1_1_interpretation(result: Mapping[str, Any]):
    aggregate = result["aggregate"]
    verdict = result["verdict"]
    fact = (
        f"All {aggregate['preparation_attempted_seeds']} fixed seeds were "
        f"attempted; local geometry passed in "
        f"{aggregate['local_geometry_pass_count']}, after-grasp geometry passed "
        f"in {aggregate['after_grasp_pass_count']}, and "
        f"{aggregate['common_snapshot_pass_count']} reached a contact-free common "
        f"snapshot. {aggregate['completed_seeds']} paired rollouts completed."
    )
    if verdict == "PHASE0B_R1_1_GO":
        inference = (
            "Local-segment canonicalization produces comparable common states "
            "and the unchanged fixture task passes all qualification gates."
        )
        scientific = (
            "Phase 0B task qualification is complete; predictive mode necessity "
            "has not yet been tested."
        )
        next_action = "Run the Phase 0C matched-state/action Oracle-mode necessity smoke."
    elif verdict == "PHASE0B_R1_1_WEAK_JAM":
        inference = (
            "Preparation, repeatability, nominal insertion, fixture contact, and "
            "slip pass, but the unchanged jam probe remains weak."
        )
        scientific = (
            "The benchmark is qualified for free/contact/slip; jam geometry needs "
            "a separately controlled R2 qualification."
        )
        next_action = (
            "Increase jam_lateral_offset_m by 0.002 m in Phase 0B-R2 and rerun "
            "the same five seeds."
        )
    elif verdict == "PHASE0B_R1_1_PREPARATION_FAIL":
        inference = (
            "The fixed one-pass initialization did not yield five valid common "
            "snapshots, so downstream rollout counts are not a task verdict."
        )
        scientific = "Initial-state generation remains unqualified."
        next_action = (
            "Inspect compatibility between the cable constraint frames and the five-bead reset geometry."
        )
    elif verdict == "PHASE0B_R1_1_NOMINAL_FAIL":
        inference = (
            "Initial-state preparation and repeatability pass, but centered "
            "insertion does not reliably achieve the unchanged progress gate."
        )
        scientific = (
            "The next limitation is nominal task geometry or control, not state preparation."
        )
        next_action = (
            "Diagnose centered nominal insertion geometry and control without lowering the progress threshold."
        )
    elif verdict == "PHASE0B_R1_1_NO_GO":
        inference = (
            "Preparation, repeatability, and nominal execution are adequate, but "
            "fixture contact or slip is not reliably realized."
        )
        scientific = "The current contact-regime task is not qualified for Phase 0C."
        next_action = "Reassess fixture-contact and slip task realization."
    else:
        inference = "A simulator, data, or snapshot error prevented valid analysis."
        scientific = "No scientific qualification conclusion is available."
        next_action = "Fix the reported engineering failure and rerun the five fixed seeds."
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
    if config.get("preparation", {}).get("defer_fixture_creation", False):
        fact, inference, scientific, next_action = _r1_1_interpretation(
            result
        )
        text = f"""# Phase 0B-R1.1 Local-Segment Canonicalization

## Verdict
{result['verdict']}

## Preparation
- attempted: {aggregate['preparation_attempted_seeds']} / 5
- local geometry pass: {aggregate['local_geometry_pass_count']} / 5
- after-grasp pass: {aggregate['after_grasp_pass_count']} / 5
- fixture overlap-free: {aggregate['fixture_overlap_free_count']} / 5
- fixture-settle contact-free: {aggregate['fixture_settle_contact_free_count']} / 5
- common snapshot PASS: {aggregate['common_snapshot_pass_count']} / 5

## Local geometry
- median minimum entry clearance: {_format(aggregate['median_minimum_entry_clearance_m'])} m
- median alignment cosine: {_format(aggregate['median_alignment_cosine'])}
- median max local speed: {_format(aggregate['median_max_local_speed_mps'])} m/s

## Repeat stability
- median RMSE @100 ms: {_format(aggregate['median_repeat_rmse_100ms'])} m
- median RMSE @250 ms: {_format(aggregate['median_repeat_rmse_250ms'])} m
- median RMSE @500 ms: {_format(aggregate['median_repeat_rmse_500ms'])} m
- seeds <= 2 mm: {aggregate['repeat_seeds_le_2mm']} / 5

## Nominal
- executable: {aggregate['nominal_executable_branches']} / 5
- progress >= 0.040 m: {aggregate['nominal_progress_successful_branches']} / 5
- median final progress: {_format(aggregate['median_nominal_final_progress_m'])} m
- sustained jam: {aggregate['nominal_jam_count']} / 5

## Fixture contact
- slide contact: {aggregate['slide_contact_branches']} / 5
- jam contact: {aggregate['jam_contact_branches']} / 5

## Modes
- sustained slip: {aggregate['sustained_slip_branches']} / 5
- sustained stick: {aggregate['sustained_stick_seeds']} / 5
- sustained jam: {aggregate['sustained_jam_branches']} / 5

## Representative transitions
- slide: {aggregate['representative_slide_sequence']}
- jam: {aggregate['representative_jam_sequence']}

## Benchmark boundary
Local-segment canonicalization is benchmark initial-state generation. It is applied before the common snapshot and identically for all future branches. No cable teleportation or state reset is allowed after the common snapshot.

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

    if config.get("canonical_frame", {}).get("mode") == "canonical":
        fact, inference, scientific, next_action = _r1_interpretation(result)
        text = f"""# Phase 0B-R1 Canonical Fixture Qualification

## Verdict
{result['verdict']}

## Preparation
- completed seeds: {aggregate['preparation_successful_seeds']} / 5
- canonical staging contact-free: {aggregate['contact_free_staging_seeds']} / 5
- fixture placement failures: {aggregate['fixture_placement_failures']} / 5
- prep failures: {aggregate['preparation_failures']} / 5

## Repeat stability
- median RMSE @100 ms: {_format(aggregate['median_repeat_rmse_100ms'])} m
- median RMSE @250 ms: {_format(aggregate['median_repeat_rmse_250ms'])} m
- median RMSE @500 ms: {_format(aggregate['median_repeat_rmse_500ms'])} m
- seeds <= 2 mm: {aggregate['repeat_seeds_le_2mm']} / 5

## Nominal
- executable: {aggregate['nominal_executable_branches']} / 5
- median final progress: {_format(aggregate['median_nominal_final_progress_m'])} m
- sustained jam: {aggregate['nominal_jam_count']} / 5

## Fixture contact
- slide contact: {aggregate['slide_contact_branches']} / 5
- jam contact: {aggregate['jam_contact_branches']} / 5

## Modes
- sustained slip in slide_probe: {aggregate['sustained_slip_branches']} / 5
- sustained stick: {aggregate['sustained_stick_seeds']} / 5
- sustained jam in jam_probe: {aggregate['sustained_jam_branches']} / 5

## Representative transitions
- slide: {aggregate['representative_slide_sequence']}
- jam: {aggregate['representative_jam_sequence']}

## Comparison with Phase 0B
- previous completed: 3 / 5
- previous repeat @500ms: 0.000741054 m
- previous slip: 2 / 3 completed
- previous jam: 0 / 3 completed
- previous preparation failures: 82003, 82004

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
