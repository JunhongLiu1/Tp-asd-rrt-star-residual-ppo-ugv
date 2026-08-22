#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional
from radiation_mapping.final_score_model import (
    calculate_final_score,
    RADIATION_REFERENCE,
    TERRAIN_REFERENCE,
    EXECUTION_TIME_REFERENCE_S,
    RADIATION_WEIGHT,
    TERRAIN_WEIGHT,
    EXECUTION_TIME_WEIGHT,
)

PLANNER_ORDER = ["asd", "tp", "aco"]
PLANNER_NAMES = {
    "asd": "ASD-RRT*",
    "tp": "TP-ASD-RRT*",
    "aco": "ACO Trackable-Safe V8",
}

WEIGHTS = {
    "radiation": RADIATION_WEIGHT,
    "terrain": TERRAIN_WEIGHT,
    "execution_time": EXECUTION_TIME_WEIGHT,
}

# Contact is recorded separately and does not add a score penalty.
FAILURE_PENALTY = 0.0
GOAL_ERROR_LIMIT_M = 0.50


def as_float(value: Any) -> Optional[float]:
    try:
        if value is None or value == "":
            return None
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def as_bool(value: Any) -> bool:
    return value is True or str(value).strip().lower() == "true"


def clipped_ratio(value: Optional[float], reference: float) -> Optional[float]:
    if value is None or reference <= 0.0:
        return None
    return normalize_metric(value, reference)


def metric(summary: Dict[str, Any], *keys: str) -> Optional[float]:
    for key in keys:
        value = as_float(summary.get(key))
        if value is not None:
            return value
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    scenario_config_path = Path(args.scenario_config)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    config = json.loads(scenario_config_path.read_text(encoding="utf-8"))
    scenario_definitions = config.get("scenarios", {})

    with manifest_path.open(newline="", encoding="utf-8") as file:
        manifest_rows = list(csv.DictReader(file, delimiter="\t"))

    rows: List[Dict[str, Any]] = []

    for item in manifest_rows:
        scenario = item["scenario"]
        planner_key = item["planner_key"]
        summary_text = (item.get("summary_json") or "").strip()
        summary_path = Path(summary_text) if summary_text else None
        runner_exit_code = int(item.get("runner_exit_code") or 999)
        summary: Dict[str, Any] = {}

        if summary_path is not None and summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except Exception:
                summary = {}

        definition = scenario_definitions.get(scenario, {})
        goal = definition.get("goal", {})

        radiation = metric(summary, "executed_radiation_map_cost")
        terrain = metric(summary, "executed_terrain_cost")
        execution_time = metric(
            summary,
            "execution_time_follower_s",
            "execution_time_s",
        )
        dose = metric(summary, "dose_during_execution_usv")
        path_length = metric(summary, "executed_path_length_m")
        planning_time = metric(summary, "planning_time_wall_s", "planning_time_s")
        final_error = metric(summary, "final_goal_error_m")
        tracking_rms = metric(summary, "tracking_rms_error_m")

        summary_success = as_bool(summary.get("success"))
        contact_pass = as_bool(summary.get("contact_pass"))
        goal_ok = final_error is not None and final_error <= GOAL_ERROR_LIMIT_M
        metrics_complete = all(
            value is not None
            for value in (radiation, terrain, execution_time)
        )
        # A completed trajectory remains valid when contact is observed.
        # Contact is retained as an execution-quality metric.
        valid_run = (
            goal_ok
            and metrics_complete
        )

        contact_observed = not contact_pass

        rows.append({
            "scenario": scenario,
            "condition": definition.get("condition", ""),
            "goal_x": as_float(goal.get("x")),
            "goal_y": as_float(goal.get("y")),
            "planner_key": planner_key,
            "planner_name": PLANNER_NAMES.get(planner_key, planner_key),
            "seed": int(item.get("seed") or 31),
            "runner_exit_code": runner_exit_code,
            "summary_success": summary_success,
            "contact_pass": contact_pass,
            "contact_observed": contact_observed,
            "valid_run": valid_run,
            "executed_radiation_map_cost": radiation,
            "executed_terrain_cost": terrain,
            "execution_time_s": execution_time,
            "dose_during_execution_usv": dose,
            "executed_path_length_m": path_length,
            "planning_time_s": planning_time,
            "final_goal_error_m": final_error,
            "tracking_rms_error_m": tracking_rms,
            "failure_reason": str(summary.get("failure_reason") or ""),
            "summary_json": summary_text,
        })

    valid_rows = [row for row in rows if row["valid_run"]]
    if not valid_rows:
        raise SystemExit(
            "[ERROR] No completed multi-goal run was available for scoring"
        )

    # Fixed references shared by formal, pilot and multi-goal tests.
    references = {
        "executed_radiation_map_cost": RADIATION_REFERENCE,
        "executed_terrain_cost": TERRAIN_REFERENCE,
        "execution_time_s": EXECUTION_TIME_REFERENCE_S,
    }

    for row in rows:
        score_result = calculate_final_score(
            radiation_cost=row["executed_radiation_map_cost"],
            terrain_cost=row["executed_terrain_cost"],
            execution_time_s=row["execution_time_s"],
        )

        row["radiation_reference"] = RADIATION_REFERENCE
        row["terrain_reference"] = TERRAIN_REFERENCE
        row["execution_time_reference_s"] = (
            EXECUTION_TIME_REFERENCE_S
        )

        row["normalized_radiation"] = (
            score_result.normalized_radiation
        )
        row["normalized_terrain"] = (
            score_result.normalized_terrain
        )
        row["normalized_execution_time"] = (
            score_result.normalized_execution_time
        )

        base = score_result.final_score

        row["base_pilot_score"] = base
        row["constraint_failure_penalty"] = 0.0
        row["pilot_final_score"] = base
        row["rank_within_goal"] = 0

    scenario_order = []
    for item in manifest_rows:
        if item["scenario"] not in scenario_order:
            scenario_order.append(item["scenario"])

    scenario_rankings: List[Dict[str, Any]] = []
    for scenario in scenario_order:
        group = [row for row in rows if row["scenario"] == scenario]
        group.sort(key=lambda row: float(row["pilot_final_score"]))
        for rank, row in enumerate(group, start=1):
            row["rank_within_goal"] = rank
            scenario_rankings.append({
                "scenario": scenario,
                "condition": row["condition"],
                "goal_x": row["goal_x"],
                "goal_y": row["goal_y"],
                "rank": rank,
                "planner_name": row["planner_name"],
                "valid_run": row["valid_run"],
                "contact_pass": row["contact_pass"],
                "contact_observed": row["contact_observed"],
                "pilot_final_score": row["pilot_final_score"],
                "executed_radiation_map_cost": row[
                    "executed_radiation_map_cost"
                ],
                "executed_terrain_cost": row["executed_terrain_cost"],
                "execution_time_s": row["execution_time_s"],
                "executed_path_length_m": row["executed_path_length_m"],
            })

    rows.sort(key=lambda row: (
        scenario_order.index(row["scenario"]),
        int(row["rank_within_goal"]),
    ))

    all_runs_csv = output_dir / "multi_goal_all_runs.csv"
    all_run_fields = list(rows[0].keys())
    with all_runs_csv.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=all_run_fields)
        writer.writeheader()
        writer.writerows(rows)

    rankings_csv = output_dir / "multi_goal_scenario_ranking.csv"
    ranking_fields = list(scenario_rankings[0].keys())
    with rankings_csv.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=ranking_fields)
        writer.writeheader()
        writer.writerows(scenario_rankings)

    tp_report: List[Dict[str, Any]] = []

    def by_key(group: List[Dict[str, Any]], key: str):
        return next((row for row in group if row["planner_key"] == key), None)

    def difference(tp_row, other_row, field):
        if (
            tp_row is None
            or other_row is None
            or tp_row.get(field) is None
            or other_row.get(field) is None
        ):
            return None
        return float(tp_row[field]) - float(other_row[field])

    for scenario in scenario_order:
        group = [row for row in rows if row["scenario"] == scenario]
        tp = by_key(group, "tp")
        asd = by_key(group, "asd")
        aco = by_key(group, "aco")

        tp_score_delta_asd = difference(
            tp, asd, "pilot_final_score"
        )
        tp_score_delta_aco = difference(
            tp, aco, "pilot_final_score"
        )
        tp_time_delta_asd = difference(tp, asd, "execution_time_s")
        tp_terrain_delta_asd = difference(
            tp, asd, "executed_terrain_cost"
        )
        tp_radiation_delta_asd = difference(
            tp, asd, "executed_radiation_map_cost"
        )
        tp_length_delta_asd = difference(
            tp, asd, "executed_path_length_m"
        )

        if tp is None or not tp["valid_run"]:
            result = "TP invalid"
            interpretation = "TP could not produce a valid formal run"
        elif int(tp["rank_within_goal"]) == 1:
            result = "TP best"
            improvements = []
            if tp_time_delta_asd is not None and tp_time_delta_asd < -0.20:
                improvements.append("lower execution time")
            if (
                tp_terrain_delta_asd is not None
                and tp_terrain_delta_asd < -0.10
            ):
                improvements.append("lower terrain cost")
            if (
                tp_radiation_delta_asd is not None
                and tp_radiation_delta_asd < -0.01
            ):
                improvements.append("lower radiation cost")
            interpretation = (
                ", ".join(improvements)
                if improvements
                else "small combined improvements across weighted metrics"
            )
        elif (
            tp_score_delta_asd is not None
            and tp_score_delta_asd < 0.0
        ):
            result = "TP beats ASD but not ACO"
            interpretation = (
                "time-aware extension improves on ASD, but ACO remains better"
            )
        else:
            result = "TP not better"
            worsening = []
            if tp_time_delta_asd is not None and tp_time_delta_asd > 0.20:
                worsening.append("execution time increased")
            if (
                tp_terrain_delta_asd is not None
                and tp_terrain_delta_asd > 0.10
            ):
                worsening.append("terrain cost increased")
            if (
                tp_radiation_delta_asd is not None
                and tp_radiation_delta_asd > 0.01
            ):
                worsening.append("radiation cost increased")
            interpretation = (
                ", ".join(worsening)
                if worsening
                else "time penalty did not create a meaningful weighted advantage"
            )

        tp_report.append({
            "scenario": scenario,
            "condition": tp["condition"] if tp else "",
            "goal_x": tp["goal_x"] if tp else None,
            "goal_y": tp["goal_y"] if tp else None,
            "tp_result": result,
            "tp_rank": tp["rank_within_goal"] if tp else None,
            "tp_valid": tp["valid_run"] if tp else False,
            "tp_pilot_final_score": (
                tp["pilot_final_score"] if tp else None
            ),
            "asd_pilot_final_score": (
                asd["pilot_final_score"] if asd else None
            ),
            "aco_pilot_final_score": (
                aco["pilot_final_score"] if aco else None
            ),
            "tp_minus_asd_final_score": tp_score_delta_asd,
            "tp_minus_asd_execution_time_s": tp_time_delta_asd,
            "tp_minus_asd_terrain_cost": tp_terrain_delta_asd,
            "tp_minus_asd_radiation_cost": tp_radiation_delta_asd,
            "tp_minus_asd_path_length_m": tp_length_delta_asd,
            "interpretation": interpretation,
        })

    tp_csv = output_dir / "tp_condition_report.csv"
    tp_fields = list(tp_report[0].keys())
    with tp_csv.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=tp_fields)
        writer.writeheader()
        writer.writerows(tp_report)

    reference_path = output_dir / "multi_goal_candidate_references.json"
    reference_path.write_text(
        json.dumps({
            "status": "exploratory_not_frozen",
            "rule": "maximum valid value across all nine pilot runs times 1.10",
            "weights": WEIGHTS,
            "references": references,
            "warning": (
                "These references are for this multi-goal pilot only. "
                "Do not replace the frozen formal references automatically."
            ),
        }, indent=2),
        encoding="utf-8",
    )

    summary_json = output_dir / "multi_goal_tp_probe_summary.json"
    summary_json.write_text(
        json.dumps({
            "comparison_type": "one run per planner at three fixed goals",
            "weights": WEIGHTS,
            "references": references,
            "all_runs": rows,
            "scenario_rankings": scenario_rankings,
            "tp_condition_report": tp_report,
        }, indent=2),
        encoding="utf-8",
    )

    print()
    print("=" * 118)
    print("MULTI-GOAL TP PROBE — ONE RUN PER PLANNER AND GOAL")
    print("Lower pilot final score is better. Fixed references are shared across formal and multi-goal tests. Normalisation is uncapped.")
    print("=" * 118)

    for scenario in scenario_order:
        group = [row for row in rows if row["scenario"] == scenario]
        group.sort(key=lambda row: int(row["rank_within_goal"]))
        first = group[0]
        print(
            f"\n{scenario}  goal=({first['goal_x']:.2f}, {first['goal_y']:.2f})"
            f"  condition={first['condition']}"
        )
        print(
            f"{'RANK':<6}{'PLANNER':<25}{'SCORE':>12}{'RAD':>12}"
            f"{'TERRAIN':>12}{'TIME':>11}{'VALID':>9}"
        )
        for row in group:
            print(
                f"{row['rank_within_goal']:<6}"
                f"{row['planner_name']:<25}"
                f"{row['pilot_final_score']:>12.6f}"
                f"{(row['executed_radiation_map_cost'] or 0):>12.6f}"
                f"{(row['executed_terrain_cost'] or 0):>12.6f}"
                f"{(row['execution_time_s'] or 0):>11.3f}"
                f"{str(row['valid_run']):>9}"
            )

    print()
    print("=" * 118)
    print("TP CONDITION REPORT")
    print("=" * 118)
    for row in tp_report:
        print(
            f"{row['scenario']}: {row['tp_result']} | "
            f"TP-ASD score delta={row['tp_minus_asd_final_score']} | "
            f"{row['interpretation']}"
        )

    print()
    print(f"All runs CSV:   {all_runs_csv}")
    print(f"Ranking CSV:    {rankings_csv}")
    print(f"TP report CSV:  {tp_csv}")
    print(f"References:     {reference_path}")
    print(f"Summary JSON:   {summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
