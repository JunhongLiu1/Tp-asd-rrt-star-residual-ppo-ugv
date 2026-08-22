#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
from typing import Dict, List


PLANNER_ORDER = {
    "asd": 0,
    "tp": 1,
    "aco": 2,
}


def parse_bool(value) -> bool:
    return str(value).strip().lower() in {
        "true", "1", "yes", "y", "pass"
    }


def parse_float(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: List[Dict[str, object]], fields):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "result_dir",
        type=Path,
        help="Multi-goal result directory",
    )
    args = parser.parse_args()

    result_dir = args.result_dir.expanduser().resolve()

    all_runs_path = result_dir / "multi_goal_all_runs.csv"
    ranking_path = result_dir / "multi_goal_scenario_ranking.csv"
    condition_path = result_dir / "multi_goal_tp_condition_report.csv"

    if not all_runs_path.exists():
        raise SystemExit(
            f"Cannot find input CSV: {all_runs_path}"
        )

    rows = load_csv(all_runs_path)

    allowed_scenarios = {
        "H_MG1_short_east",
        "H_MG2_long_south",
    }

    rows = [
        row for row in rows
        if row.get("scenario") in allowed_scenarios
    ]

    if not rows:
        raise SystemExit(
            "No short-route or long-route results were found."
        )

    for row in rows:
        summary_success = parse_bool(
            row.get("summary_success")
        )

        runner_exit_code = int(
            parse_float(row.get("runner_exit_code"), 1)
        )

        terrain_cost = parse_float(
            row.get("executed_terrain_cost")
        )
        radiation_cost = parse_float(
            row.get("executed_radiation_map_cost")
        )
        execution_time = parse_float(
            row.get("execution_time_s")
        )

        # Contact is retained as an execution-quality indicator,
        # but it no longer automatically invalidates the run.
        new_valid = (
            runner_exit_code == 0
            and summary_success
            and terrain_cost >= 0.0
            and radiation_cost >= 0.0
            and execution_time > 0.0
        )

        contact_pass = parse_bool(row.get("contact_pass"))

        row["valid_run"] = str(new_valid)

        # Keep the original contact result separately.
        row["contact_observed"] = str(not contact_pass)

        # Contact-related slowdown is already included in
        # execution_time_s. Therefore, do not add +100.
        base_score = parse_float(
            row.get("base_pilot_score"),
            parse_float(row.get("pilot_final_score")),
        )

        row["constraint_failure_penalty"] = 0.0
        row["pilot_final_score"] = base_score

        if new_valid and not contact_pass:
            row["failure_reason"] = (
                "contact observed; retained as valid execution"
            )
        elif new_valid:
            row["failure_reason"] = ""
        else:
            old_reason = row.get("failure_reason", "").strip()
            row["failure_reason"] = (
                old_reason or "execution result incomplete"
            )

    # Rank every planner that completed the execution successfully.
    rankings = []

    scenarios = sorted({
        row["scenario"] for row in rows
    })

    for scenario in scenarios:
        scenario_rows = [
            row for row in rows
            if row["scenario"] == scenario
        ]

        scenario_rows.sort(
            key=lambda row: (
                not parse_bool(row["valid_run"]),
                parse_float(row["pilot_final_score"], float("inf")),
                PLANNER_ORDER.get(row.get("planner_key"), 99),
            )
        )

        for rank, row in enumerate(scenario_rows, start=1):
            row["rank_within_goal"] = rank

            rankings.append({
                "scenario": row.get("scenario", ""),
                "condition": row.get("condition", ""),
                "goal_x": row.get("goal_x", ""),
                "goal_y": row.get("goal_y", ""),
                "rank": rank,
                "planner_name": row.get("planner_name", ""),
                "valid_run": row.get("valid_run", ""),
                "contact_pass": row.get("contact_pass", ""),
                "contact_observed": row.get(
                    "contact_observed", ""
                ),
                "pilot_final_score": row.get(
                    "pilot_final_score", ""
                ),
                "executed_radiation_map_cost": row.get(
                    "executed_radiation_map_cost", ""
                ),
                "executed_terrain_cost": row.get(
                    "executed_terrain_cost", ""
                ),
                "execution_time_s": row.get(
                    "execution_time_s", ""
                ),
                "executed_path_length_m": row.get(
                    "executed_path_length_m", ""
                ),
            })

    # Build a simpler TP condition report.
    condition_rows = []

    for scenario in scenarios:
        scenario_rows = [
            row for row in rows
            if row["scenario"] == scenario
        ]

        by_key = {
            row.get("planner_key"): row
            for row in scenario_rows
        }

        tp = by_key.get("tp")
        asd = by_key.get("asd")
        aco = by_key.get("aco")

        if not tp:
            continue

        valid_sorted = sorted(
            [
                row for row in scenario_rows
                if parse_bool(row["valid_run"])
            ],
            key=lambda row: parse_float(
                row["pilot_final_score"],
                float("inf"),
            ),
        )

        tp_rank = next(
            (
                index + 1
                for index, row in enumerate(valid_sorted)
                if row.get("planner_key") == "tp"
            ),
            "",
        )

        if tp_rank == 1:
            tp_result = "TP best"
        elif parse_bool(tp["valid_run"]):
            tp_result = "TP not best"
        else:
            tp_result = "TP invalid"

        condition_rows.append({
            "scenario": scenario,
            "condition": tp.get("condition", ""),
            "goal_x": tp.get("goal_x", ""),
            "goal_y": tp.get("goal_y", ""),
            "tp_result": tp_result,
            "tp_rank": tp_rank,
            "tp_valid": tp.get("valid_run", ""),
            "tp_contact_observed": tp.get(
                "contact_observed", ""
            ),
            "tp_pilot_final_score": tp.get(
                "pilot_final_score", ""
            ),
            "asd_pilot_final_score": (
                asd.get("pilot_final_score", "")
                if asd else ""
            ),
            "aco_pilot_final_score": (
                aco.get("pilot_final_score", "")
                if aco else ""
            ),
            "tp_minus_asd_final_score": (
                parse_float(tp.get("pilot_final_score"))
                - parse_float(asd.get("pilot_final_score"))
                if asd else ""
            ),
            "tp_minus_asd_execution_time_s": (
                parse_float(tp.get("execution_time_s"))
                - parse_float(asd.get("execution_time_s"))
                if asd else ""
            ),
            "tp_minus_asd_terrain_cost": (
                parse_float(tp.get("executed_terrain_cost"))
                - parse_float(
                    asd.get("executed_terrain_cost")
                )
                if asd else ""
            ),
            "tp_minus_asd_radiation_cost": (
                parse_float(
                    tp.get("executed_radiation_map_cost")
                )
                - parse_float(
                    asd.get(
                        "executed_radiation_map_cost"
                    )
                )
                if asd else ""
            ),
        })

    all_fields = list(rows[0].keys())

    if "contact_observed" not in all_fields:
        all_fields.append("contact_observed")

    write_csv(all_runs_path, rows, all_fields)

    ranking_fields = list(rankings[0].keys())
    write_csv(ranking_path, rankings, ranking_fields)

    condition_fields = list(condition_rows[0].keys())
    write_csv(condition_path, condition_rows, condition_fields)

    print()
    print("Recalculation completed.")
    print(f"Updated: {all_runs_path}")
    print(f"Updated: {ranking_path}")
    print(f"Updated: {condition_path}")
    print()
    print(
        "Contact is now reported separately and does not "
        "automatically invalidate a completed execution."
    )


if __name__ == "__main__":
    main()
