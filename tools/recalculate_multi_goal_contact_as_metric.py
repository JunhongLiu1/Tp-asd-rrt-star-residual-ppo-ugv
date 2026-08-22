#!/usr/bin/env python3

import csv
import sys
from pathlib import Path


def to_bool(value):
    return str(value).strip().lower() in {
        "true", "1", "yes", "pass"
    }


def to_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: recalculate_multi_goal_contact_as_metric.py RESULT_DIR"
    )

result_dir = Path(sys.argv[1]).expanduser().resolve()

all_runs_file = result_dir / "multi_goal_all_runs.csv"
ranking_file = result_dir / "multi_goal_scenario_ranking.csv"

if not all_runs_file.exists():
    raise SystemExit(f"Missing file: {all_runs_file}")

with all_runs_file.open(
    "r", newline="", encoding="utf-8"
) as f:
    rows = list(csv.DictReader(f))

if not rows:
    raise SystemExit("No rows found.")

for row in rows:
    radiation = to_float(
        row.get("executed_radiation_map_cost")
    )
    terrain = to_float(
        row.get("executed_terrain_cost")
    )
    execution_time = to_float(
        row.get("execution_time_s")
    )
    path_length = to_float(
        row.get("executed_path_length_m")
    )

    contact_pass = to_bool(row.get("contact_pass"))
    old_reason = row.get("failure_reason", "").strip()

    metrics_complete = (
        radiation is not None
        and terrain is not None
        and execution_time is not None
        and execution_time > 0
        and path_length is not None
        and path_length > 0
    )

    contact_only_failure = (
        not contact_pass
        and (
            old_reason == ""
            or "contact_acceptance_pass" in old_reason
            or "contact observed" in old_reason.lower()
        )
    )

    # A completed run remains valid when contact is the only issue.
    new_valid = metrics_complete and (
        contact_pass or contact_only_failure
    )

    row["valid_run"] = str(new_valid)
    row["contact_observed"] = str(not contact_pass)

    base_score = to_float(
        row.get("base_pilot_score"),
        to_float(row.get("pilot_final_score"), 0.0),
    )

    row["constraint_failure_penalty"] = "0.0"
    row["pilot_final_score"] = str(base_score)

    if new_valid and not contact_pass:
        row["failure_reason"] = (
            "contact observed; retained as valid execution metric"
        )
    elif new_valid:
        row["failure_reason"] = ""

# Rewrite all-runs CSV
fields = list(rows[0].keys())

if "contact_observed" not in fields:
    fields.append("contact_observed")

with all_runs_file.open(
    "w", newline="", encoding="utf-8"
) as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)

# Rebuild ranking
ranking_rows = []

for scenario in sorted({row["scenario"] for row in rows}):
    scenario_rows = [
        row for row in rows
        if row["scenario"] == scenario
    ]

    scenario_rows.sort(
        key=lambda row: to_float(
            row.get("pilot_final_score"),
            float("inf"),
        )
    )

    for rank, row in enumerate(scenario_rows, start=1):
        row["rank_within_goal"] = str(rank)

        ranking_rows.append({
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

with ranking_file.open(
    "w", newline="", encoding="utf-8"
) as f:
    writer = csv.DictWriter(
        f,
        fieldnames=list(ranking_rows[0].keys()),
    )
    writer.writeheader()
    writer.writerows(ranking_rows)

print("Updated:", all_runs_file)
print("Updated:", ranking_file)
print("Contact is retained as a metric, not an invalidity condition.")
