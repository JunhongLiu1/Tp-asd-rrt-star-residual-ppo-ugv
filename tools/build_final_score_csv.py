#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

DOSE_WEIGHT = 0.5
TERRAIN_WEIGHT = 0.3
TIME_WEIGHT = 0.2


def as_float(value: Any) -> Optional[float]:
    try:
        if value is None or value == "":
            return None
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def load_manifest(path: Path) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        for row in reader:
            rows.append(dict(row))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    manifest_path = Path(args.manifest).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    manifest_rows = load_manifest(manifest_path)
    result_rows: List[Dict[str, Any]] = []

    for item in manifest_rows:
        summary_text = (item.get("summary_json") or "").strip()
        runner_exit_code = int(item.get("runner_exit_code") or 999)
        summary_path = Path(summary_text) if summary_text else None

        if summary_path is None or not summary_path.is_file():
            result_rows.append({
                "rank": "",
                "comparison_valid": False,
                "scenario": item.get("scenario", ""),
                "planner_key": item.get("planner_key", ""),
                "planner_name": item.get("planner_key", ""),
                "seed": item.get("seed", ""),
                "success": False,
                "runner_exit_code": runner_exit_code,
                "final_score": "",
                "recorded_final_score": "",
                "score_difference": "",
                "dose_weight": DOSE_WEIGHT,
                "terrain_weight": TERRAIN_WEIGHT,
                "time_weight": TIME_WEIGHT,
                "dose_during_execution_usv": "",
                "executed_terrain_cost": "",
                "execution_time_s": "",
                "executed_radiation_map_cost": "",
                "executed_path_length_m": "",
                "planning_time_s": "",
                "final_goal_error_m": "",
                "tracking_rms_error_m": "",
                "contact_pass": False,
                "failure_reason": "summary.json missing",
                "summary_json": summary_text,
            })
            continue

        data = json.loads(summary_path.read_text(encoding="utf-8"))
        dose = as_float(data.get("dose_during_execution_usv"))
        terrain = as_float(data.get("executed_terrain_cost"))
        execution_time = as_float(data.get("execution_time_follower_s"))
        recorded_score = as_float(data.get("executed_final_coupled_score"))

        computed_score: Optional[float] = None
        if dose is not None and terrain is not None and execution_time is not None:
            computed_score = (
                DOSE_WEIGHT * dose
                + TERRAIN_WEIGHT * terrain
                + TIME_WEIGHT * execution_time
            )

        success = (
            as_bool(data.get("success"))
            and runner_exit_code == 0
            and computed_score is not None
        )

        score_difference: Optional[float] = None
        if computed_score is not None and recorded_score is not None:
            score_difference = computed_score - recorded_score

        result_rows.append({
            "rank": "",
            "comparison_valid": False,
            "scenario": data.get("scenario", item.get("scenario", "")),
            "planner_key": data.get("planner_key", item.get("planner_key", "")),
            "planner_name": data.get("planner_name", item.get("planner_key", "")),
            "seed": data.get("seed", item.get("seed", "")),
            "success": success,
            "runner_exit_code": runner_exit_code,
            "final_score": computed_score,
            "recorded_final_score": recorded_score,
            "score_difference": score_difference,
            "dose_weight": DOSE_WEIGHT,
            "terrain_weight": TERRAIN_WEIGHT,
            "time_weight": TIME_WEIGHT,
            "dose_during_execution_usv": dose,
            "executed_terrain_cost": terrain,
            "execution_time_s": execution_time,
            "executed_radiation_map_cost": as_float(
                data.get("executed_radiation_map_cost")
            ),
            "executed_path_length_m": as_float(
                data.get("executed_path_length_m")
            ),
            "planning_time_s": as_float(data.get("planning_time_wall_s")),
            "final_goal_error_m": as_float(data.get("final_goal_error_m")),
            "tracking_rms_error_m": as_float(data.get("tracking_rms_error_m")),
            "contact_pass": as_bool(data.get("contact_pass")),
            "failure_reason": data.get("failure_reason", ""),
            "summary_json": str(summary_path),
        })

    expected_planners = {"asd", "tp", "aco"}
    successful_planners = {
        str(row["planner_key"])
        for row in result_rows
        if row["success"]
    }
    comparison_valid = (
        len(result_rows) == 3
        and successful_planners == expected_planners
    )

    successful_rows = [row for row in result_rows if row["success"]]
    successful_rows.sort(key=lambda row: float(row["final_score"]))
    rank_by_planner = {
        str(row["planner_key"]): index
        for index, row in enumerate(successful_rows, start=1)
    }

    for row in result_rows:
        row["comparison_valid"] = comparison_valid
        if row["success"]:
            row["rank"] = rank_by_planner[str(row["planner_key"])]

    result_rows.sort(
        key=lambda row: (
            0 if row["success"] else 1,
            int(row["rank"]) if row["rank"] != "" else 999,
            str(row["planner_key"]),
        )
    )

    csv_path = output_dir / "final_score_comparison.csv"
    json_path = output_dir / "final_score_comparison.json"

    fieldnames = [
        "rank",
        "comparison_valid",
        "scenario",
        "planner_key",
        "planner_name",
        "seed",
        "success",
        "runner_exit_code",
        "final_score",
        "recorded_final_score",
        "score_difference",
        "dose_weight",
        "terrain_weight",
        "time_weight",
        "dose_during_execution_usv",
        "executed_terrain_cost",
        "execution_time_s",
        "executed_radiation_map_cost",
        "executed_path_length_m",
        "planning_time_s",
        "final_goal_error_m",
        "tracking_rms_error_m",
        "contact_pass",
        "failure_reason",
        "summary_json",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result_rows)

    payload = {
        "comparison_valid": comparison_valid,
        "ranking_rule": "Lower final_score is better",
        "final_score_formula": (
            "0.5*dose_during_execution_usv + "
            "0.3*executed_terrain_cost + "
            "0.2*execution_time_s"
        ),
        "weights": {
            "dose": DOSE_WEIGHT,
            "terrain": TERRAIN_WEIGHT,
            "time": TIME_WEIGHT,
        },
        "rows": result_rows,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("\n" + "=" * 106)
    print("ONE-RUN FINAL SCORE COMPARISON")
    print("Lower final score is better")
    print("=" * 106)
    print(
        f"{'RANK':<6}{'PLANNER':<22}{'FINAL SCORE':>14}"
        f"{'TIME(s)':>12}{'TERRAIN':>12}{'DOSE(uSv)':>14}"
        f"{'RAD MAP':>12}{'SUCCESS':>10}"
    )
    for row in result_rows:
        def fmt(value: Any, digits: int = 6) -> str:
            if value is None or value == "":
                return "N/A"
            return f"{float(value):.{digits}f}"

        print(
            f"{str(row['rank']):<6}"
            f"{str(row['planner_name']):<22}"
            f"{fmt(row['final_score']):>14}"
            f"{fmt(row['execution_time_s'], 2):>12}"
            f"{fmt(row['executed_terrain_cost'], 6):>12}"
            f"{fmt(row['dose_during_execution_usv'], 6):>14}"
            f"{fmt(row['executed_radiation_map_cost'], 6):>12}"
            f"{str(row['success']):>10}"
        )

    print("=" * 106)
    print("comparison_valid:", comparison_valid)
    print("CSV: ", csv_path)
    print("JSON:", json_path)

    return 0 if comparison_valid else 2


if __name__ == "__main__":
    sys.exit(main())
