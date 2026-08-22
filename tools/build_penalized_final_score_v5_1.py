#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

WEIGHTS = {"radiation": 0.4, "terrain": 0.4, "execution_time": 0.2}
REFERENCE_MARGIN = 1.10
FAILURE_PENALTY = 100.0
EXPECTED = {"asd", "tp", "aco"}


def as_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def load_manifest(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [dict(row) for row in csv.DictReader(handle, delimiter="\t")]


def clipped_norm(value: float, reference: float) -> float:
    if reference <= 0.0:
        raise ValueError(f"Reference must be positive, got {reference}")
    return max(0.0, min(value / reference, 1.0))


def spread_percent(values: List[float]) -> Optional[float]:
    if not values:
        return None
    minimum = min(values)
    maximum = max(values)
    if minimum <= 0.0:
        return None
    return 100.0 * (maximum - minimum) / minimum


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--radiation-map-stats")
    args = parser.parse_args()

    manifest = Path(args.manifest).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, Any]] = []
    for item in load_manifest(manifest):
        summary_text = (item.get("summary_json") or "").strip()
        summary_path = Path(summary_text) if summary_text else None
        try:
            runner_exit_code = int(item.get("runner_exit_code") or 999)
        except ValueError:
            runner_exit_code = 999

        row: Dict[str, Any] = {
            "scenario": item.get("scenario", ""),
            "planner_key": item.get("planner_key", ""),
            "planner_name": item.get("planner_key", ""),
            "seed": item.get("seed", ""),
            "runner_exit_code": runner_exit_code,
            "summary_json": summary_text,
            "success": False,
            "contact_pass": False,
            "failure_reason": "summary.json missing",
        }

        if summary_path is not None and summary_path.is_file():
            data = json.loads(summary_path.read_text(encoding="utf-8"))
            radiation = as_float(data.get("executed_radiation_map_cost"))
            terrain = as_float(data.get("executed_terrain_cost"))
            execution_time = as_float(data.get("execution_time_follower_s"))
            contact_pass = as_bool(data.get("contact_pass"))
            success = (
                as_bool(data.get("success"))
                and runner_exit_code == 0
                and contact_pass
                and radiation is not None
                and terrain is not None
                and execution_time is not None
            )
            row.update({
                "scenario": data.get("scenario", row["scenario"]),
                "planner_key": data.get("planner_key", row["planner_key"]),
                "planner_name": data.get("planner_name", row["planner_name"]),
                "seed": data.get("seed", row["seed"]),
                "success": success,
                "contact_pass": contact_pass,
                "failure_reason": data.get("failure_reason", "") if not success else "",
                "executed_radiation_map_cost": radiation,
                "executed_terrain_cost": terrain,
                "execution_time_s": execution_time,
                "dose_during_execution_usv": as_float(data.get("dose_during_execution_usv")),
                "executed_path_length_m": as_float(data.get("executed_path_length_m")),
                "planning_time_s": as_float(data.get("planning_time_wall_s")),
                "final_goal_error_m": as_float(data.get("final_goal_error_m")),
                "tracking_rms_error_m": as_float(data.get("tracking_rms_error_m")),
                "contact_chassis_blocks": data.get("contact_chassis_blocks"),
                "legacy_recorded_score": as_float(data.get("executed_final_coupled_score")),
                "summary_json": str(summary_path),
            })
        rows.append(row)

    planner_keys = {str(row.get("planner_key")) for row in rows}
    metric_rows = [
        row for row in rows
        if row.get("executed_radiation_map_cost") is not None
        and row.get("executed_terrain_cost") is not None
        and row.get("execution_time_s") is not None
    ]
    if len(rows) != 3 or planner_keys != EXPECTED:
        raise RuntimeError(
            f"Expected exactly asd/tp/aco rows; got rows={len(rows)}, keys={sorted(planner_keys)}"
        )
    if len(metric_rows) != 3:
        raise RuntimeError("All three runs must contain R/T/time metrics to produce a penalized ranking")

    references = {
        "version": "r3_penalized_candidate_v5_1",
        "status": "single-seed pilot candidate_not_frozen",
        "reference_rule": "max of all three completed trajectories multiplied by 1.10",
        "reference_margin": REFERENCE_MARGIN,
        "weights": WEIGHTS,
        "normalization_clipped_to_0_1": True,
        "failure_penalty": FAILURE_PENALTY,
        "radiation_reference": max(float(r["executed_radiation_map_cost"]) for r in metric_rows) * REFERENCE_MARGIN,
        "terrain_reference": max(float(r["executed_terrain_cost"]) for r in metric_rows) * REFERENCE_MARGIN,
        "execution_time_reference_s": max(float(r["execution_time_s"]) for r in metric_rows) * REFERENCE_MARGIN,
    }
    references_path = output_dir / "penalized_score_references_candidate.json"
    references_path.write_text(json.dumps(references, indent=2), encoding="utf-8")

    for row in rows:
        rn = clipped_norm(float(row["executed_radiation_map_cost"]), references["radiation_reference"])
        tn = clipped_norm(float(row["executed_terrain_cost"]), references["terrain_reference"])
        en = clipped_norm(float(row["execution_time_s"]), references["execution_time_reference_s"])
        base = 100.0 * (
            WEIGHTS["radiation"] * rn
            + WEIGHTS["terrain"] * tn
            + WEIGHTS["execution_time"] * en
        )
        penalty = 0.0 if row["success"] else FAILURE_PENALTY
        row.update({
            "radiation_weight": WEIGHTS["radiation"],
            "terrain_weight": WEIGHTS["terrain"],
            "execution_time_weight": WEIGHTS["execution_time"],
            "radiation_reference": references["radiation_reference"],
            "terrain_reference": references["terrain_reference"],
            "execution_time_reference_s": references["execution_time_reference_s"],
            "normalized_radiation": rn,
            "normalized_terrain": tn,
            "normalized_execution_time": en,
            "base_final_score": base,
            "constraint_failure_penalty": penalty,
            "final_score": base + penalty,
        })

    rows.sort(key=lambda row: float(row["final_score"]))
    for index, row in enumerate(rows, 1):
        row["rank"] = index

    all_success = all(bool(row["success"]) for row in rows)
    ranking_complete = len(rows) == 3 and all(row.get("final_score") is not None for row in rows)

    map_stats = None
    if args.radiation_map_stats:
        stats_path = Path(args.radiation_map_stats).expanduser().resolve()
        if stats_path.is_file():
            map_stats = json.loads(stats_path.read_text(encoding="utf-8"))

    r_values = [float(row["executed_radiation_map_cost"]) for row in metric_rows]
    assessment = {
        "ranking_complete": ranking_complete,
        "all_three_runs_valid": all_success,
        "successful_run_count": sum(1 for row in rows if row["success"]),
        "failed_planners": [row["planner_name"] for row in rows if not row["success"]],
        "failure_policy": (
            "Base score is 0-100 after clipped normalization. "
            "Any constraint-invalid run receives +100, so every invalid run ranks below every valid run."
        ),
        "radiation_cost_relative_spread_percent_all_completed": spread_percent(r_values),
        "radiation_map_stats": map_stats,
    }

    csv_path = output_dir / "penalized_final_score_comparison.csv"
    json_path = output_dir / "penalized_final_score_comparison.json"
    assessment_path = output_dir / "penalized_score_assessment.json"

    fields = [
        "rank", "scenario", "planner_key", "planner_name", "seed",
        "success", "contact_pass", "runner_exit_code",
        "final_score", "base_final_score", "constraint_failure_penalty",
        "radiation_weight", "terrain_weight", "execution_time_weight",
        "executed_radiation_map_cost", "radiation_reference", "normalized_radiation",
        "executed_terrain_cost", "terrain_reference", "normalized_terrain",
        "execution_time_s", "execution_time_reference_s", "normalized_execution_time",
        "dose_during_execution_usv", "executed_path_length_m", "planning_time_s",
        "final_goal_error_m", "tracking_rms_error_m", "contact_chassis_blocks",
        "failure_reason", "summary_json",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    assessment_path.write_text(json.dumps(assessment, indent=2), encoding="utf-8")
    payload = {
        "ranking_rule": "lower final_score is better",
        "formula_valid": "100*(0.4*R_norm + 0.4*T_norm + 0.2*E_norm)",
        "formula_invalid": "base_final_score + 100 constraint failure penalty",
        "references": references,
        "assessment": assessment,
        "rows": rows,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def fmt(value: Any, digits: int = 6) -> str:
        return "N/A" if value is None or value == "" else f"{float(value):.{digits}f}"

    print("\n" + "=" * 124)
    print("R3 PENALIZED FINAL SCORE — SINGLE-SEED PILOT")
    print("Lower is better. Valid score=base 0-100; invalid run=base+100 penalty.")
    print("=" * 124)
    print(f"{'RANK':<6}{'PLANNER':<22}{'FINAL':>12}{'BASE':>12}{'PENALTY':>12}{'RAD COST':>12}{'TERRAIN':>12}{'TIME':>10}{'VALID':>10}")
    for row in rows:
        print(
            f"{row['rank']:<6}{str(row['planner_name']):<22}"
            f"{fmt(row['final_score']):>12}{fmt(row['base_final_score']):>12}"
            f"{fmt(row['constraint_failure_penalty'], 1):>12}"
            f"{fmt(row['executed_radiation_map_cost']):>12}"
            f"{fmt(row['executed_terrain_cost']):>12}"
            f"{fmt(row['execution_time_s'], 2):>10}{str(row['success']):>10}"
        )
    print("=" * 124)
    print("ranking_complete:", ranking_complete)
    print("all_three_runs_valid:", all_success)
    print("failed_planners:", assessment["failed_planners"])
    print("CSV:", csv_path)
    print("JSON:", json_path)
    print("References:", references_path)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
