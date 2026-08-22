#!/usr/bin/env python3
import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional
from radiation_mapping.final_score_model import normalize_metric


PLANNER_ORDER = ["asd", "tp", "aco"]
PLANNER_NAMES = {
    "asd": "ASD-RRT*",
    "tp": "TP-ASD-RRT*",
    "aco": "ACO Trackable-Safe V8",
}


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
    if value is None or reference <= 0:
        return None
    return normalize_metric(value, reference)


def mean_or_none(values: List[float]) -> Optional[float]:
    return statistics.fmean(values) if values else None


def median_or_none(values: List[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def stdev_or_zero(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def fmt(value: Optional[float], digits: int = 6) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    output_dir = Path(args.output_dir)
    config_path = Path(args.config)
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    weights = cfg["weights"]
    refs = cfg["references"]
    constraints = cfg["constraints"]

    rw = float(weights["radiation"])
    tw = float(weights["terrain"])
    ew = float(weights["execution_time"])
    r_ref = float(refs["executed_radiation_map_cost"])
    t_ref = float(refs["executed_terrain_cost"])
    e_ref = float(refs["execution_time_s"])
    failure_penalty = 0.0
    missing_base = float(constraints["missing_metrics_base_score"])
    goal_limit = float(constraints["valid_run_requires_final_goal_error_m_lte"])

    with manifest_path.open(newline="", encoding="utf-8") as f:
        manifest_rows = list(csv.DictReader(f, delimiter="\t"))

    run_rows: List[Dict[str, Any]] = []

    for item in manifest_rows:
        planner_key = item["planner_key"]
        summary_path_text = item.get("summary_json", "").strip()
        summary_path = Path(summary_path_text) if summary_path_text else None
        runner_exit_code = int(item.get("runner_exit_code", "999") or 999)
        summary: Dict[str, Any] = {}

        if summary_path and summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except Exception:
                summary = {}

        radiation = as_float(summary.get("executed_radiation_map_cost"))
        terrain = as_float(summary.get("executed_terrain_cost"))
        execution_time = as_float(summary.get("execution_time_follower_s"))
        dose = as_float(summary.get("dose_during_execution_usv"))
        path_length = as_float(summary.get("executed_path_length_m"))
        planning_time = as_float(summary.get("planning_time_wall_s"))
        final_error = as_float(summary.get("final_goal_error_m"))
        tracking_rms = as_float(summary.get("tracking_rms_error_m"))
        contact_pass = as_bool(summary.get("contact_pass"))
        summary_success = as_bool(summary.get("success"))

        r_norm = clipped_ratio(radiation, r_ref)
        t_norm = clipped_ratio(terrain, t_ref)
        e_norm = clipped_ratio(execution_time, e_ref)

        metrics_complete = all(v is not None for v in (r_norm, t_norm, e_norm))
        goal_ok = final_error is not None and final_error <= goal_limit
        valid = (
            runner_exit_code == 0
            and summary_success
            and contact_pass
            and goal_ok
            and metrics_complete
        )

        if metrics_complete:
            base_score = 100.0 * (
                rw * float(r_norm)
                + tw * float(t_norm)
                + ew * float(e_norm)
            )
        else:
            base_score = missing_base

        penalty = 0.0 if valid else failure_penalty
        final_score = base_score + penalty

        failure_reason = str(summary.get("failure_reason") or "").strip()
        if not valid and not failure_reason:
            reasons = []
            if runner_exit_code != 0:
                reasons.append(f"runner_exit_code_{runner_exit_code}")
            if not summary:
                reasons.append("summary_missing_or_unreadable")
            if summary and not summary_success:
                reasons.append("summary_success_false")
            if summary and not contact_pass:
                reasons.append("contact_pass_false")
            if final_error is None:
                reasons.append("final_goal_error_missing")
            elif not goal_ok:
                reasons.append("final_goal_error_exceeded")
            if not metrics_complete:
                reasons.append("score_metrics_missing")
            failure_reason = ";".join(reasons)

        run_rows.append({
            "repeat_index": int(item["repeat_index"]),
            "planner_key": planner_key,
            "planner_name": PLANNER_NAMES.get(planner_key, planner_key),
            "seed_argument": int(item["seed_argument"]),
            "runner_exit_code": runner_exit_code,
            "summary_success": summary_success,
            "contact_pass": contact_pass,
            "valid_run": valid,
            "final_score": final_score,
            "base_final_score": base_score,
            "constraint_failure_penalty": penalty,
            "executed_radiation_map_cost": radiation,
            "radiation_reference": r_ref,
            "normalized_radiation": r_norm,
            "executed_terrain_cost": terrain,
            "terrain_reference": t_ref,
            "normalized_terrain": t_norm,
            "execution_time_s": execution_time,
            "execution_time_reference_s": e_ref,
            "normalized_execution_time": e_norm,
            "dose_during_execution_usv": dose,
            "executed_path_length_m": path_length,
            "planning_time_s": planning_time,
            "final_goal_error_m": final_error,
            "tracking_rms_error_m": tracking_rms,
            "failure_reason": failure_reason,
            "summary_json": summary_path_text,
        })

    run_rows.sort(key=lambda row: (row["repeat_index"], PLANNER_ORDER.index(row["planner_key"])))

    run_csv = output_dir / "final_score_formal_runs.csv"
    run_fields = list(run_rows[0].keys()) if run_rows else []
    with run_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=run_fields)
        writer.writeheader()
        writer.writerows(run_rows)

    planner_rows: List[Dict[str, Any]] = []

    for planner_key in PLANNER_ORDER:
        rows = [r for r in run_rows if r["planner_key"] == planner_key]
        scores = [float(r["final_score"]) for r in rows]
        base_scores = [float(r["base_final_score"]) for r in rows]
        valid_rows = [r for r in rows if r["valid_run"]]
        valid_scores = [float(r["final_score"]) for r in valid_rows]

        raw_r = [float(r["executed_radiation_map_cost"]) for r in rows
                 if r["executed_radiation_map_cost"] is not None]
        raw_t = [float(r["executed_terrain_cost"]) for r in rows
                 if r["executed_terrain_cost"] is not None]
        raw_e = [float(r["execution_time_s"]) for r in rows
                 if r["execution_time_s"] is not None]
        doses = [float(r["dose_during_execution_usv"]) for r in rows
                 if r["dose_during_execution_usv"] is not None]

        attempted = len(rows)
        valid_count = len(valid_rows)
        failed_count = attempted - valid_count

        planner_rows.append({
            "rank": 0,
            "planner_key": planner_key,
            "planner_name": PLANNER_NAMES[planner_key],
            "attempted_runs": attempted,
            "valid_runs": valid_count,
            "failed_runs": failed_count,
            "success_rate_percent": (100.0 * valid_count / attempted) if attempted else 0.0,
            "mean_final_score_all_attempts": mean_or_none(scores),
            "std_final_score_all_attempts": stdev_or_zero(scores),
            "median_final_score_all_attempts": median_or_none(scores),
            "minimum_final_score": min(scores) if scores else None,
            "maximum_final_score": max(scores) if scores else None,
            "mean_base_score_all_attempts": mean_or_none(base_scores),
            "mean_final_score_valid_only": mean_or_none(valid_scores),
            "mean_executed_radiation_map_cost": mean_or_none(raw_r),
            "mean_executed_terrain_cost": mean_or_none(raw_t),
            "mean_execution_time_s": mean_or_none(raw_e),
            "mean_dose_during_execution_usv": mean_or_none(doses),
        })

    planner_rows.sort(key=lambda row: (
        float("inf") if row["mean_final_score_all_attempts"] is None
        else row["mean_final_score_all_attempts"]
    ))
    for index, row in enumerate(planner_rows, start=1):
        row["rank"] = index

    summary_csv = output_dir / "final_score_planner_summary.csv"
    summary_fields = list(planner_rows[0].keys()) if planner_rows else []
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(planner_rows)

    attempted_by_planner = {
        key: sum(1 for r in run_rows if r["planner_key"] == key)
        for key in PLANNER_ORDER
    }

    batch_json = {
        "methodology": cfg,
        "manifest": str(manifest_path),
        "run_csv": str(run_csv),
        "planner_summary_csv": str(summary_csv),
        "attempted_by_planner": attempted_by_planner,
        "ranking_complete": all(attempted_by_planner[k] > 0 for k in PLANNER_ORDER),
        "all_runs_valid": all(r["valid_run"] for r in run_rows) if run_rows else False,
        "planner_summary": planner_rows,
    }
    batch_json_path = output_dir / "final_score_batch_summary.json"
    batch_json_path.write_text(json.dumps(batch_json, indent=2), encoding="utf-8")

    print()
    print("=" * 132)
    print("R3 FORMAL FINAL-SCORE SUMMARY")
    print("Ranking uses the mean penalized final score across every attempted run. Lower is better.")
    print("=" * 132)
    print(
        f"{'RANK':<6}{'PLANNER':<22}{'MEAN FINAL':>14}{'STD':>12}"
        f"{'VALID/TOTAL':>15}{'SUCCESS %':>13}{'MEAN RAD':>13}"
        f"{'MEAN TERRAIN':>15}{'MEAN TIME':>12}"
    )
    for row in planner_rows:
        valid_total = f"{row['valid_runs']}/{row['attempted_runs']}"
        print(
            f"{row['rank']:<6}"
            f"{row['planner_name']:<22}"
            f"{fmt(row['mean_final_score_all_attempts']):>14}"
            f"{fmt(row['std_final_score_all_attempts']):>12}"
            f"{valid_total:>15}"
            f"{fmt(row['success_rate_percent'], 2):>13}"
            f"{fmt(row['mean_executed_radiation_map_cost']):>13}"
            f"{fmt(row['mean_executed_terrain_cost']):>15}"
            f"{fmt(row['mean_execution_time_s'], 3):>12}"
        )
    print("=" * 132)
    print(f"Run CSV:     {run_csv}")
    print(f"Summary CSV: {summary_csv}")
    print(f"Summary JSON:{batch_json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
