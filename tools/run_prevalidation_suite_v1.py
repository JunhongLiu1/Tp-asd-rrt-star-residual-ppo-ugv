#!/usr/bin/env python3
import argparse
import csv
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

SCENARIOS = ("H_B1_balanced", "H_R1_radiation", "H_T1_terrain")
PLANNERS = ("asd", "tp", "aco")


def latest_summary(root: Path, scenario: str, planner: str) -> Optional[Path]:
    folder = root / scenario / planner
    if not folder.is_dir():
        return None
    candidates = sorted(folder.glob("run_*/summary.json"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Run 3x3 formal pre-validation suite")
    parser.add_argument("--workspace", default="~/terrain_radiation_ws")
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--stop-on-failure", action="store_true")
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser().resolve()
    runner = workspace / "tools/formal_experiment_runner_v3.py"
    if not runner.is_file():
        print(f"ERROR: runner not found: {runner}", file=sys.stderr)
        return 2

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suite_dir = workspace / "formal_experiments_v3" / f"prevalidation_suite_{stamp}"
    suite_dir.mkdir(parents=True, exist_ok=False)
    records: List[Dict[str, object]] = []
    overall = 0

    for scenario in SCENARIOS:
        for planner in PLANNERS:
            run_id = f"pre_{scenario}_{planner}"
            print("\n" + "=" * 78)
            print(f"PREVALIDATION: scenario={scenario} planner={planner} seed={args.seed}")
            print("=" * 78, flush=True)
            command = [
                sys.executable,
                str(runner),
                "--planner", planner,
                "--scenario", scenario,
                "--run-id", run_id,
                "--seed", str(args.seed),
                "--workspace", str(workspace),
            ]
            result = subprocess.run(command, check=False)
            summary_path = latest_summary(
                workspace / "formal_experiments_v3", scenario, planner
            )
            summary: Dict[str, object] = {}
            if summary_path and summary_path.is_file():
                try:
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                except Exception as error:
                    summary = {"summary_read_error": str(error)}

            record = {
                "scenario": scenario,
                "planner": planner,
                "seed": args.seed,
                "return_code": result.returncode,
                "success": bool(summary.get("success", False)),
                "failure_reason": summary.get("failure_reason", ""),
                "planning_time_wall_s": summary.get("planning_time_wall_s"),
                "execution_time_follower_s": summary.get("execution_time_follower_s"),
                "executed_path_length_m": summary.get("executed_path_length_m"),
                "dose_during_execution_usv": summary.get("dose_during_execution_usv"),
                "executed_terrain_cost": summary.get("executed_terrain_cost"),
                "final_goal_error_m": summary.get("final_goal_error_m"),
                "contact_pass": summary.get("contact_pass"),
                "terrain_valid_sample_count": summary.get("terrain_valid_sample_count"),
                "terrain_out_of_bounds_count": summary.get("terrain_out_of_bounds_count"),
                "summary_path": str(summary_path) if summary_path else "",
            }
            records.append(record)
            (suite_dir / "progress.json").write_text(
                json.dumps(records, indent=2), encoding="utf-8"
            )

            passed = result.returncode == 0 and bool(record["success"])
            print(f"[SUITE] {'PASS' if passed else 'FAIL'}: {scenario} / {planner}")
            if not passed:
                overall = 1
                if args.stop_on_failure:
                    break
        if overall and args.stop_on_failure:
            break

    json_path = suite_dir / "prevalidation_summary.json"
    csv_path = suite_dir / "prevalidation_summary.csv"
    json_path.write_text(json.dumps(records, indent=2), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(records[0].keys()) if records else [])
        if records:
            writer.writeheader()
            writer.writerows(records)

    print("\n" + "=" * 78)
    print(f"PREVALIDATION SUITE {'PASS' if overall == 0 and len(records) == 9 else 'INCOMPLETE/FAIL'}")
    print(f"JSON: {json_path}")
    print(f"CSV:  {csv_path}")
    print("=" * 78)
    return overall if len(records) == 9 else 2


if __name__ == "__main__":
    sys.exit(main())
