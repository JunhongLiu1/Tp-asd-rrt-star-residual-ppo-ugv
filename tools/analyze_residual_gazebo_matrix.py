#!/usr/bin/env python3
"""Validate and summarize the paired residual-control Gazebo matrix."""

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "acceptance_logs" / "residual_gazebo_matrix_20260822"
SCENARIOS = (
    "lateral_offset", "heading_offset", "turn_90", "s_curve",
    "medium_path", "estop_replan_recover",
)
POLICIES = ("zero", "ppo")


def load_result(policy, scenario):
    path = DATA / policy / scenario / "result.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    if data["policy"] != policy or data["scenario"] != scenario:
        raise RuntimeError("result identity mismatch: " + str(path))
    if not math.isfinite(float(data["duration_sec"])) or data["duration_sec"] < 0:
        raise RuntimeError("invalid duration: " + str(path))
    return path, data


def main():
    rows = []
    for scenario in SCENARIOS:
        for policy in POLICIES:
            path, data = load_result(policy, scenario)
            rows.append({
                "scenario": scenario,
                "policy": policy,
                "completed": int(data["completed"]),
                "duration_sec": data["duration_sec"],
                "samples": data["samples"],
                "lateral_rms_m": data["lateral_rms_m"],
                "lateral_max_m": data["lateral_max_m"],
                "heading_rms_rad": data["heading_rms_rad"],
                "heading_max_rad": data["heading_max_rad"],
                "peak_final_linear_mps": data["peak_final_linear"],
                "peak_final_angular_rps": data["peak_final_angular"],
                "negative_command_count": data["negative_command_count"],
                "estop_nonzero_count": data["estop_nonzero_count"],
                "unsafe_boundary_stop": int(data["unsafe_boundary_stop"]),
                "rl_fault_latched": int(data["rl_fault_latched"]),
                "terminal_zero_observed": int(data["terminal_zero"]),
                "source": str(path.relative_to(ROOT)),
            })

    summary = {"matrix_valid": True, "runs": len(rows), "policies": {}}
    for policy in POLICIES:
        selected = [row for row in rows if row["policy"] == policy]
        summary["policies"][policy] = {
            "runs": len(selected),
            "completed": sum(row["completed"] for row in selected),
            "completion_rate": sum(row["completed"] for row in selected) / len(selected),
            "negative_command_count": sum(row["negative_command_count"] for row in selected),
            "estop_nonzero_count": sum(row["estop_nonzero_count"] for row in selected),
            "unsafe_boundary_stops": sum(row["unsafe_boundary_stop"] for row in selected),
            "rl_fault_latches": sum(row["rl_fault_latched"] for row in selected),
            "terminal_zero_observed_runs": sum(
                row["terminal_zero_observed"] for row in selected
            ),
            "mean_duration_sec": sum(row["duration_sec"] for row in selected) / len(selected),
        }
    summary["paired_completion_delta_ppo_minus_zero"] = (
        summary["policies"]["ppo"]["completion_rate"] -
        summary["policies"]["zero"]["completion_rate"]
    )
    summary["interpretation"] = (
        "Control-focused fixed-path Gazebo stress matrix; it does not replace "
        "the separate online-planner and synthetic TP-ASD tests. A time-limit "
        "run is retained as a performance failure, not discarded."
    )

    with (DATA / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    (DATA / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
