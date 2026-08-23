#!/usr/bin/env python3
"""Build source-linked compact tables for the final project results."""

import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "analysis_datasets" / "final_results_20260822"


def read_json(relative):
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    planner_path = "acceptance_logs/tp_asd_20260822/synthetic_ab_30seeds.json"
    rl_path = "acceptance_logs/rl_ppo_20260822/retrain50k_ab_evaluation_100seeds.json"
    gazebo_path = "acceptance_logs/residual_gazebo_matrix_20260822/summary.json"
    pid_path = "acceptance_logs/pid_supplemental_20260822/report.json"
    planner = read_json(planner_path)
    rl = read_json(rl_path)
    gazebo = read_json(gazebo_path)
    pid = read_json(pid_path)

    planner_rows = []
    for scenario, result in planner["results"].items():
        for policy in ("legacy", "adaptive"):
            value = result[policy]
            planner_rows.append({
                "scenario": scenario,
                "variant": policy,
                "successes": int(value["success"].split("/")[0]),
                "runs": int(value["success"].split("/")[1]),
                "mean_cost": value["mean_cost"],
                "mean_time_us": value["mean_time_us"],
                "mean_iterations": value["mean_iterations"],
                "mean_nodes": value["mean_nodes"],
                "source": planner_path,
            })
    write_csv(OUTPUT / "planner_paired_summary.csv", planner_rows)

    controller_rows = []
    for key, label in (("zero_baseline", "zero_residual"), ("candidate", "ppo_50k")):
        value = rl[key]
        controller_rows.append({
            "environment": "deterministic_surrogate",
            "variant": label,
            "runs": value["episodes"],
            "success_rate": value["success_rate"],
            "mean_duration": value["mean_episode_length"],
            "duration_unit": "control_steps",
            "mean_return": value["mean_return"],
            "negative_commands": 0,
            "safety_terminations": value["safety_terminations"],
            "worker_fault_latches": 0,
            "source": rl_path,
        })
    for label in ("zero", "ppo"):
        value = gazebo["policies"][label]
        controller_rows.append({
            "environment": "gazebo_hard_terrain_fixed_path_stress",
            "variant": label,
            "runs": value["runs"],
            "success_rate": value["completion_rate"],
            "mean_duration": value["mean_duration_sec"],
            "duration_unit": "seconds",
            "mean_return": "",
            "negative_commands": value["negative_command_count"],
            "safety_terminations": value["unsafe_boundary_stops"],
            "worker_fault_latches": value["rl_fault_latches"],
            "source": gazebo_path,
        })
    write_csv(OUTPUT / "controller_stage_summary.csv", controller_rows)

    pid_rows = []
    for case in pid["cases"]:
        if case["name"] not in ("pid_off_straight", "pi_straight"):
            continue
        pid_rows.append({
            "variant": case["name"],
            "duration_sec": case["duration_sec"],
            "tracking_cycles": case["tracking_cycles"],
            "max_lateral_error_m": case["max_lateral_error_m"],
            "max_heading_error_rad": case["max_heading_error_rad"],
            "reverse_commands": case["reverse_commands"],
            "terminal_zero_commands": case["terminal_zero_commands"],
            "source": pid_path,
        })
    write_csv(OUTPUT / "pid_straight_ab.csv", pid_rows)

    claims = {
        "dataset_label": "FINAL_RESULTS_WITH_STAGE_SEPARATION",
        "deployable": False,
        "claims": [
            {"stage": "planner_synthetic", "accepted": True, "source": planner_path},
            {"stage": "ppo_surrogate", "accepted": rl["acceptance_passed"], "source": rl_path},
            {"stage": "ppo_gazebo_short_smoke", "accepted": True,
             "source": "acceptance_logs/rl_ppo_20260822/learned_gazebo_smoke_valid"},
            {"stage": "ppo_gazebo_stress_performance", "accepted": False, "source": gazebo_path},
            {"stage": "physical_robot_deployment", "accepted": False, "source": "not_tested"},
        ],
    }
    (OUTPUT / "claims.json").write_text(
        json.dumps(claims, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = {
        "dataset": "final_results_20260822",
        "generated_from": [planner_path, rl_path, gazebo_path, pid_path],
        "files": {},
    }
    for path in sorted(OUTPUT.glob("*")):
        if path.name == "manifest.json" or not path.is_file():
            continue
        manifest["files"][path.name] = {
            "bytes": path.stat().st_size,
            "sha256": digest(path),
        }
    (OUTPUT / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
