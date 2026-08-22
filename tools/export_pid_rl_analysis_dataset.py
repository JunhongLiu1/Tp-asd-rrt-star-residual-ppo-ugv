#!/usr/bin/env python3
"""Export compact CSV tables from the accepted PID/RL evidence."""

import csv
import hashlib
import json
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "analysis_datasets" / "pid_rl_20260822"
EVALUATION = (
    ROOT / "acceptance_logs" / "rl_ppo_20260822" /
    "retrain50k_ab_evaluation_100seeds.json"
)
BAG = (
    ROOT / "acceptance_logs" / "rl_ppo_20260822" /
    "learned_gazebo_smoke_valid" / "learned_policy_bag"
)


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def export_offline_evaluation(data):
    rows = []
    for policy_key, policy_label in (
        ("candidate", "ppo_50k"),
        ("zero_baseline", "zero_residual"),
    ):
        record = data[policy_key]
        for index, (length, episode_return) in enumerate(zip(
                record["episode_lengths"], record["episode_returns"])):
            rows.append({
                "seed": record["seed"] + index,
                "policy": policy_label,
                "episode_length_steps": length,
                "episode_return": episode_return,
                "goal_reached": 1,
                "time_limit": 0,
                "safety_termination": 0,
            })
    write_csv(
        OUTPUT / "offline_episode_results.csv",
        ["seed", "policy", "episode_length_steps", "episode_return",
         "goal_reached", "time_limit", "safety_termination"],
        rows,
    )

    component_rows = []
    for policy_key, policy_label in (
        ("candidate", "ppo_50k"),
        ("zero_baseline", "zero_residual"),
    ):
        for component, value in data[policy_key][
                "reward_components_mean_per_episode"].items():
            component_rows.append({
                "policy": policy_label,
                "reward_component": component,
                "mean_per_episode": value,
            })
    write_csv(
        OUTPUT / "offline_reward_components.csv",
        ["policy", "reward_component", "mean_per_episode"],
        component_rows,
    )


def export_experiment_summary(data):
    rows = [
        {
            "experiment": "pid_ab", "variant": "pid_off",
            "goal_time_sec": 20.07, "peak_linear_mps": 0.0341,
            "reverse_commands": 0, "terminal_zero": 1,
            "max_lateral_error_m": 0.00011,
            "max_heading_error_rad": 0.00694,
            "saturated_cycles": 0, "tracking_cycles": 139,
            "source": "acceptance_logs/pid_tuning_20260822/baseline_strict_bag",
        },
        {
            "experiment": "pid_ab", "variant": "frozen_pi",
            "goal_time_sec": 19.97, "peak_linear_mps": 0.0580,
            "reverse_commands": 0, "terminal_zero": 1,
            "max_lateral_error_m": 0.000088,
            "max_heading_error_rad": 0.00606,
            "saturated_cycles": 28, "tracking_cycles": 162,
            "source": "acceptance_logs/pid_tuning_20260822/pi_strict_bag",
        },
        {
            "experiment": "offline_rl", "variant": "zero_residual",
            "goal_time_sec": "", "peak_linear_mps": "",
            "reverse_commands": 0, "terminal_zero": 1,
            "max_lateral_error_m": "", "max_heading_error_rad": "",
            "saturated_cycles": 0, "tracking_cycles": "",
            "source": str(EVALUATION.relative_to(ROOT)),
        },
        {
            "experiment": "offline_rl", "variant": "ppo_50k",
            "goal_time_sec": "", "peak_linear_mps": "",
            "reverse_commands": 0, "terminal_zero": 1,
            "max_lateral_error_m": "", "max_heading_error_rad": "",
            "saturated_cycles": 0, "tracking_cycles": "",
            "source": str(EVALUATION.relative_to(ROOT)),
        },
        {
            "experiment": "gazebo_rl_smoke", "variant": "ppo_50k",
            "goal_time_sec": 7.8, "peak_linear_mps": 0.07990,
            "reverse_commands": 0, "terminal_zero": 1,
            "max_lateral_error_m": "", "max_heading_error_rad": "",
            "saturated_cycles": "", "tracking_cycles": "",
            "source": str(BAG.relative_to(ROOT)),
        },
    ]
    fields = list(rows[0])
    write_csv(OUTPUT / "experiment_summary.csv", fields, rows)


def export_bag_timeseries():
    databases = sorted(BAG.glob("*.db3"))
    if len(databases) != 1:
        raise RuntimeError("expected exactly one db3 file in {}".format(BAG))
    connection = sqlite3.connect("file:{}?mode=ro".format(databases[0]), uri=True)
    topic_rows = connection.execute(
        "SELECT id, name, type FROM topics").fetchall()
    topic_map = {
        topic_id: (name, message_type)
        for topic_id, name, message_type in topic_rows
    }
    selected = {
        "/control/pid_baseline_cmd",
        "/control/base_cmd",
        "/cmd_vel",
        "/control/pure_pursuit_metrics",
        "/tp_asd_rrt_star_cpp_follower_status",
        "/tp_asd_rrt_star_cpp_safety_status",
        "/control/residual_rl_status",
        "/e_stop",
        "/odometry/filtered",
    }
    rows = []
    messages = connection.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    message_classes = {}
    for topic_id, timestamp, raw in messages:
        topic, message_type = topic_map[topic_id]
        if topic not in selected:
            continue
        if message_type not in message_classes:
            message_classes[message_type] = get_message(message_type)
        message = deserialize_message(raw, message_classes[message_type])
        row = {
            "timestamp_ns": timestamp, "time_from_start_sec": 0.0,
            "topic": topic, "linear_x": "", "angular_z": "",
            "status": "", "e_stop": "", "odom_x": "", "odom_y": "",
            "lateral_error_m": "", "heading_error_rad": "",
            "reference_linear_mps": "", "actual_linear_mps": "",
            "reference_angular_rps": "", "actual_angular_rps": "",
            "saturated": "", "terrain_impedance": "",
        }
        if hasattr(message, "linear") and hasattr(message, "angular"):
            row["linear_x"] = message.linear.x
            row["angular_z"] = message.angular.z
        elif topic == "/odometry/filtered":
            row["odom_x"] = message.pose.pose.position.x
            row["odom_y"] = message.pose.pose.position.y
            row["linear_x"] = message.twist.twist.linear.x
            row["angular_z"] = message.twist.twist.angular.z
        elif topic == "/control/pure_pursuit_metrics":
            for field in (
                "lateral_error_m", "heading_error_rad",
                "reference_linear_mps", "actual_linear_mps",
                "reference_angular_rps", "actual_angular_rps",
                "saturated", "terrain_impedance",
            ):
                row[field] = getattr(message, field)
        elif topic == "/e_stop":
            row["e_stop"] = int(message.data)
        elif hasattr(message, "data"):
            row["status"] = message.data
        rows.append(row)
    connection.close()
    if rows:
        start = min(row["timestamp_ns"] for row in rows)
        for row in rows:
            row["time_from_start_sec"] = (row["timestamp_ns"] - start) / 1e9
    fields = list(rows[0]) if rows else []
    write_csv(OUTPUT / "gazebo_control_timeseries.csv", fields, rows)
    return len(rows)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with EVALUATION.open(encoding="utf-8") as stream:
        evaluation = json.load(stream)
    export_offline_evaluation(evaluation)
    export_experiment_summary(evaluation)
    timeseries_rows = export_bag_timeseries()
    manifest = {
        "dataset": "pid_rl_20260822",
        "offline_episode_rows": 200,
        "gazebo_timeseries_rows": timeseries_rows,
        "checkpoint": evaluation["checkpoint"],
        "checkpoint_sha256": (
            "8233e2504909a97844cb3f97c72ab7c7756b1762a997a48168904256c2f1c742"
        ),
        "source_evaluation": str(EVALUATION.relative_to(ROOT)),
        "source_bag": str(BAG.relative_to(ROOT)),
        "files": {},
    }
    for path in sorted(OUTPUT.glob("*.csv")):
        manifest["files"][path.name] = {
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
        }
    with (OUTPUT / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
