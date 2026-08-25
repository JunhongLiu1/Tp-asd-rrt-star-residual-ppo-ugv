#!/usr/bin/env python3
"""Summarise the supplemental turn/recovery bags in a reproducible report.

This intentionally reports the pre-PID turn bags separately from the PID A/B
bags: they validate direction and safety behaviour, but are not evidence of a
PID improvement.
"""

import json
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "acceptance_logs" / "pid_supplemental_20260822"


def read_bag(bag_dir):
    db = next(Path(bag_dir).glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    topics = {row[0]: (row[1], row[2]) for row in con.execute(
        "SELECT id, name, type FROM topics")}
    rows = []
    classes = {}
    for topic_id, stamp, raw in con.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
        topic, type_name = topics[topic_id]
        if topic not in {"/cmd_vel", "/control/base_cmd",
                         "/control/pure_pursuit_metrics",
                         "/tp_asd_rrt_star_cpp_follower_status",
                         "/e_stop"}:
            continue
        classes.setdefault(type_name, get_message(type_name))
        msg = deserialize_message(raw, classes[type_name])
        row = {"stamp": stamp, "topic": topic}
        if hasattr(msg, "linear"):
            row.update(linear=float(msg.linear.x), angular=float(msg.angular.z))
        elif topic == "/control/pure_pursuit_metrics":
            for field in ("lateral_error_m", "heading_error_rad", "saturated"):
                row[field] = getattr(msg, field)
        elif hasattr(msg, "data"):
            row["data"] = msg.data
        rows.append(row)
    con.close()
    return rows


def commands(rows, topic):
    return [r for r in rows if r["topic"] == topic and "linear" in r]


def summary(name, bag_dir, kind):
    rows = read_bag(bag_dir)
    base = commands(rows, "/control/base_cmd")
    final = commands(rows, "/cmd_vel")
    metrics = [r for r in rows if r["topic"] == "/control/pure_pursuit_metrics"]
    statuses = [r.get("data", "") for r in rows
                if r["topic"] == "/tp_asd_rrt_star_cpp_follower_status"]
    status_kinds = sorted({value.split(":", 1)[0] for value in statuses})
    source = {
        "name": name,
        "kind": kind,
        "bag": str(Path(bag_dir).relative_to(ROOT)),
        "duration_sec": ((max(r["stamp"] for r in rows) -
                           min(r["stamp"] for r in rows)) / 1e9) if rows else 0.0,
        "base_cycles": len(base),
        "final_cycles": len(final),
        "tracking_cycles": len(metrics),
        "peak_linear_mps": max((abs(r["linear"]) for r in final), default=0.0),
        "peak_angular_rps": max((abs(r["angular"]) for r in final), default=0.0),
        "reverse_commands": sum(r["linear"] < -1e-6 for r in final),
        "terminal_zero_commands": sum(
            abs(r["linear"]) < 1e-9 and abs(r["angular"]) < 1e-9
            for r in final[-20:]),
        "max_lateral_error_m": max(
            (abs(r["lateral_error_m"]) for r in metrics), default=0.0),
        "max_heading_error_rad": max(
            (abs(r["heading_error_rad"]) for r in metrics), default=0.0),
        "mean_lateral_error_m": (
            sum(abs(r["lateral_error_m"]) for r in metrics) / len(metrics)
            if metrics else 0.0),
        "mean_heading_error_rad": (
            sum(abs(r["heading_error_rad"]) for r in metrics) / len(metrics)
            if metrics else 0.0),
        "saturated_cycles": sum(bool(r.get("saturated", False)) for r in metrics),
        "status_kinds": status_kinds,
    }
    if name == "radiation_estop":
        events = [r for r in rows if r["topic"] == "/e_stop"]
        estop = [r for r in events if r.get("data")]
        release = [r for r in events if not r.get("data")]
        source["estop_true_events"] = len(estop)
        if estop:
            end = next((r["stamp"] for r in release
                        if r["stamp"] > estop[0]["stamp"]), max(r["stamp"] for r in rows))
            after = [r for r in final if estop[0]["stamp"] <= r["stamp"] <= end]
            source["post_estop_nonzero_commands"] = sum(
                abs(r["linear"]) > 1e-9 or abs(r["angular"]) > 1e-9
                for r in after)
    return source


def main():
    cases = [
        ("left_turn", ROOT / "acceptance_logs/pre_pid_20260822/left_turn_bag", "pre_pid_safety"),
        ("right_turn", ROOT / "acceptance_logs/pre_pid_20260822/right_turn_bag", "pre_pid_safety"),
        ("radiation_estop", ROOT / "acceptance_logs/pre_pid_20260822/radiation_estop_bag", "pre_pid_safety"),
        ("pid_off_straight", ROOT / "acceptance_logs/pid_tuning_20260822/baseline_strict_bag", "pid_ab"),
        ("pi_straight", ROOT / "acceptance_logs/pid_tuning_20260822/pi_strict_bag", "pid_ab"),
    ]
    for name, variant in (("left_pid_off3", "pid_curve_ab"),
                          ("left_pid_on", "pid_curve_ab"),
                          ("dem_left_pid_off2", "pid_dem_curve_ab"),
                          ("dem_left_pid_on", "pid_dem_curve_ab"),
                          ("dem_right_pid_off", "pid_dem_curve_ab"),
                          ("dem_right_pid_on", "pid_dem_curve_ab"),
                          ("dem_long_pid_off", "pid_dem_long_ab"),
                          ("dem_long_pid_on", "pid_dem_long_ab")):
        bag = ROOT / "acceptance_logs" / "pid_supplemental_20260822" / f"{name}_bag"
        if bag.is_dir() and list(bag.glob("*.db3")):
            cases.append((name, bag, variant))
    report = {"date": "2026-08-22", "cases": [summary(*case) for case in cases]}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
