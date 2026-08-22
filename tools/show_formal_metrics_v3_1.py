#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def latest_summary(root: Path) -> Path:
    candidates = list(root.rglob("summary.json"))
    if not candidates:
        raise SystemExit(f"No summary.json found below {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


parser = argparse.ArgumentParser()
parser.add_argument("summary", nargs="?", help="Optional summary.json path")
parser.add_argument(
    "--root",
    default="~/terrain_radiation_ws/formal_experiments_v3",
)
args = parser.parse_args()

path = Path(args.summary).expanduser() if args.summary else latest_summary(
    Path(args.root).expanduser()
)
data = json.loads(path.read_text(encoding="utf-8"))

print("=" * 74)
print(path)
print("=" * 74)
for key in (
    "success",
    "scenario",
    "planner_name",
    "seed",
    "planned_path_length_m",
    "executed_path_length_m",
    "planned_terrain_cost",
    "executed_terrain_cost",
    "planned_radiation_map_cost",
    "executed_radiation_map_cost",
    "dose_during_execution_usv",
    "execution_time_follower_s",
    "final_goal_error_m",
    "tracking_rms_error_m",
    "terrain_valid_sample_count",
    "terrain_out_of_bounds_count",
    "radiation_valid_sample_count",
    "radiation_out_of_bounds_count",
    "contact_pass",
):
    print(f"{key:42s}: {data.get(key)}")
