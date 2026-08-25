#!/usr/bin/env python3
"""Summarise the gated difficult-scenario PI A/B runs."""

import csv
import json
import math
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "acceptance_logs" / "pi_difficult_scenarios_20260822"
SELECTED = {
    "lateral_offset": {"off": "pid_off_valid3", "on": "pid_on_valid2"},
    "heading_offset": {"off": "pid_off_valid2", "on": "pid_on_valid2"},
    "turn_90": {"off": "pid_off_valid1", "on": "pid_on_valid1"},
    "s_curve": {"off": "pid_off_valid1", "on": "pid_on_valid1"},
    "medium_path": {"off": "pid_off_valid1", "on": "pid_on_valid1"},
    "estop_replan_recover": {"off": "pid_off_valid1", "on": "pid_on_valid1"},
}


def stats(values):
    values = [abs(float(value)) for value in values if math.isfinite(float(value))]
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values),
        "max": max(values),
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95": sorted(values)[round(0.95 * (len(values) - 1))],
    }


def main():
    rows = []
    for scenario, variants in SELECTED.items():
        for pid, directory in variants.items():
            folder = DATA / scenario / directory
            result = json.loads((folder / "result.json").read_text())
            with (folder / "samples.csv").open() as stream:
                samples = list(csv.DictReader(stream))
            lateral = stats(row["lateral_error"] for row in samples)
            heading = stats(row["heading_error"] for row in samples)
            replanned_nonzero = sum(
                row["replanned"] == "1" and
                (abs(float(row["final_linear"])) > 1e-4 or
                 abs(float(row["final_angular"])) > 1e-4)
                for row in samples)
            row = {
                "scenario": scenario, "pid": pid, "run_count": 1,
                "completed": result["completed"],
                "duration_sec": result["duration_sec"],
                "initial_lateral_error_m": result["initial_lateral_error_m"],
                "initial_heading_error_rad": result["initial_heading_error_rad"],
                **{"lateral_" + key: value for key, value in lateral.items()},
                **{"heading_" + key: value for key, value in heading.items()},
                "peak_final_linear": result["peak_final_linear"],
                "peak_final_angular": result["peak_final_angular"],
                "negative_command_count": result["negative_command_count"],
                "estop_nonzero_count": result["estop_nonzero_count"],
                "replanned_nonzero_count": replanned_nonzero,
                "unsafe_boundary_stop": result["unsafe_boundary_stop"],
                "source": str(folder.relative_to(ROOT)),
            }
            rows.append(row)
    with (DATA / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (DATA / "summary.json").write_text(json.dumps({"runs": rows}, indent=2) + "\n")
    print(json.dumps({"runs": rows}, indent=2))


if __name__ == "__main__":
    main()
