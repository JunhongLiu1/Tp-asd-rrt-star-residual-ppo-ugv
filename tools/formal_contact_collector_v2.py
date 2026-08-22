#!/usr/bin/env python3
import argparse
import json
import math
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
HIST_STEP_M = 0.00001
HIST_MAX_M = 0.050
HIST_BINS = int(HIST_MAX_M / HIST_STEP_M) + 1

STOP_REQUESTED = False


def request_stop(signum, frame):
    del signum, frame
    global STOP_REQUESTED
    STOP_REQUESTED = True


class DepthStats:
    def __init__(self) -> None:
        self.contact_blocks = 0
        self.points = 0
        self.minimum = math.inf
        self.maximum = 0.0
        self.total = 0.0
        self.histogram = [0] * HIST_BINS
        self.above_1mm = 0
        self.above_2mm = 0
        self.above_3mm = 0
        self.above_5mm = 0
        self.above_10mm = 0

    def add_block(self, depths: List[float]) -> None:
        self.contact_blocks += 1
        for depth in depths:
            if not math.isfinite(depth) or depth < 0.0:
                continue
            self.points += 1
            self.total += depth
            self.minimum = min(self.minimum, depth)
            self.maximum = max(self.maximum, depth)
            index = min(int(depth / HIST_STEP_M), HIST_BINS - 1)
            self.histogram[index] += 1
            self.above_1mm += int(depth > 0.001)
            self.above_2mm += int(depth > 0.002)
            self.above_3mm += int(depth > 0.003)
            self.above_5mm += int(depth > 0.005)
            self.above_10mm += int(depth > 0.010)

    def percentile(self, probability: float) -> Optional[float]:
        if self.points == 0:
            return None
        target = max(1, int(math.ceil(probability * self.points)))
        cumulative = 0
        for index, count in enumerate(self.histogram):
            cumulative += count
            if cumulative >= target:
                return index * HIST_STEP_M
        return self.maximum

    def as_dict(self) -> Dict[str, object]:
        if self.points == 0:
            return {
                "contact_blocks": self.contact_blocks,
                "points": 0,
                "minimum_m": None,
                "mean_m": None,
                "median_m": None,
                "p95_m": None,
                "p99_m": None,
                "maximum_m": None,
                "above_1mm": 0,
                "above_2mm": 0,
                "above_3mm": 0,
                "above_5mm": 0,
                "above_10mm": 0,
            }
        return {
            "contact_blocks": self.contact_blocks,
            "points": self.points,
            "minimum_m": self.minimum,
            "mean_m": self.total / self.points,
            "median_m": self.percentile(0.50),
            "p95_m": self.percentile(0.95),
            "p99_m": self.percentile(0.99),
            "maximum_m": self.maximum,
            "above_1mm": self.above_1mm,
            "above_2mm": self.above_2mm,
            "above_3mm": self.above_3mm,
            "above_5mm": self.above_5mm,
            "above_10mm": self.above_10mm,
        }


class ContactCollector:
    def __init__(self, topic: str, output_dir: Path, timeline_period: float) -> None:
        self.topic = topic
        self.output_dir = output_dir
        self.timeline_period = max(0.001, timeline_period)
        self.stats = {name: DepthStats() for name in WHEELS}
        self.stats["chassis"] = DepthStats()
        self.overall = DepthStats()
        self.message_count = 0
        self.terrain_blocks = 0
        self.unclassified_blocks = 0
        self.support_message_counts = {str(i): 0 for i in range(5)}
        self.support_duration_s = {str(i): 0.0 for i in range(5)}
        self.wheel_message_counts = {name: 0 for name in WHEELS}
        self.chassis_message_count = 0
        self.total_timed_duration_s = 0.0
        self.longest_low_support_s = 0.0
        self.current_low_support_s = 0.0
        self.longest_chassis_contact_s = 0.0
        self.current_chassis_contact_s = 0.0
        self.previous_time: Optional[float] = None
        self.previous_support: Optional[int] = None
        self.previous_chassis = False
        self.last_timeline_time: Optional[float] = None
        self.reset_message()

    def reset_message(self) -> None:
        self.message_started = False
        self.message_sec: Optional[int] = None
        self.message_nsec: Optional[int] = None
        self.message_time: Optional[float] = None
        self.message_wheels: Set[str] = set()
        self.message_chassis = False
        self.message_max_depth = {name: 0.0 for name in WHEELS}
        self.message_point_count = {name: 0 for name in WHEELS}

    @staticmethod
    def quoted_value(line: bytes) -> str:
        first = line.find(b'"')
        if first < 0:
            return ""
        second = line.find(b'"', first + 1)
        if second < 0:
            return ""
        return line[first + 1:second].decode("utf-8", errors="replace")

    @staticmethod
    def number_value(line: bytes) -> Optional[float]:
        separator = line.find(b":")
        if separator < 0:
            return None
        try:
            return float(line[separator + 1:].strip())
        except ValueError:
            return None

    @staticmethod
    def classify(collision1: str, collision2: str) -> str:
        combined = (collision1 + " " + collision2).lower()
        if "terrain_collision" not in combined and "dem_inspired_terrain" not in combined:
            return "nonterrain"
        for wheel in WHEELS:
            if wheel + "_wheel" in combined:
                return wheel
        chassis_terms = (
            "base_collision", "chassis_collision", "base_link_collision",
            "top_chassis", "bottom_chassis", "base_footprint", "husky::base_link",
        )
        if any(term in combined for term in chassis_terms):
            return "chassis"
        return "unclassified"

    def process_contact(self, collision1: str, collision2: str, depths: List[float]) -> None:
        classification = self.classify(collision1, collision2)
        if classification == "nonterrain":
            return
        self.terrain_blocks += 1
        if classification == "unclassified":
            self.unclassified_blocks += 1
            return
        self.stats[classification].add_block(depths)
        if classification in WHEELS:
            self.overall.add_block(depths)
            self.message_wheels.add(classification)
            valid = [value for value in depths if math.isfinite(value) and value >= 0.0]
            if valid:
                self.message_max_depth[classification] = max(
                    self.message_max_depth[classification], max(valid)
                )
                self.message_point_count[classification] += len(valid)
        elif classification == "chassis":
            self.message_chassis = True

    def process_message(self, timeline_writer) -> None:
        if not self.message_started:
            return
        self.message_count += 1
        support = min(len(self.message_wheels), 4)
        self.support_message_counts[str(support)] += 1
        for wheel in self.message_wheels:
            self.wheel_message_counts[wheel] += 1
        if self.message_chassis:
            self.chassis_message_count += 1

        current_time = self.message_time
        if (
            self.previous_time is not None
            and current_time is not None
            and self.previous_support is not None
        ):
            delta = current_time - self.previous_time
            if 0.0 < delta <= 1.0:
                self.total_timed_duration_s += delta
                self.support_duration_s[str(self.previous_support)] += delta
                if self.previous_support <= 2:
                    self.current_low_support_s += delta
                    self.longest_low_support_s = max(
                        self.longest_low_support_s, self.current_low_support_s
                    )
                else:
                    self.current_low_support_s = 0.0
                if self.previous_chassis:
                    self.current_chassis_contact_s += delta
                    self.longest_chassis_contact_s = max(
                        self.longest_chassis_contact_s,
                        self.current_chassis_contact_s,
                    )
                else:
                    self.current_chassis_contact_s = 0.0
            else:
                self.current_low_support_s = 0.0
                self.current_chassis_contact_s = 0.0

        should_write = False
        if current_time is not None:
            if self.last_timeline_time is None:
                should_write = True
            elif current_time - self.last_timeline_time >= self.timeline_period:
                should_write = True
        if should_write:
            row = [
                f"{current_time:.9f}", str(support), int(self.message_chassis),
            ]
            for wheel in WHEELS:
                row.extend([
                    f"{self.message_max_depth[wheel]:.9f}",
                    str(self.message_point_count[wheel]),
                ])
            timeline_writer.writerow(row)
            self.last_timeline_time = current_time

        self.previous_time = current_time
        self.previous_support = support
        self.previous_chassis = self.message_chassis

    def summary(self, wall_duration_s: float, gz_exit_code: Optional[int]) -> Dict[str, object]:
        wheel_stats = {name: self.stats[name].as_dict() for name in WHEELS}
        contact_wheels = [name for name in WHEELS if self.stats[name].points > 0]
        p95_pass = all(
            wheel_stats[name]["p95_m"] is not None
            and float(wheel_stats[name]["p95_m"]) < 0.003
            for name in contact_wheels
        )
        chassis_stats = self.stats["chassis"].as_dict()
        chassis_maximum_m = chassis_stats.get("maximum_m")
        chassis_maximum_m = (
            float(chassis_maximum_m)
            if chassis_maximum_m is not None
            else 0.0
        )

        chassis_contact_acceptable = (
            chassis_maximum_m < 0.001
            and self.longest_chassis_contact_s < 0.10
        )

        acceptance = {
            "at_least_three_wheels": len(contact_wheels) >= 3,
            "chassis_contact_acceptable": chassis_contact_acceptable,
            "all_contacting_wheel_p95_below_3mm": p95_pass,
            "longest_low_support_below_1s": self.longest_low_support_s < 1.0,
        }
        acceptance["overall_pass"] = all(acceptance.values())
        return {
            "topic": self.topic,
            "collector_wall_duration_s": wall_duration_s,
            "gz_exit_code": gz_exit_code,
            "message_count": self.message_count,
            "terrain_contact_blocks": self.terrain_blocks,
            "unclassified_terrain_blocks": self.unclassified_blocks,
            "wheel_stats": wheel_stats,
            "chassis_stats": self.stats["chassis"].as_dict(),
            "overall_wheel_stats": self.overall.as_dict(),
            "support_message_counts": self.support_message_counts,
            "support_duration_s": self.support_duration_s,
            "wheel_message_counts": self.wheel_message_counts,
            "chassis_message_count": self.chassis_message_count,
            "total_timed_duration_s": self.total_timed_duration_s,
            "longest_low_support_s": self.longest_low_support_s,
            "longest_chassis_contact_s": self.longest_chassis_contact_s,
            "contacting_wheels": contact_wheels,
            "acceptance": acceptance,
        }

    def run(self) -> int:
        import csv

        self.output_dir.mkdir(parents=True, exist_ok=True)
        timeline_path = self.output_dir / "contact_timeline.csv"
        summary_path = self.output_dir / "contact_summary.json"
        start_wall = time.monotonic()
        process = subprocess.Popen(
            ["gz", "topic", "-e", self.topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        if process.stdout is None:
            raise RuntimeError("Could not open gz topic stdout")

        in_time = False
        time_depth = 0
        in_contact = False
        contact_depth = 0
        collision1 = ""
        collision2 = ""
        depths: List[float] = []
        last_progress = start_wall

        with timeline_path.open("w", newline="", encoding="utf-8") as timeline_file:
            writer = csv.writer(timeline_file)
            header = ["sim_time_s", "support_wheels", "chassis_contact"]
            for wheel in WHEELS:
                header.extend([f"{wheel}_max_depth_m", f"{wheel}_points"])
            writer.writerow(header)

            while not STOP_REQUESTED:
                ready, _, _ = select.select([process.stdout], [], [], 0.20)
                if not ready:
                    if process.poll() is not None:
                        break
                    continue
                raw_line = process.stdout.readline()
                if not raw_line:
                    if process.poll() is not None:
                        break
                    continue
                stripped = raw_line.strip()

                if not in_contact and stripped == b"time {":
                    if self.message_started:
                        self.process_message(writer)
                    self.reset_message()
                    self.message_started = True
                    in_time = True
                    time_depth = 1
                    continue

                if in_time:
                    if stripped.startswith(b"sec:"):
                        value = self.number_value(stripped)
                        if value is not None:
                            self.message_sec = int(value)
                    elif stripped.startswith(b"nsec:"):
                        value = self.number_value(stripped)
                        if value is not None:
                            self.message_nsec = int(value)
                    time_depth += raw_line.count(b"{") - raw_line.count(b"}")
                    if time_depth <= 0:
                        in_time = False
                        if self.message_sec is not None:
                            nsec = self.message_nsec if self.message_nsec is not None else 0
                            self.message_time = self.message_sec + nsec * 1.0e-9
                    continue

                if not in_contact and stripped == b"contact {":
                    if not self.message_started:
                        self.message_started = True
                    in_contact = True
                    contact_depth = 1
                    collision1 = ""
                    collision2 = ""
                    depths = []
                    continue

                if in_contact:
                    if stripped.startswith(b"collision1:"):
                        collision1 = self.quoted_value(stripped)
                    elif stripped.startswith(b"collision2:"):
                        collision2 = self.quoted_value(stripped)
                    elif stripped.startswith(b"depth:"):
                        value = self.number_value(stripped)
                        if value is not None:
                            depths.append(value)
                    contact_depth += raw_line.count(b"{") - raw_line.count(b"}")
                    if contact_depth <= 0:
                        self.process_contact(collision1, collision2, depths)
                        in_contact = False
                    continue

                now = time.monotonic()
                if now - last_progress >= 10.0:
                    print(
                        "[contact] messages={} blocks={} duration={:.1f}s".format(
                            self.message_count, self.terrain_blocks, now - start_wall
                        ),
                        flush=True,
                    )
                    last_progress = now

            if in_contact:
                self.process_contact(collision1, collision2, depths)
            if self.message_started:
                self.process_message(writer)

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            process.wait(timeout=2.0)

        summary = self.summary(time.monotonic() - start_wall, process.returncode)
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary["acceptance"], indent=2), flush=True)
        return 0 if self.message_count > 0 else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timeline-period", type=float, default=0.02)
    args = parser.parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    collector = ContactCollector(args.topic, Path(args.output_dir), args.timeline_period)
    return collector.run()


if __name__ == "__main__":
    sys.exit(main())
