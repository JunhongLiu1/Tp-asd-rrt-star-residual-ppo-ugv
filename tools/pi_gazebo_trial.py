#!/usr/bin/env python3
"""Run one reproducible fixed-path PI A/B trial against Gazebo Husky."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Bool, String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def path_shape(name):
    if name == "lateral_offset":
        return [(0.0, -0.45), (1.6, -0.45)]
    if name == "heading_offset":
        return [(0.0, 0.0), (1.6, -0.746)]
    if name == "turn_90":
        return [(0.0, 0.0), (0.9, 0.0), (0.9, 0.9)]
    if name == "s_curve":
        return [(0.12 * i, 0.35 * math.sin(math.pi * 0.12 * i / 0.9))
                for i in range(16)]
    if name == "medium_path":
        return [(0.0, 0.0), (0.7, 0.0), (1.2, 0.5), (1.8, 0.1),
                (2.3, 0.6)]
    if name == "estop_replan_recover":
        return [(0.0, 0.0), (1.8, 0.0)]
    raise ValueError("unknown scenario: " + name)


def closest_error(x, y, yaw, points):
    best = None
    for a, b in zip(points[:-1], points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 <= 1e-12:
            continue
        t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2))
        px, py = a[0] + t * dx, a[1] + t * dy
        distance = math.hypot(x - px, y - py)
        heading = abs(angle_wrap(math.atan2(dy, dx) - yaw))
        candidate = (distance, heading)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best if best is not None else (float("nan"), float("nan"))


class Trial(Node):
    def __init__(self, args):
        super().__init__("pi_gazebo_trial")
        self.args = args
        self.shape = path_shape(args.scenario)
        self.active_points = []
        self.odom = None
        self.world_xy = None
        self.world_yaw = None
        self.unsafe = False
        self.base = Twist()
        self.baseline = Twist()
        self.base_received_wall = None
        self.baseline_received_wall = None
        self.final = Twist()
        self.status = ""
        self.status_seen = []
        self.safety_status = ""
        self.safety_seen = []
        self.rl_status = ""
        self.rl_status_seen = []
        self.rows = []
        self.start_wall = None
        self.first_motion = None
        self.goal_wall = None
        self.estop_on_wall = None
        self.estop_off_wall = None
        self.replanned = False
        self.estop_active = False
        self.pub_path = self.create_publisher(PathMsg, "/tp_asd_rrt_star_cpp_path", 10)
        self.pub_estop = self.create_publisher(Bool, "/e_stop", 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 20)
        self.create_subscription(ModelStates, "/model_states", self.on_models, 10)
        self.create_subscription(Twist, "/control/base_cmd", self.on_base, 20)
        self.create_subscription(Twist, "/control/pid_baseline_cmd",
                                 self.on_baseline, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_final, 20)
        self.create_subscription(String, "/tp_asd_rrt_star_cpp_follower_status",
                                 self.on_status, 20)
        self.create_subscription(String, "/tp_asd_rrt_star_cpp_safety_status",
                                 self.on_safety_status, 20)
        self.create_subscription(String, "/control/residual_rl_status",
                                 self.on_rl_status, 20)
        self.set_state = self.create_client(SetEntityState, "/set_entity_state")

    def on_base(self, msg):
        self.base = msg
        self.base_received_wall = time.monotonic()

    def on_baseline(self, msg):
        self.baseline = msg
        self.baseline_received_wall = time.monotonic()

    def on_final(self, msg):
        self.final = msg

    def on_status(self, msg):
        self.status = msg.data
        self.status_seen.append(msg.data)
        if (self.start_wall is not None and
                msg.data.startswith("GOAL_REACHED") and
                self.goal_wall is None):
            self.goal_wall = time.monotonic()

    def on_safety_status(self, msg):
        self.safety_status = msg.data
        self.safety_seen.append(msg.data)

    def on_rl_status(self, msg):
        self.rl_status = msg.data
        self.rl_status_seen.append(msg.data)

    def on_models(self, msg):
        if "husky" not in msg.name:
            return
        pose = msg.pose[msg.name.index("husky")]
        self.world_xy = (pose.position.x, pose.position.y)
        self.world_yaw = yaw_of(pose.orientation)
        if not (-8.5 <= pose.position.x <= 9.0 and
                -13.5 <= pose.position.y <= 13.5 and
                -0.1 <= pose.position.z <= 3.0):
            self.unsafe = True
            self.publish_estop(True)

    def on_odom(self, msg):
        self.odom = msg
        if self.start_wall is None:
            return
        now = time.monotonic()
        pose = msg.pose.pose
        yaw = self.world_yaw if self.world_yaw is not None else yaw_of(
            pose.orientation)
        map_x, map_y = self.world_xy if self.world_xy is not None else (
            pose.position.x, pose.position.y)
        lateral, heading = closest_error(
            map_x, map_y, yaw, self.active_points)
        if self.first_motion is None and abs(self.final.linear.x) > 1e-4:
            self.first_motion = now
        self.rows.append({
            "wall_sec": now - self.start_wall,
            "odom_x": pose.position.x, "odom_y": pose.position.y,
            "map_x": map_x, "map_y": map_y, "yaw": yaw,
            "actual_linear": msg.twist.twist.linear.x,
            "actual_angular": msg.twist.twist.angular.z,
            "baseline_linear": self.baseline.linear.x,
            "baseline_angular": self.baseline.angular.z,
            "command_pair_age_delta_sec": (
                abs(self.base_received_wall - self.baseline_received_wall)
                if self.base_received_wall is not None and
                self.baseline_received_wall is not None else float("nan")
            ),
            "base_linear": self.base.linear.x, "base_angular": self.base.angular.z,
            "final_linear": self.final.linear.x, "final_angular": self.final.angular.z,
            "lateral_error": lateral, "heading_error": heading,
            "estop": int(self.estop_active), "replanned": int(self.replanned),
            "status": self.status,
            "rl_status": self.rl_status,
        })

    def publish_path(self):
        msg = PathMsg()
        msg.header.frame_id = self.args.path_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.active_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_path.publish(msg)

    def publish_estop(self, value):
        msg = Bool()
        msg.data = value
        self.estop_active = value
        self.pub_estop.publish(msg)

    def reset_model(self):
        if not self.set_state.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("/set_entity_state unavailable")
        model_deadline = time.monotonic() + 30.0
        while self.world_xy is None and time.monotonic() < model_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.world_xy is None:
            raise RuntimeError("Husky is absent from /model_states")
        state = EntityState()
        state.name = "husky"
        state.reference_frame = "world"
        state.pose.position.x = self.args.reset_x
        state.pose.position.y = self.args.reset_y
        state.pose.position.z = self.args.reset_z
        state.pose.orientation.w = 1.0
        request = SetEntityState.Request()
        request.state = state
        for _ in range(3):
            future = self.set_state.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if (future.done() and future.result() is not None and
                    future.result().success):
                return
            time.sleep(1.0)
        raise RuntimeError("failed to reset Husky state")


def percentile(values, q):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * q
    lo, hi = int(math.floor(position)), int(math.ceil(position))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--pid", choices=("on", "off"), required=True)
    parser.add_argument("--policy", choices=("zero", "ppo"), default="zero")
    parser.add_argument("--path-frame", default="odom")
    parser.add_argument("--reset-x", type=float, default=-1.13)
    parser.add_argument("--reset-y", type=float, default=-7.80)
    parser.add_argument("--reset-z", type=float, default=1.50)
    parser.add_argument("--run", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=75.0)
    args = parser.parse_args()
    rclpy.init()
    node = Trial(args)
    try:
        node.reset_model()
        settle_end = time.monotonic() + 5.0
        while time.monotonic() < settle_end:
            node.publish_estop(False)
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.odom is None:
            raise RuntimeError("no odometry after reset")
        if args.path_frame == "map":
            anchor_x, anchor_y = node.world_xy
            initial_yaw = node.world_yaw
        else:
            anchor = node.odom.pose.pose.position
            anchor_x, anchor_y = anchor.x, anchor.y
            initial_yaw = yaw_of(node.odom.pose.pose.orientation)
        cosine, sine = math.cos(initial_yaw), math.sin(initial_yaw)
        node.active_points = [
            (anchor_x + cosine * x - sine * y,
             anchor_y + sine * x + cosine * y)
            for x, y in node.shape
        ]
        initial_lateral, initial_heading = closest_error(
            anchor_x, anchor_y, initial_yaw, node.active_points)
        if args.scenario == "lateral_offset" and not 0.40 <= initial_lateral <= 0.50:
            raise RuntimeError("invalid initial lateral error: {:.6f}".format(initial_lateral))
        if args.scenario == "heading_offset" and not 0.38 <= initial_heading <= 0.50:
            raise RuntimeError("invalid initial heading error: {:.6f}".format(initial_heading))
        node.start_wall = time.monotonic()
        deadline = node.start_wall + args.timeout_sec
        next_path = 0.0
        while time.monotonic() < deadline and node.goal_wall is None and not node.unsafe:
            now = time.monotonic()
            elapsed = now - node.start_wall
            if elapsed < 1.0 and now >= next_path:
                node.publish_path()
                next_path = now + 0.5
            if args.scenario != "estop_replan_recover":
                node.publish_estop(False)
            if args.scenario == "estop_replan_recover":
                if elapsed < 5.0:
                    node.publish_estop(False)
                elif 5.0 <= elapsed < 7.5:
                    if node.estop_on_wall is None:
                        node.estop_on_wall = now
                    node.publish_estop(True)
                elif elapsed >= 7.5:
                    if node.estop_off_wall is None:
                        node.estop_off_wall = now
                        if node.odom is not None:
                            if args.path_frame == "map":
                                px, py = node.world_xy
                            else:
                                p = node.odom.pose.pose.position
                                px, py = p.x, p.y
                            node.active_points = [
                                (px, py), (px + 0.8, py + 0.7),
                                (px + 1.8, py + 0.7),
                            ]
                            node.replanned = True
                            node.publish_path()
                    node.publish_estop(False)
            rclpy.spin_once(node, timeout_sec=0.02)
        # End every trial through the same fail-closed path used by an
        # operator abort.  This makes terminal-zero evidence meaningful even
        # when the scenario reaches its time limit while still tracking.
        node.publish_estop(True)
        stop_deadline = time.monotonic() + 2.0
        while time.monotonic() < stop_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        args.output.mkdir(parents=True, exist_ok=True)
        sample_path = args.output / "samples.csv"
        if node.rows:
            with sample_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=node.rows[0].keys())
                writer.writeheader()
                writer.writerows(node.rows)
        lateral = [r["lateral_error"] for r in node.rows if math.isfinite(r["lateral_error"])]
        heading = [r["heading_error"] for r in node.rows if math.isfinite(r["heading_error"])]
        estop_rows = [r for r in node.rows if r["estop"]]
        aligned_rows = [
            r for r in node.rows
            if math.isfinite(r["command_pair_age_delta_sec"]) and
            r["command_pair_age_delta_sec"] <= 0.005 and
            r["rl_status"] == "policy_command"
        ]
        result = {
            "scenario": args.scenario, "pid": args.pid, "run": args.run,
            "policy": args.policy,
            "initial_lateral_error_m": initial_lateral,
            "initial_heading_error_rad": initial_heading,
            "completed": node.goal_wall is not None,
            "unsafe_boundary_stop": node.unsafe,
            "duration_sec": (node.goal_wall or time.monotonic()) - node.start_wall,
            "samples": len(node.rows),
            "lateral_rms_m": math.sqrt(sum(v * v for v in lateral) / len(lateral)) if lateral else None,
            "lateral_max_m": max(lateral) if lateral else None,
            "lateral_p95_m": percentile(lateral, 0.95),
            "heading_rms_rad": math.sqrt(sum(v * v for v in heading) / len(heading)) if heading else None,
            "heading_max_rad": max(heading) if heading else None,
            "heading_p95_rad": percentile(heading, 0.95),
            "peak_base_linear": max((abs(r["base_linear"]) for r in node.rows), default=0.0),
            "peak_base_angular": max((abs(r["base_angular"]) for r in node.rows), default=0.0),
            "peak_final_linear": max((abs(r["final_linear"]) for r in node.rows), default=0.0),
            "peak_final_angular": max((abs(r["final_angular"]) for r in node.rows), default=0.0),
            "negative_command_count": sum(r["final_linear"] < -1e-6 for r in node.rows),
            "max_abs_linear_residual": max(
                (abs(r["base_linear"] - r["baseline_linear"])
                 for r in aligned_rows), default=0.0),
            "max_abs_angular_residual": max(
                (abs(r["base_angular"] - r["baseline_angular"])
                 for r in aligned_rows), default=0.0),
            "aligned_residual_samples": len(aligned_rows),
            "residual_bound_violations": sum(
                abs(r["base_linear"] - r["baseline_linear"]) > 0.020001 or
                abs(r["base_angular"] - r["baseline_angular"]) > 0.100001
                for r in aligned_rows
            ),
            "estop_nonzero_count": sum(abs(r["final_linear"]) > 1e-6 or abs(r["final_angular"]) > 1e-6 for r in estop_rows),
            "terminal_zero": bool(node.rows) and all(
                abs(r["final_linear"]) <= 1e-6 and
                abs(r["final_angular"]) <= 1e-6
                for r in node.rows[-5:]
            ),
            "rl_fault_latched": any(
                "RL_LATCHED_DISABLED" in status
                for status in node.rl_status_seen
            ),
            "replanned": node.replanned,
            "statuses": sorted(set(s.split(":", 1)[0] for s in node.status_seen)),
            "safety_statuses": sorted(set(node.safety_seen)),
            "rl_statuses": sorted(set(
                status.split(":", 1)[0] for status in node.rl_status_seen
            )),
        }
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
        return 0 if result["completed"] else 3
    finally:
        node.publish_estop(True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
