#!/usr/bin/env python3
"""Single-run black-box dynamic-radiation replanning acceptance probe."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg
from radiation_interfaces.msg import RadiationMeasurement, RiskMap
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64


class Probe(Node):
    def __init__(self, args):
        super().__init__("dynamic_radiation_e2e_probe")
        self.args = args
        self.risk_versions = []
        self.doses = []
        self.planner_states = []
        self.follower_states = []
        self.paths = []
        self.commands = []
        self.create_subscription(RiskMap, "/risk_map/continuous", self.on_risk, 10)
        self.create_subscription(RadiationMeasurement, "/radiation/sensor_measurement", self.on_dose, 10)
        self.create_subscription(Float64, "/radiation/dose_rate_usv_h", self.on_scalar_dose, 10)
        self.create_subscription(String, "/tp_asd_rrt_star_cpp_status", self.on_planner, 10)
        self.create_subscription(String, "/tp_asd_rrt_star_cpp_follower_status", self.on_follower, 10)
        self.create_subscription(PathMsg, "/tp_asd_rrt_star_cpp_path", self.on_path, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.set_state = self.create_client(SetEntityState, "/set_entity_state")

    def stamp(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_risk(self, msg):
        self.risk_versions.append((self.stamp(), int(msg.version)))

    def on_dose(self, msg):
        self.doses.append((self.stamp(), float(msg.dose_rate_usv_h)))

    def on_scalar_dose(self, msg):
        self.doses.append((self.stamp(), float(msg.data)))

    def on_planner(self, msg):
        self.planner_states.append((self.stamp(), msg.data))

    def on_follower(self, msg):
        self.follower_states.append((self.stamp(), msg.data))

    def on_path(self, msg):
        self.paths.append((self.stamp(), len(msg.poses)))

    def on_command(self, msg):
        self.commands.append((self.stamp(), msg.linear.x, msg.angular.z))

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.args.goal_x
        msg.pose.position.y = self.args.goal_y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def move_source(self):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.args.source
        request.state.reference_frame = "world"
        request.state.pose.position.x = self.args.source_x
        request.state.pose.position.y = self.args.source_y
        request.state.pose.position.z = self.args.source_z
        request.state.pose.orientation.w = 1.0
        return self.set_state.call_async(request)


def spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="radiation_source_gamma")
    parser.add_argument("--source-x", type=float, default=9.50)
    parser.add_argument("--source-y", type=float, default=5.98)
    parser.add_argument("--source-z", type=float, default=1.20)
    parser.add_argument("--goal-x", type=float, default=7.20)
    parser.add_argument("--goal-y", type=float, default=5.98)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args)
    result = {"passed": False, "uses_ground_truth_as_planner_input": False}
    try:
        if not spin_until(node, lambda: bool(node.risk_versions), 10.0):
            raise RuntimeError("continuous risk map unavailable")
        if not node.set_state.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Gazebo set_entity_state service unavailable")
        node.publish_goal()
        if not spin_until(node, lambda: len(node.paths) >= 1, 15.0):
            raise RuntimeError("initial path unavailable")
        initial_paths = len(node.paths)
        initial_version = node.risk_versions[-1][1]
        moved_at = node.stamp()
        future = node.move_source()
        if not spin_until(node, lambda: future.done(), 5.0):
            raise RuntimeError("source move timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("source move rejected: " + (response.status_message if response else "no response"))

        def closed_loop_seen():
            newer = any(v > initial_version for _, v in node.risk_versions)
            invalid_times = [t for t, s in node.follower_states if "PATH_INVALIDATED" in s]
            invalid = bool(invalid_times)
            replanned = len(node.paths) > initial_paths
            recovered = invalid and any(
                ("RECOVERING" in s or s.startswith("TRACKING")) and
                t > invalid_times[0]
                for t, s in node.follower_states)
            return newer and invalid and replanned and recovered

        spin_until(node, closed_loop_seen, args.timeout)
        after_move = [(t, x, z) for t, x, z in node.commands if t >= moved_at]
        negative = sum(x < -1e-6 for _, x, _ in after_move)
        invalid_times = [t for t, s in node.follower_states if "PATH_INVALIDATED" in s]
        first_stop = next((t for t, x, z in after_move if abs(x) <= 1e-6 and abs(z) <= 1e-6), None)
        result.update({
            "initial_risk_version": initial_version,
            "final_risk_version": node.risk_versions[-1][1],
            "max_observed_dose_usv_h": max((d for _, d in node.doses), default=None),
            "paths_before_disturbance": initial_paths,
            "paths_after_disturbance": len(node.paths) - initial_paths,
            "path_invalidated_seen": bool(invalid_times),
            "planner_stale_snapshot_seen": any("STALE_SNAPSHOT" in s for _, s in node.planner_states),
            "negative_command_count": negative,
            "stop_latency_sec": None if not invalid_times or first_stop is None else max(0.0, first_stop - invalid_times[0]),
            "planner_states": sorted(set(s for _, s in node.planner_states)),
            "follower_states": sorted(set(s for _, s in node.follower_states)),
        })
        result["passed"] = closed_loop_seen() and negative == 0
    except Exception as exc:  # acceptance probe must always leave evidence
        result["error"] = str(exc)
    finally:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "result.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
