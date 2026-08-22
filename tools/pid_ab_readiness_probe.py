#!/usr/bin/env python3
"""Wait for planner inputs, publish one goal, and require a non-empty path.

This is intentionally compatible with ROS 2 Foxy, whose ``ros2 topic echo``
does not provide ``--once``.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from radiation_interfaces.msg import RiskMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class ReadinessProbe(Node):
    def __init__(self, args):
        super().__init__("pid_ab_readiness_probe")
        self.args = args
        self.terrain = None
        self.risk_grid = None
        self.continuous_risk = None
        self.odom = None
        self.path = None
        self.goal_sent = False
        self.received_at = {}

        latched_map_qos = QoSProfile(depth=1)
        latched_map_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid, args.terrain_topic, self._terrain_callback,
            latched_map_qos)
        self.create_subscription(
            OccupancyGrid, args.risk_topic, self._risk_callback,
            latched_map_qos)
        self.create_subscription(
            RiskMap, args.continuous_risk_topic,
            self._continuous_risk_callback, latched_map_qos)
        self.create_subscription(
            Odometry, args.odom_topic, self._odom_callback, 10)
        self.create_subscription(
            Path, args.path_topic, self._path_callback, 10)
        self.goal_publisher = self.create_publisher(
            PoseStamped, args.goal_topic, 10)

    def _terrain_callback(self, message):
        self.terrain = message
        self.received_at["terrain"] = time.monotonic()

    def _risk_callback(self, message):
        self.risk_grid = message
        self.received_at["risk"] = time.monotonic()

    def _continuous_risk_callback(self, message):
        self.continuous_risk = message
        self.received_at["continuous_risk"] = time.monotonic()

    def _odom_callback(self, message):
        self.odom = message
        self.received_at["odom"] = time.monotonic()

    def _path_callback(self, message):
        if self.goal_sent and message.poses:
            self.path = message

    @staticmethod
    def _valid_grid(message):
        if message is None:
            return False
        expected = int(message.info.width) * int(message.info.height)
        return (
            message.info.width > 0 and
            message.info.height > 0 and
            message.info.resolution > 0.0 and
            len(message.data) == expected
        )

    def missing_inputs(self):
        missing = []
        now = time.monotonic()
        if not self._valid_grid(self.terrain):
            missing.append(self.args.terrain_topic)
        if not self._valid_grid(self.risk_grid):
            missing.append(self.args.risk_topic)

        if self.continuous_risk is None:
            missing.append(self.args.continuous_risk_topic)
        else:
            cell_count = (
                int(self.continuous_risk.info.width) *
                int(self.continuous_risk.info.height)
            )
            if (
                cell_count <= 0 or
                len(self.continuous_risk.dose_rate_usv_h) != cell_count or
                len(self.continuous_risk.confidence) != cell_count
            ):
                missing.append(self.args.continuous_risk_topic + "(invalid)")

        if self.odom is None or not self.odom.header.frame_id:
            missing.append(self.args.odom_topic)

        input_timestamps = (
            ("terrain", self.args.terrain_topic),
            ("risk", self.args.risk_topic),
            ("continuous_risk", self.args.continuous_risk_topic),
            ("odom", self.args.odom_topic),
        )
        for key, topic in input_timestamps:
            received = self.received_at.get(key)
            if (
                received is not None and
                now - received > self.args.max_input_age_sec
            ):
                missing.append(topic + "(stale)")

        if self._valid_grid(self.terrain) and self._valid_grid(self.risk_grid):
            terrain_geometry = (
                self.terrain.info.width,
                self.terrain.info.height,
                round(float(self.terrain.info.resolution), 9),
                round(float(self.terrain.info.origin.position.x), 9),
                round(float(self.terrain.info.origin.position.y), 9),
            )
            risk_geometry = (
                self.risk_grid.info.width,
                self.risk_grid.info.height,
                round(float(self.risk_grid.info.resolution), 9),
                round(float(self.risk_grid.info.origin.position.x), 9),
                round(float(self.risk_grid.info.origin.position.y), 9),
            )
            if terrain_geometry != risk_geometry:
                missing.append("terrain/risk geometry mismatch")
        return missing

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = self.args.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.args.goal_x
        goal.pose.position.y = self.args.goal_y
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self.goal_sent = True


def parse_arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--goal-republish-sec", type=float, default=1.0)
    parser.add_argument("--max-input-age-sec", type=float, default=2.5)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--terrain-topic", default="/terrain_impedance_map")
    parser.add_argument("--risk-topic", default="/risk_map")
    parser.add_argument(
        "--continuous-risk-topic", default="/risk_map/continuous")
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--path-topic", default="/tp_asd_rrt_star_cpp_path")
    args = parser.parse_args(argv)
    if (
        not math.isfinite(args.goal_x) or
        not math.isfinite(args.goal_y) or
        not math.isfinite(args.timeout_sec) or
        args.timeout_sec <= 0.0 or
        not math.isfinite(args.goal_republish_sec) or
        args.goal_republish_sec <= 0.0 or
        not math.isfinite(args.max_input_age_sec) or
        args.max_input_age_sec <= 0.0
    ):
        parser.error("goal coordinates and positive timeouts must be finite")
    return args


def main(argv=None):
    ros_argv = sys.argv if argv is None else [sys.argv[0]] + list(argv)
    args = parse_arguments(remove_ros_args(args=ros_argv)[1:])
    rclpy.init(args=ros_argv)
    node = ReadinessProbe(args)
    deadline = time.monotonic() + args.timeout_sec
    next_goal_time = None
    last_missing = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = node.missing_inputs()
            if missing:
                current_missing = ", ".join(missing)
                if current_missing != last_missing:
                    print("WAITING_INPUTS: " + current_missing, flush=True)
                    last_missing = current_missing
                continue

            if next_goal_time is None:
                print("INPUTS_READY: terrain, risk maps, and odometry", flush=True)
                next_goal_time = 0.0

            now = time.monotonic()
            if now >= next_goal_time:
                node.publish_goal()
                print(
                    "GOAL_PUBLISHED: ({:.3f}, {:.3f})".format(
                        args.goal_x, args.goal_y),
                    flush=True)
                next_goal_time = now + args.goal_republish_sec

            if node.path is not None:
                print(
                    "PATH_READY: {} poses".format(len(node.path.poses)),
                    flush=True)
                return 0

        missing = node.missing_inputs()
        if missing:
            print(
                "TIMEOUT_WAITING_INPUTS: " + ", ".join(missing),
                file=sys.stderr,
                flush=True)
        else:
            print(
                "TIMEOUT_WAITING_PATH: planner inputs were ready but no "
                "non-empty path arrived",
                file=sys.stderr,
                flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
