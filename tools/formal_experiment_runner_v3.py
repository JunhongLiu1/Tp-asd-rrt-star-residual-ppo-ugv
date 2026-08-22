#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


class ManagedProcess:
    def __init__(self, name: str, command: Sequence[str], log_path: Path, env: Dict[str, str]):
        self.name = name
        self.command = list(command)
        self.log_path = log_path
        self.log_file = log_path.open("w", encoding="utf-8")
        self.closed = False
        self.process = subprocess.Popen(
            self.command,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )

    def poll(self) -> Optional[int]:
        return self.process.poll()

    def stop(self, first_signal: int = signal.SIGINT, timeout: float = 8.0) -> None:
        if self.closed:
            return
        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), first_signal)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
                    try:
                        self.process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        pass
        if not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()
        self.closed = True


class ExperimentNode(Node):
    def __init__(self, path_topic: str, goal_x: float, goal_y: float) -> None:
        super().__init__("formal_experiment_runner_v2_monitor")
        self.path_topic = path_topic
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.terrain_received = False
        self.radiation_received = False
        self.terrain_map: Optional[OccupancyGrid] = None
        self.radiation_map: Optional[OccupancyGrid] = None
        self.odom: Optional[Odometry] = None
        self.dose: Optional[float] = None
        self.path: Optional[RosPath] = None
        self.path_received_wall: Optional[float] = None
        self.goal_published_wall: Optional[float] = None
        self.execution_started_wall: Optional[float] = None
        self.execution_start_pose: Optional[Tuple[float, float, float]] = None
        self.execution_start_dose: Optional[float] = None
        self.trajectory: List[Tuple[float, float, float, float]] = []
        self.last_trajectory_wall = 0.0

        # The monitor starts before the environment publishers.
        # BEST_EFFORT + VOLATILE is compatible with both reliable and
        # best-effort runtime publishers and avoids intermittent DDS
        # discovery/QoS mismatches after Gazebo restarts.
        map_monitor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            OccupancyGrid,
            "/terrain_impedance_map",
            self.terrain_callback,
            map_monitor_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/radiation_map",
            self.radiation_callback,
            map_monitor_qos,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Float64,
            "/radiation/accumulated_dose_usv",
            self.dose_callback,
            sensor_qos,
        )
        self.create_subscription(
            RosPath,
            path_topic,
            self.path_callback,
            path_qos,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            command_qos,
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            command_qos,
        )
        self.stop_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            command_qos,
        )

    def terrain_callback(self, msg: OccupancyGrid) -> None:
        self.terrain_map = msg
        self.terrain_received = True

    def radiation_callback(self, msg: OccupancyGrid) -> None:
        self.radiation_map = msg
        self.radiation_received = True

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        if self.execution_started_wall is not None:
            now = time.monotonic()
            if now - self.last_trajectory_wall >= 0.05:
                position = msg.pose.pose.position
                self.trajectory.append((now, position.x, position.y, position.z))
                self.last_trajectory_wall = now

    def dose_callback(self, msg: Float64) -> None:
        self.dose = float(msg.data)

    def path_callback(self, msg: RosPath) -> None:
        if self.goal_published_wall is None or len(msg.poses) < 3:
            return
        if self.path is None:
            self.path = msg
            self.path_received_wall = time.monotonic()

    def cmd_callback(self, msg: Twist) -> None:
        moving = abs(msg.linear.x) > 1.0e-4 or abs(msg.angular.z) > 1.0e-4
        if moving and self.execution_started_wall is None:
            self.execution_started_wall = time.monotonic()
            self.execution_start_dose = self.dose
            if self.odom is not None:
                position = self.odom.pose.pose.position
                self.execution_start_pose = (position.x, position.y, position.z)
                self.trajectory.append(
                    (self.execution_started_wall, position.x, position.y, position.z)
                )

    def publish_goal(self) -> None:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_published_wall = time.monotonic()
        self.goal_publisher.publish(goal)

    def stop_robot(self) -> None:
        message = Twist()
        for _ in range(5):
            self.stop_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)


class FormalExperimentRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ws = Path(args.workspace).expanduser().resolve()
        self.install_share = self.ws / "install/radiation_mapping/share/radiation_mapping"
        self.launch_dir = self.install_share / "launch"
        self.tools_dir = self.ws / "tools"
        self.collector_script = self.tools_dir / "formal_contact_collector_v2.py"
        self.env = os.environ.copy()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = "run_{}_{}".format(self.safe_name(args.run_id), timestamp)
        self.run_dir = (
            self.ws
            / "formal_experiments_v3"
            / self.safe_name(args.scenario)
            / self.safe_name(args.planner)
            / run_name
        )
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.processes: List[ManagedProcess] = []
        self.node: Optional[ExperimentNode] = None
        self.contact_process: Optional[ManagedProcess] = None
        self.follower_process: Optional[ManagedProcess] = None
        self.planner_process: Optional[ManagedProcess] = None
        self.start_position: Optional[Tuple[float, float, float]] = None
        self.contact_topic: Optional[str] = None
        self.failure_reason = ""
        self.rclpy_started = False
        self.follower_exit_wall: Optional[float] = None

    @staticmethod
    def safe_name(value: str) -> str:
        return "".join(char if char.isalnum() or char in "-_" else "_" for char in value)

    def planner_config(self) -> Dict[str, str]:
        if self.args.planner == "asd":
            return {
                "label": "ASD-RRT*",
                "path_topic": "/asd_rrt_star_path",
                "node_name": "/asd_rrt_star_planner",
                "include_time_penalty": "false",
            }
        if self.args.planner == "tp":
            return {
                "label": "TP-ASD-RRT*",
                "path_topic": "/tp_asd_rrt_star_path",
                "node_name": "/tp_asd_rrt_star_planner",
                "include_time_penalty": "true",
            }
        if self.args.planner == "aco":
            return {
                "label": "ACO",
                "path_topic": "/aco_path",
                "node_name": "/aco_planner",
                "include_time_penalty": "false",
            }
        raise ValueError("Unsupported planner")

    def start_process(self, name: str, command: Sequence[str], log_name: str) -> ManagedProcess:
        process = ManagedProcess(name, command, self.run_dir / log_name, self.env)
        self.processes.append(process)
        print("[START] {} pid={}".format(name, process.process.pid), flush=True)
        return process

    @staticmethod
    def run_command(command: Sequence[str], timeout: float = 20.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            list(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )

    def clean_old_stack(self) -> None:
        print("========== CLEAN OLD STACK ==========", flush=True)
        patterns = (
            "gazebo_radiation_husky_formal.launch.py",
            "terrain_services.launch.py",
            "final_asd_rrt_star_planner",
            "final_aco_formal.launch.py",
            "aco_planner",
            "formal_path_waypoint_follower",
            "formal_contact_collector_v2.py",
        )
        for pattern in patterns:
            subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        subprocess.run(["pkill", "-TERM", "-x", "gzserver"], check=False)
        subprocess.run(["pkill", "-TERM", "-x", "gzclient"], check=False)
        time.sleep(4.0)
        subprocess.run(["pkill", "-KILL", "-x", "gzserver"], check=False)
        subprocess.run(["pkill", "-KILL", "-x", "gzclient"], check=False)
        # Do not restart the ROS 2 CLI daemon here. It is not required by
        # rclpy and restarting it immediately before launching the stack can
        # introduce unnecessary discovery delays.
        time.sleep(3.0)

    def validate_files(self) -> None:
        required = [
            self.launch_dir / "gazebo_radiation_husky_formal.launch.py",
            self.launch_dir / "terrain_services.launch.py",
            self.launch_dir / "module_final_asd_rrt_star.launch.py",
            self.collector_script,
        ]
        if self.args.planner == "aco":
            required.append(self.launch_dir / "final_aco_formal.launch.py")
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise RuntimeError("Missing required files: " + ", ".join(missing))

    def spin_until(self, predicate, timeout: float, label: str) -> bool:
        if self.node is None:
            return False
        start = time.monotonic()
        last_print = start
        while rclpy.ok() and time.monotonic() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.10)
            if predicate():
                print("[PASS] {}".format(label), flush=True)
                return True
            now = time.monotonic()
            if now - last_print >= 5.0:
                print("[WAIT] {} elapsed={:.1f}s".format(label, now - start), flush=True)
                last_print = now
        print("[FAIL] {}".format(label), flush=True)
        return False

    def launch_stack(self) -> None:
        gazebo_launch = self.launch_dir / "gazebo_radiation_husky_formal.launch.py"
        terrain_launch = self.launch_dir / "terrain_services.launch.py"
        self.start_process(
            "gazebo",
            [
                "ros2", "launch", str(gazebo_launch),
                "start_x:={}".format(self.args.start_x),
                "start_y:={}".format(self.args.start_y),
                "start_z:={}".format(self.args.start_z),
                "start_yaw:={}".format(self.args.start_yaw),
                "world_path:={}".format(self.args.world_path),
                "use_sim_time:=true",
            ],
            "gazebo.log",
        )
        time.sleep(5.0)
        self.start_process(
            "terrain_services",
            [
                "ros2", "launch", str(terrain_launch),
                "terrain:={}".format(self.args.terrain),
                "data_directory:={}".format(self.args.data_directory),
                "use_sim_time:=true",
            ],
            "terrain.log",
        )

    def wait_for_environment(self) -> None:
        assert self.node is not None

        start = time.monotonic()
        last_print = 0.0

        while rclpy.ok() and time.monotonic() - start < self.args.stack_timeout:
            rclpy.spin_once(self.node, timeout_sec=0.10)

            states = {
                "terrain_map": self.node.terrain_received,
                "radiation_map": self.node.radiation_received,
                "ground_truth_odom": self.node.odom is not None,
                "dose_stream": self.node.dose is not None,
            }

            if all(states.values()):
                print(
                    "[PASS] terrain map + radiation map + "
                    "ground-truth odom + dose stream",
                    flush=True,
                )
                ready = True
                break

            elapsed = time.monotonic() - start

            if elapsed - last_print >= 5.0:
                process_states = {}

                for process in self.processes:
                    process_states[process.name] = process.poll()

                print(
                    "[WAIT] environment elapsed={:.1f}s "
                    "terrain={} radiation={} odom={} dose={} "
                    "process_exit_codes={}".format(
                        elapsed,
                        states["terrain_map"],
                        states["radiation_map"],
                        states["ground_truth_odom"],
                        states["dose_stream"],
                        process_states,
                    ),
                    flush=True,
                )
                last_print = elapsed
        else:
            ready = False

        if not ready:
            missing = [
                name
                for name, received in states.items()
                if not received
            ]

            process_states = {
                process.name: process.poll()
                for process in self.processes
            }

            raise RuntimeError(
                "Environment inputs did not become ready; "
                "missing={}; process_exit_codes={}".format(
                    missing,
                    process_states,
                )
            )
        time.sleep(self.args.settle_time)
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.10)
        assert self.node.odom is not None
        position = self.node.odom.pose.pose.position
        self.start_position = (position.x, position.y, position.z)
        error = math.hypot(position.x - self.args.start_x, position.y - self.args.start_y)
        print("Settled start=({:.3f}, {:.3f}, {:.3f}), XY error={:.3f} m".format(
            position.x, position.y, position.z, error
        ))
        if error > self.args.start_tolerance:
            raise RuntimeError("Robot did not settle near the formal start position")

    def launch_planner(self, config: Dict[str, str]) -> None:
        if self.args.planner in ("asd", "tp"):
            module_launch = self.launch_dir / "module_final_asd_rrt_star.launch.py"
            command = [
                "ros2", "launch", str(module_launch),
                "node_name:={}".format(config["node_name"].lstrip("/")),
                "output_path_topic:={}".format(config["path_topic"]),
                "terrain_topic:=/terrain_impedance_map",
                "radiation_topic:=/radiation_map",
                "cost_profile:=balanced",
                "cost_model_config:={}".format(self.args.cost_model_config),
                "include_time_penalty:={}".format(config["include_time_penalty"]),
                "random_seed:={}".format(self.args.seed),
                "terrain_input_max:=100.0",
                "radiation_input_mode:=normalized_occupancy",
                "radiation_input_max:=100.0",
                "odom_topic:=/ground_truth/odom",
                "odom_to_map_x:=0.0",
                "odom_to_map_y:=0.0",
                "odom_to_map_yaw:=0.0",
                "use_sim_time:=true",
            ]
        else:
            command = [
                "ros2", "launch", str(self.launch_dir / "final_aco_formal.launch.py"),
                "aco_seed:={}".format(self.args.seed),
                "cost_model_config:={}".format(self.args.cost_model_config),
                "odom_topic:=/ground_truth/odom",
                "use_sim_time:=true",
            ]
        self.planner_process = self.start_process("planner", command, "planner.log")
        assert self.node is not None
        if not self.spin_until(
            lambda: self.node.count_publishers(config["path_topic"]) >= 1
            and self.node.goal_publisher.get_subscription_count() >= 1,
            self.args.planner_timeout,
            "planner path publisher + /goal_pose subscriber",
        ):
            raise RuntimeError("Planner endpoints did not become ready")
        # ASD/TP receive random_seed directly through their launch arguments.
        # ACO receives aco_seed through its launch argument.
        # Do not attempt a second runtime parameter update.
        print(
            "[PASS] planner launched with requested seed {}".format(
                self.args.seed
            ),
            flush=True,
        )
        time.sleep(5.0)

    def apply_seed(self, node_name: str) -> None:
        result = self.run_command(["ros2", "param", "list", node_name], timeout=10.0)
        parameters = result.stdout.splitlines()
        parameter_name = None
        for candidate in ("random_seed", "seed", "aco_seed"):
            if any(line.strip() == candidate for line in parameters):
                parameter_name = candidate
                break
        if parameter_name is not None:
            set_result = self.run_command(
                ["ros2", "param", "set", node_name, parameter_name, str(self.args.seed)],
                timeout=10.0,
            )
            seed_output = (
                (set_result.stdout or "") + "\n" +
                (set_result.stderr or "")
            ).strip()

            if (
                set_result.returncode != 0
                or "successful" not in seed_output.lower()
            ):
                raise RuntimeError(
                    "Could not set planner seed: " + seed_output
                )
            print("[PASS] planner seed {}={}".format(parameter_name, self.args.seed))
        elif self.args.seed != 31:
            raise RuntimeError(
                "Planner exposes no seed parameter. Only the verified default seed 31 can be recorded honestly."
            )
        else:
            print("[INFO] planner exposes no seed parameter; using verified default/hard-coded seed 31")

    def launch_follower(self, config: Dict[str, str]) -> None:
        assert self.node is not None
        baseline_subscribers = self.node.count_subscribers(config["path_topic"])
        result_csv = self.run_dir / "execution_result.csv"
        self.follower_process = self.start_process(
            "follower",
            [
                "ros2", "run", "radiation_mapping", "formal_path_waypoint_follower",
                "--ros-args",
                "-p", "path_topic:={}".format(config["path_topic"]),
                "-p", "planner_name:={}".format(config["label"]),
                "-p", "dose_topic:=/radiation/accumulated_dose_usv",
                "-p", "terrain_topic:=/terrain_impedance_map",
                "-p", "radiation_topic:=/radiation_map",
                "-p", "result_csv:={}".format(result_csv),
                "-p", "shutdown_on_finish:=true",
                "-p", "use_sim_time:=true",
            ],
            "follower.log",
        )
        if not self.spin_until(
            lambda: self.node.count_subscribers(config["path_topic"]) > baseline_subscribers,
            30.0,
            "follower path subscription",
        ):
            raise RuntimeError("Follower did not subscribe to the selected path")
        if self.follower_process.poll() is not None:
            raise RuntimeError("Follower exited during startup")

    def discover_contact_topic(self) -> str:
        result = self.run_command(["gz", "topic", "-l"], timeout=15.0)
        candidates = [line.strip() for line in result.stdout.splitlines() if line.strip().endswith("/physics/contacts")]
        if not candidates:
            raise RuntimeError("Gazebo contact topic was not found")
        self.contact_topic = candidates[0]
        print("[PASS] contact topic {}".format(self.contact_topic))
        return self.contact_topic

    def launch_contact_collector(self) -> None:
        if self.contact_process is not None:
            return
        if self.contact_topic is None:
            self.discover_contact_topic()
        self.contact_process = self.start_process(
            "contact_collector",
            [
                sys.executable,
                str(self.collector_script),
                "--topic", str(self.contact_topic),
                "--output-dir", str(self.run_dir),
                "--timeline-period", str(self.args.contact_timeline_period),
            ],
            "contact_collector.log",
        )
        time.sleep(0.5)
        if self.contact_process.poll() is not None:
            raise RuntimeError("Contact collector exited during startup")

    def publish_goal_and_wait_for_path(self, config: Dict[str, str]) -> None:
        assert self.node is not None
        print("========== PUBLISH GOAL ==========", flush=True)
        print("goal=({:.3f}, {:.3f})".format(self.args.goal_x, self.args.goal_y), flush=True)
        self.node.publish_goal()
        if not self.spin_until(
            lambda: self.node.path is not None,
            self.args.planning_timeout,
            "fresh path on {}".format(config["path_topic"]),
        ):
            raise RuntimeError("Planner did not publish a fresh valid path")
        self.write_planned_path()
        self.launch_contact_collector()
        print("[PASS] fresh path received; contact recording started before motion", flush=True)

    def write_planned_path(self) -> None:
        assert self.node is not None and self.node.path is not None
        path_file = self.run_dir / "planned_path.csv"
        with path_file.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["index", "x", "y", "z"])
            for index, pose_stamped in enumerate(self.node.path.poses):
                position = pose_stamped.pose.position
                writer.writerow([index, position.x, position.y, position.z])

    def wait_for_execution(self) -> int:
        assert self.node is not None and self.follower_process is not None
        print("========== EXECUTION ==========", flush=True)
        start_wait = time.monotonic()
        execution_deadline: Optional[float] = None
        last_print = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            follower_code = self.follower_process.poll()

            # The ROS2 follower may finish, save its CSV, and call
            # rclpy.shutdown(), while the `ros2 run` wrapper remains alive.
            # Treat a valid result row in this run's CSV as authoritative
            # completion instead of waiting for the 360 s timeout.
            result_path = self.run_dir / "execution_result.csv"

            if result_path.is_file():
                try:
                    with result_path.open(
                        "r",
                        newline="",
                        encoding="utf-8",
                    ) as result_file:
                        result_rows = list(csv.DictReader(result_file))
                except (OSError, csv.Error):
                    result_rows = []

                if result_rows:
                    self.follower_exit_wall = time.monotonic()

                    if follower_code is None:
                        self.follower_process.stop(
                            signal.SIGTERM,
                            timeout=3.0,
                        )

                    print(
                        "[PASS] follower result CSV detected; "
                        "execution complete",
                        flush=True,
                    )
                    return 0

            if self.node.execution_started_wall is not None and execution_deadline is None:
                execution_deadline = self.node.execution_started_wall + self.args.execution_timeout
                print("[PASS] execution motion detected", flush=True)
            if follower_code is not None:
                self.follower_exit_wall = time.monotonic()
                return follower_code
            now = time.monotonic()
            if self.node.execution_started_wall is None:
                if now - start_wait > self.args.execution_start_timeout:
                    raise RuntimeError("Follower did not begin moving after path publication")
            elif execution_deadline is not None and now >= execution_deadline:
                self.node.stop_robot()
                self.follower_process.stop(signal.SIGTERM, timeout=3.0)
                self.follower_exit_wall = time.monotonic()
                return 124
            if now - last_print >= 10.0:
                if self.node.execution_started_wall is None:
                    print("[WAIT] follower motion start", flush=True)
                else:
                    print(
                        "[RUN] execution elapsed={:.1f}s trajectory_samples={}".format(
                            now - self.node.execution_started_wall, len(self.node.trajectory)
                        ),
                        flush=True,
                    )
                last_print = now

    def stop_contact_collector(self) -> None:
        if self.contact_process is not None:
            self.contact_process.stop(signal.SIGINT, timeout=12.0)

    @staticmethod
    def path_points(path: RosPath) -> List[Tuple[float, float]]:
        return [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]

    @staticmethod
    def path_length(points: List[Tuple[float, float]]) -> float:
        return sum(
            math.hypot(x2 - x1, y2 - y1)
            for (x1, y1), (x2, y2) in zip(points[:-1], points[1:])
        )

    @staticmethod
    def occupancy_value(
        map_msg: Optional[OccupancyGrid],
        x: float,
        y: float,
    ) -> Optional[float]:
        if map_msg is None or map_msg.info.resolution <= 0.0:
            return None

        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if (
            map_x < 0
            or map_x >= map_msg.info.width
            or map_y < 0
            or map_y >= map_msg.info.height
        ):
            return None

        index = map_y * map_msg.info.width + map_x
        if index < 0 or index >= len(map_msg.data):
            return None

        value = float(map_msg.data[index])
        return 100.0 if value < 0.0 else value

    @classmethod
    def integrated_map_cost(
        cls,
        points: List[Tuple[float, float]],
        map_msg: Optional[OccupancyGrid],
        divisor: float,
    ) -> Tuple[Optional[float], int, int]:
        if map_msg is None or len(points) < 2 or divisor <= 0.0:
            return None, 0, 0

        total = 0.0
        valid = 0
        out_of_bounds = 0

        for (x1, y1), (x2, y2) in zip(points[:-1], points[1:]):
            segment_distance = math.hypot(x2 - x1, y2 - y1)
            if segment_distance < 1.0e-9:
                continue

            midpoint_x = 0.5 * (x1 + x2)
            midpoint_y = 0.5 * (y1 + y2)
            value = cls.occupancy_value(map_msg, midpoint_x, midpoint_y)

            if value is None:
                out_of_bounds += 1
                continue

            valid += 1
            total += (value / divisor) * segment_distance

        return total, valid, out_of_bounds

    @staticmethod
    def tracking_errors(
        trajectory: List[Tuple[float, float, float, float]],
        path_points: List[Tuple[float, float]],
    ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        if not trajectory or not path_points:
            return None, None, None
        errors: List[float] = []
        for _, x, y, _ in trajectory:
            errors.append(min(math.hypot(x - px, y - py) for px, py in path_points))
        mean = sum(errors) / len(errors)
        rms = math.sqrt(sum(value * value for value in errors) / len(errors))
        return mean, rms, max(errors)

    def write_trajectory(self) -> None:
        assert self.node is not None
        path = self.run_dir / "executed_trajectory.csv"
        start = self.node.execution_started_wall
        with path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_wall_s", "x", "y", "z"])
            for wall_time, x, y, z in self.node.trajectory:
                elapsed = wall_time - start if start is not None else 0.0
                writer.writerow([elapsed, x, y, z])

    def parse_follower_result(self) -> Dict[str, str]:
        path = self.run_dir / "execution_result.csv"
        if not path.is_file():
            return {}
        with path.open("r", newline="", encoding="utf-8") as input_file:
            rows = list(csv.DictReader(input_file))
        return rows[-1] if rows else {}

    def parse_contact_summary(self) -> Dict[str, object]:
        path = self.run_dir / "contact_summary.json"
        if not path.is_file():
            return {}
        return json.loads(path.read_text(encoding="utf-8"))

    def finalise(self, config: Dict[str, str], follower_code: int) -> Tuple[bool, Dict[str, object]]:
        assert self.node is not None and self.node.path is not None
        self.node.stop_robot()
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.write_trajectory()
        follower = self.parse_follower_result()
        contact = self.parse_contact_summary()
        points = self.path_points(self.node.path)
        planned_length = self.path_length(points)
        planned_terrain_cost, planned_terrain_valid, planned_terrain_oob = (
            self.integrated_map_cost(points, self.node.terrain_map, 10.0)
        )
        planned_radiation_cost, planned_radiation_valid, planned_radiation_oob = (
            self.integrated_map_cost(points, self.node.radiation_map, 100.0)
        )
        mean_error, rms_error, max_error = self.tracking_errors(self.node.trajectory, points)
        final_position = None
        if self.node.odom is not None:
            position = self.node.odom.pose.pose.position
            final_position = (position.x, position.y, position.z)
        goal_error = None
        if final_position is not None:
            goal_error = math.hypot(
                final_position[0] - self.args.goal_x,
                final_position[1] - self.args.goal_y,
            )
        planning_time = None
        if self.node.goal_published_wall is not None and self.node.path_received_wall is not None:
            planning_time = self.node.path_received_wall - self.node.goal_published_wall
        execution_wall = None
        if self.node.execution_started_wall is not None and self.follower_exit_wall is not None:
            execution_wall = self.follower_exit_wall - self.node.execution_started_wall

        follower_terrain_ready = self.bool_or_false(
            follower.get("terrain_map_received")
        )
        follower_dose_ready = self.bool_or_false(
            follower.get("dose_received")
        )
        follower_radiation_ready = self.bool_or_false(
            follower.get("radiation_map_received")
        )
        terrain_valid_samples = self.int_or_zero(
            follower.get("terrain_valid_sample_count")
        )
        terrain_out_of_bounds = self.int_or_zero(
            follower.get("terrain_out_of_bounds_count")
        )
        radiation_valid_samples = self.int_or_zero(
            follower.get("radiation_valid_sample_count")
        )
        radiation_out_of_bounds = self.int_or_zero(
            follower.get("radiation_out_of_bounds_count")
        )
        contact_pass = bool(contact.get("acceptance", {}).get("overall_pass", False))
        contact_observed = not contact_pass

        # Contact is recorded as an execution-quality metric.
        # It does not automatically invalidate a completed execution.
        success_conditions = {
            "follower_exit_zero": follower_code == 0,
            "follower_csv_written": bool(follower),
            "follower_received_terrain_map": follower_terrain_ready,
            "follower_received_dose_stream": follower_dose_ready,
            "follower_received_radiation_map": follower_radiation_ready,
            "terrain_metric_has_valid_samples": terrain_valid_samples > 0,
            "radiation_metric_has_valid_samples": radiation_valid_samples > 0,
            "path_has_at_least_3_poses": len(points) >= 3,
            "final_goal_error_within_tolerance": (
                goal_error is not None
                and goal_error <= self.args.goal_tolerance
            ),
        }
        success = all(success_conditions.values())
        if not success and not self.failure_reason:
            self.failure_reason = "; ".join(
                name for name, passed in success_conditions.items() if not passed
            )

        summary: Dict[str, object] = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "scenario": self.args.scenario,
            "planner_key": self.args.planner,
            "planner_name": config["label"],
            "run_id": self.args.run_id,
            "seed": self.args.seed,
            "terrain": self.args.terrain,
            "world_path": self.args.world_path,
            "data_directory": self.args.data_directory,
            "radiation_profile": self.args.radiation_profile,
            "start_requested_x": self.args.start_x,
            "start_requested_y": self.args.start_y,
            "start_requested_z": self.args.start_z,
            "start_settled_x": self.start_position[0] if self.start_position else None,
            "start_settled_y": self.start_position[1] if self.start_position else None,
            "start_settled_z": self.start_position[2] if self.start_position else None,
            "goal_x": self.args.goal_x,
            "goal_y": self.args.goal_y,
            "path_topic": config["path_topic"],
            "path_pose_count": len(points),
            "planned_path_length_m": planned_length,
            "planned_terrain_cost": planned_terrain_cost,
            "planned_terrain_valid_sample_count": planned_terrain_valid,
            "planned_terrain_out_of_bounds_count": planned_terrain_oob,
            "planned_radiation_map_cost": planned_radiation_cost,
            "planned_radiation_valid_sample_count": planned_radiation_valid,
            "planned_radiation_out_of_bounds_count": planned_radiation_oob,
            "planning_time_wall_s": planning_time,
            "execution_time_wall_s": execution_wall,
            "execution_time_follower_s": self.float_or_none(follower.get("execution_time_s")),
            "executed_path_length_m": self.float_or_none(follower.get("executed_path_length_m")),
            "dose_during_execution_usv": self.float_or_none(follower.get("dose_during_path_following")),
            "executed_terrain_cost": self.float_or_none(follower.get("executed_terrain_cost")),
            "executed_radiation_map_cost": self.float_or_none(
                follower.get("executed_radiation_map_cost")
            ),
            "executed_final_coupled_score": self.float_or_none(follower.get("executed_final_coupled_score")),
            "dose_monitor_total_dose_usv": self.float_or_none(follower.get("dose_monitor_total_dose")),
            "follower_terrain_map_received": follower_terrain_ready,
            "follower_radiation_map_received": follower_radiation_ready,
            "follower_dose_received": follower_dose_ready,
            "terrain_valid_sample_count": terrain_valid_samples,
            "terrain_out_of_bounds_count": terrain_out_of_bounds,
            "radiation_valid_sample_count": radiation_valid_samples,
            "radiation_out_of_bounds_count": radiation_out_of_bounds,
            "terrain_cost_definition": "sum((terrain_value/10)*segment_length_m)",
            "radiation_map_cost_definition": "sum((radiation_value/100)*segment_length_m)",
            "dose_definition": "finish_accumulated_dose_minus_start_accumulated_dose_usv",
            "start_elevation_m": getattr(self.args, "start_elevation_m", None),
            "start_impedance": getattr(self.args, "start_impedance", None),
            "goal_elevation_m": getattr(self.args, "goal_elevation_m", None),
            "goal_impedance": getattr(self.args, "goal_impedance", None),
            "final_x": final_position[0] if final_position else None,
            "final_y": final_position[1] if final_position else None,
            "final_z": final_position[2] if final_position else None,
            "final_goal_error_m": goal_error,
            "tracking_mean_error_m": mean_error,
            "tracking_rms_error_m": rms_error,
            "tracking_max_error_m": max_error,
            "trajectory_samples": len(self.node.trajectory),
            "follower_exit_code": follower_code,
            "contact_message_count": contact.get("message_count"),
            "contact_duration_s": contact.get("total_timed_duration_s"),
            "contact_overall_p95_m": contact.get("overall_wheel_stats", {}).get("p95_m") if contact else None,
            "contact_overall_p99_m": contact.get("overall_wheel_stats", {}).get("p99_m") if contact else None,
            "contact_overall_max_m": contact.get("overall_wheel_stats", {}).get("maximum_m") if contact else None,
            "contact_chassis_blocks": contact.get("chassis_stats", {}).get("contact_blocks") if contact else None,
            "contact_longest_low_support_s": contact.get("longest_low_support_s") if contact else None,
            "contact_pass": contact_pass,
            "contact_observed": contact_observed,
            "success": success,
            "failure_reason": self.failure_reason,
            "run_directory": str(self.run_dir),
            "success_conditions": success_conditions,
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.append_master_csv(summary)
        return success, summary

    @staticmethod
    def float_or_none(value: Optional[str]) -> Optional[float]:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except ValueError:
            return None

    @staticmethod
    def int_or_zero(value: Optional[str]) -> int:
        try:
            return int(value) if value not in (None, "") else 0
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def bool_or_false(value: Optional[str]) -> bool:
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def append_master_csv(self, summary: Dict[str, object]) -> None:
        master = self.ws / "formal_experiments_v3/formal_execution_summary_v3.csv"
        master.parent.mkdir(parents=True, exist_ok=True)
        flattened = {
            key: value
            for key, value in summary.items()
            if key != "success_conditions"
        }
        new_fields = list(flattened.keys())

        if not master.is_file() or master.stat().st_size == 0:
            with master.open("w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(output, fieldnames=new_fields)
                writer.writeheader()
                writer.writerow(flattened)
            return

        with master.open("r", newline="", encoding="utf-8") as input_file:
            reader = csv.DictReader(input_file)
            old_fields = list(reader.fieldnames or [])
            old_rows = list(reader)

        merged_fields = old_fields + [
            field for field in new_fields if field not in old_fields
        ]

        if merged_fields != old_fields:
            temporary = master.with_suffix(master.suffix + ".tmp")
            with temporary.open("w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(output, fieldnames=merged_fields)
                writer.writeheader()
                for row in old_rows:
                    writer.writerow(row)
            temporary.replace(master)

        with master.open("a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=merged_fields)
            writer.writerow(flattened)

    def write_manifest(self, config: Dict[str, str]) -> None:
        manifest = {
            "runner_version": "3.1-radiation-map-metrics",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "arguments": vars(self.args),
            "planner_config": config,
            "run_directory": str(self.run_dir),
        }
        (self.run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    def cleanup(self) -> None:
        if self.node is not None:
            try:
                self.node.stop_robot()
            except Exception:
                pass
        self.stop_contact_collector()
        for process in reversed(self.processes):
            if process is self.contact_process:
                continue
            try:
                process.stop(signal.SIGINT, timeout=8.0)
            except Exception:
                pass
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if self.rclpy_started and rclpy.ok():
            rclpy.shutdown()
        self.rclpy_started = False

    def run(self) -> int:
        config = self.planner_config()
        self.write_manifest(config)
        print("==================================================")
        print("FORMAL EXPERIMENT RUNNER V3")
        print("planner  = {}".format(config["label"]))
        print("run_id   = {}".format(self.args.run_id))
        print("seed     = {}".format(self.args.seed))
        print("start    = ({:.3f}, {:.3f}, {:.3f})".format(self.args.start_x, self.args.start_y, self.args.start_z))
        print("goal     = ({:.3f}, {:.3f})".format(self.args.goal_x, self.args.goal_y))
        print("terrain  = {}".format(self.args.terrain))
        print("world    = {}".format(self.args.world_path))
        print("radiation= {}".format(self.args.radiation_profile))
        print("run_dir  = {}".format(self.run_dir))
        print("==================================================")
        try:
            self.validate_files()
            self.clean_old_stack()
            rclpy.init()
            self.rclpy_started = True
            self.node = ExperimentNode(config["path_topic"], self.args.goal_x, self.args.goal_y)
            self.launch_stack()
            self.wait_for_environment()
            self.discover_contact_topic()
            self.launch_planner(config)
            self.launch_follower(config)
            self.publish_goal_and_wait_for_path(config)
            follower_code = self.wait_for_execution()
            self.stop_contact_collector()
            success, summary = self.finalise(config, follower_code)
            print("==================================================")
            print("FORMAL EXPERIMENT RESULT: {}".format("PASS" if success else "FAIL"))
            print("planning_time_wall_s = {}".format(summary["planning_time_wall_s"]))
            print("execution_time_s     = {}".format(summary["execution_time_follower_s"]))
            print("executed_distance_m  = {}".format(summary["executed_path_length_m"]))
            print("terrain_cost         = {}".format(summary["executed_terrain_cost"]))
            print("radiation_map_cost   = {}".format(summary["executed_radiation_map_cost"]))
            print("dose_execution_usv   = {}".format(summary["dose_during_execution_usv"]))
            print("final_goal_error_m   = {}".format(summary["final_goal_error_m"]))
            print("tracking_rms_error_m = {}".format(summary["tracking_rms_error_m"]))
            print("contact_pass         = {}".format(summary["contact_pass"]))
            print("summary              = {}".format(self.run_dir / "summary.json"))
            print("==================================================")
            return 0 if success else 1
        except Exception as error:
            self.failure_reason = str(error)
            print("[ERROR] {}".format(error), file=sys.stderr, flush=True)
            failure = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "scenario": self.args.scenario,
                "planner_key": self.args.planner,
                "run_id": self.args.run_id,
                "seed": self.args.seed,
                "success": False,
                "failure_reason": self.failure_reason,
                "run_directory": str(self.run_dir),
            }
            (self.run_dir / "failure.json").write_text(json.dumps(failure, indent=2), encoding="utf-8")
            return 2
        finally:
            self.cleanup()


def _resolve_workspace_path(workspace: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = workspace / path
    return path.resolve()


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError("JSON file not found: {}".format(path))
    return json.loads(path.read_text(encoding="utf-8"))


def _terrain_point(
    npz_path: Path,
    metadata_path: Path,
    fallback: Dict[str, Any],
    x: float,
    y: float,
) -> Dict[str, Any]:
    if not npz_path.is_file():
        raise FileNotFoundError("Terrain NPZ not found: {}".format(npz_path))

    with np.load(npz_path) as data:
        required = (
            "elevation_m",
            "terrain_impedance",
            "validity_mask",
            "traversability_mask",
        )
        for key in required:
            if key not in data.files:
                raise KeyError("Terrain NPZ is missing array: {}".format(key))
        elevation = np.asarray(data["elevation_m"], dtype=np.float64)
        impedance = np.asarray(data["terrain_impedance"], dtype=np.float64)
        validity = np.asarray(data["validity_mask"], dtype=bool)
        traversability = np.asarray(data["traversability_mask"], dtype=bool)

    if metadata_path.is_file():
        metadata = _load_json(metadata_path)
        grid = metadata["grid"]
        resolution = float(grid["resolution_m"])
        origin_x = float(grid["origin_x_m"])
        origin_y = float(grid["origin_y_m"])
        width = int(grid["width_cells"])
        height = int(grid["height_cells"])
    else:
        resolution = float(fallback["resolution_m"])
        origin_x = float(fallback["origin_x_m"])
        origin_y = float(fallback["origin_y_m"])
        height, width = elevation.shape

    if elevation.shape != (height, width):
        raise RuntimeError(
            "Terrain array shape {} does not match metadata {}".format(
                elevation.shape, (height, width)
            )
        )

    column = int(math.floor((x - origin_x) / resolution))
    grid_row = int(math.floor((y - origin_y) / resolution))
    if column < 0 or column >= width or grid_row < 0 or grid_row >= height:
        raise RuntimeError(
            "Point ({:.3f}, {:.3f}) is outside terrain bounds".format(x, y)
        )
    source_row = height - 1 - grid_row
    return {
        "elevation_m": float(elevation[source_row, column]),
        "impedance": float(impedance[source_row, column]),
        "valid": bool(validity[source_row, column]),
        "traversable": bool(traversability[source_row, column]),
        "grid_row": grid_row,
        "source_row": source_row,
        "column": column,
        "resolution_m": resolution,
        "origin_x_m": origin_x,
        "origin_y_m": origin_y,
    }


def resolve_scenario(args: argparse.Namespace) -> argparse.Namespace:
    workspace = Path(args.workspace).expanduser().resolve()
    args.cost_model_config = str(
        Path(args.cost_model_config).expanduser().resolve()
    )
    config_path = _resolve_workspace_path(workspace, args.scenario_config)
    config = _load_json(config_path)
    scenarios = config.get("scenarios", {})
    if args.scenario not in scenarios:
        raise KeyError(
            "Unknown scenario '{}'. Available: {}".format(
                args.scenario, ", ".join(sorted(scenarios))
            )
        )

    defaults = dict(config.get("defaults", {}))
    scenario = dict(scenarios[args.scenario])
    start = dict(scenario.get("start", {}))
    goal = dict(scenario.get("goal", {}))

    args.workspace = str(workspace)
    args.scenario_config = str(config_path)
    args.terrain = args.terrain or scenario.get("terrain") or defaults.get("terrain", "hard")
    args.radiation_profile = (
        args.radiation_profile
        or scenario.get("radiation_profile")
        or defaults.get("radiation_profile", "unspecified")
    )

    world_value = args.world_path or scenario.get("world_path") or defaults.get("world_path")
    data_value = args.data_directory or scenario.get("data_directory") or defaults.get("data_directory")
    if not world_value or not data_value:
        raise RuntimeError("Scenario must define world_path and data_directory")
    args.world_path = str(_resolve_workspace_path(workspace, world_value))
    args.data_directory = str(_resolve_workspace_path(workspace, data_value))

    args.start_x = float(args.start_x if args.start_x is not None else start["x"])
    args.start_y = float(args.start_y if args.start_y is not None else start["y"])
    args.start_yaw = float(args.start_yaw if args.start_yaw is not None else start.get("yaw", 0.0))
    args.goal_x = float(args.goal_x if args.goal_x is not None else goal["x"])
    args.goal_y = float(args.goal_y if args.goal_y is not None else goal["y"])
    args.goal_tolerance = float(
        args.goal_tolerance
        if args.goal_tolerance is not None
        else scenario.get("goal_tolerance_m", defaults.get("goal_tolerance_m", 0.50))
    )
    args.start_tolerance = float(
        args.start_tolerance
        if args.start_tolerance is not None
        else scenario.get("start_tolerance_m", defaults.get("start_tolerance_m", 0.75))
    )
    args.spawn_clearance = float(
        args.spawn_clearance
        if args.spawn_clearance is not None
        else scenario.get("spawn_clearance_m", defaults.get("spawn_clearance_m", 0.30))
    )

    data_dir = Path(args.data_directory)
    npz_path = data_dir / "terrain_layers_{}.npz".format(args.terrain)
    metadata_path = data_dir / "terrain_layers_{}_metadata.json".format(args.terrain)
    fallback = dict(defaults.get("map_fallback", {}))
    if not fallback:
        raise RuntimeError("Scenario config requires defaults.map_fallback")

    start_info = _terrain_point(
        npz_path, metadata_path, fallback, args.start_x, args.start_y
    )
    goal_info = _terrain_point(
        npz_path, metadata_path, fallback, args.goal_x, args.goal_y
    )
    for label, info in (("start", start_info), ("goal", goal_info)):
        if not info["valid"]:
            raise RuntimeError("{} point is invalid in the terrain data".format(label))
        if not info["traversable"]:
            raise RuntimeError("{} point is marked non-traversable".format(label))

    if args.start_z is None:
        args.start_z = start_info["elevation_m"] + args.spawn_clearance
    else:
        args.start_z = float(args.start_z)

    args.start_elevation_m = start_info["elevation_m"]
    args.start_impedance = start_info["impedance"]
    args.goal_elevation_m = goal_info["elevation_m"]
    args.goal_impedance = goal_info["impedance"]
    args.scenario_description = scenario.get("description", "")

    if not Path(args.world_path).is_file():
        raise FileNotFoundError("World file not found: {}".format(args.world_path))
    return args


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Formal Husky Gazebo experiment runner v3")
    parser.add_argument("--planner", choices=("asd", "tp", "aco", "all"), default="asd")
    parser.add_argument("--run-id", default="001")
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument(
        "--cost-model-config",
        default=(
            "~/terrain_radiation_ws/src/radiation_mapping/"
            "config/final_cost_model_v1.json"
        ),
    )
    parser.add_argument("--scenario", default="H_B1_balanced")
    parser.add_argument(
        "--scenario-config",
        default="src/radiation_mapping/config/formal_scenarios_v1.json",
    )
    parser.add_argument("--world-path", default=None)
    parser.add_argument("--terrain", default=None)
    parser.add_argument("--data-directory", default=None)
    parser.add_argument("--radiation-profile", default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--goal-tolerance", type=float, default=None)
    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--start-z", type=float, default=None)
    parser.add_argument("--start-yaw", type=float, default=None)
    parser.add_argument("--spawn-clearance", type=float, default=None)
    parser.add_argument("--start-tolerance", type=float, default=None)
    parser.add_argument("--workspace", default="~/terrain_radiation_ws")
    parser.add_argument("--stack-timeout", type=float, default=120.0)
    parser.add_argument("--planner-timeout", type=float, default=60.0)
    parser.add_argument("--planning-timeout", type=float, default=300.0)
    parser.add_argument("--execution-start-timeout", type=float, default=60.0)
    parser.add_argument("--execution-timeout", type=float, default=360.0)
    parser.add_argument("--settle-time", type=float, default=10.0)
    parser.add_argument("--contact-timeline-period", type=float, default=0.02)
    return parser


def run_all(args: argparse.Namespace) -> int:
    script = str(Path(__file__).resolve())
    overall = 0
    for planner in ("asd", "tp", "aco"):
        command = [
            sys.executable, script,
            "--planner", planner,
            "--run-id", "{}_{}".format(args.run_id, planner),
            "--seed", str(args.seed),
            "--scenario", args.scenario,
            "--scenario-config", args.scenario_config,
            "--world-path", args.world_path,
            "--terrain", args.terrain,
            "--data-directory", args.data_directory,
            "--radiation-profile", args.radiation_profile,
            "--goal-x", str(args.goal_x),
            "--goal-y", str(args.goal_y),
            "--goal-tolerance", str(args.goal_tolerance),
            "--start-x", str(args.start_x),
            "--start-y", str(args.start_y),
            "--start-z", str(args.start_z),
            "--start-yaw", str(args.start_yaw),
            "--spawn-clearance", str(args.spawn_clearance),
            "--start-tolerance", str(args.start_tolerance),
            "--workspace", args.workspace,
            "--stack-timeout", str(args.stack_timeout),
            "--planner-timeout", str(args.planner_timeout),
            "--planning-timeout", str(args.planning_timeout),
            "--execution-start-timeout", str(args.execution_start_timeout),
            "--execution-timeout", str(args.execution_timeout),
            "--settle-time", str(args.settle_time),
            "--contact-timeline-period", str(args.contact_timeline_period),
        ]
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            overall = result.returncode
    return overall


def main() -> int:
    parser = build_parser()
    args = resolve_scenario(parser.parse_args())
    if args.planner == "all":
        return run_all(args)
    return FormalExperimentRunner(args).run()


if __name__ == "__main__":
    sys.exit(main())
