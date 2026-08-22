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
from typing import Dict, List, Optional, Sequence, Tuple

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

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_reliable = QoSProfile(
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
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/radiation_map",
            self.radiation_callback,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.odom_callback,
            volatile_reliable,
        )
        self.create_subscription(
            Float64,
            "/radiation/accumulated_dose_usv",
            self.dose_callback,
            volatile_reliable,
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
            volatile_reliable,
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            volatile_reliable,
        )
        self.stop_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            volatile_reliable,
        )

    def terrain_callback(self, msg: OccupancyGrid) -> None:
        del msg
        self.terrain_received = True

    def radiation_callback(self, msg: OccupancyGrid) -> None:
        del msg
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
            / "formal_experiments"
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
            "gazebo_radiation_husky.launch.py",
            "terrain_services.launch.py",
            "final_asd_rrt_star_planner",
            "final_aco.launch.py",
            "aco_planner",
            "asd_path_waypoint_follower",
            "formal_contact_collector_v2.py",
        )
        for pattern in patterns:
            subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        subprocess.run(["pkill", "-TERM", "-x", "gzserver"], check=False)
        subprocess.run(["pkill", "-TERM", "-x", "gzclient"], check=False)
        time.sleep(4.0)
        subprocess.run(["pkill", "-KILL", "-x", "gzserver"], check=False)
        subprocess.run(["pkill", "-KILL", "-x", "gzclient"], check=False)
        subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        time.sleep(1.0)
        subprocess.run(["ros2", "daemon", "start"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        time.sleep(2.0)

    def validate_files(self) -> None:
        required = [
            self.launch_dir / "gazebo_radiation_husky.launch.py",
            self.launch_dir / "terrain_services.launch.py",
            self.launch_dir / "module_final_asd_rrt_star.launch.py",
            self.collector_script,
        ]
        if self.args.planner == "aco":
            required.append(self.launch_dir / "final_aco.launch.py")
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
        gazebo_launch = self.launch_dir / "gazebo_radiation_husky.launch.py"
        terrain_launch = self.launch_dir / "terrain_services.launch.py"
        self.start_process(
            "gazebo",
            [
                "ros2", "launch", str(gazebo_launch),
                "start_x:={}".format(self.args.start_x),
                "start_y:={}".format(self.args.start_y),
                "start_z:={}".format(self.args.start_z),
                "start_yaw:={}".format(self.args.start_yaw),
                "use_sim_time:=true",
            ],
            "gazebo.log",
        )
        time.sleep(5.0)
        self.start_process(
            "terrain_services",
            [
                "ros2", "launch", str(terrain_launch),
                "terrain:=hard", "use_sim_time:=true",
            ],
            "terrain.log",
        )

    def wait_for_environment(self) -> None:
        assert self.node is not None
        ready = self.spin_until(
            lambda: self.node.terrain_received and self.node.radiation_received and self.node.odom is not None,
            self.args.stack_timeout,
            "terrain map + radiation map + ground-truth odom",
        )
        if not ready:
            raise RuntimeError("Environment inputs did not become ready")
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
                "include_time_penalty:={}".format(config["include_time_penalty"]),
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
            command = ["ros2", "launch", str(self.launch_dir / "final_aco.launch.py")]
        self.planner_process = self.start_process("planner", command, "planner.log")
        assert self.node is not None
        if not self.spin_until(
            lambda: self.node.count_publishers(config["path_topic"]) >= 1
            and self.node.goal_publisher.get_subscription_count() >= 1,
            self.args.planner_timeout,
            "planner path publisher + /goal_pose subscriber",
        ):
            raise RuntimeError("Planner endpoints did not become ready")
        self.apply_seed(config["node_name"])
        time.sleep(5.0)

    def apply_seed(self, node_name: str) -> None:
        result = self.run_command(["ros2", "param", "list", node_name], timeout=10.0)
        parameters = result.stdout.splitlines()
        parameter_name = None
        for candidate in ("random_seed", "seed"):
            if any(line.strip() == candidate for line in parameters):
                parameter_name = candidate
                break
        if parameter_name is not None:
            set_result = self.run_command(
                ["ros2", "param", "set", node_name, parameter_name, str(self.args.seed)],
                timeout=10.0,
            )
            if set_result.returncode != 0 or "Successful" not in set_result.stdout:
                raise RuntimeError("Could not set planner seed: " + set_result.stdout.strip())
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
                "ros2", "run", "radiation_mapping", "asd_path_waypoint_follower",
                "--ros-args",
                "-p", "path_topic:={}".format(config["path_topic"]),
                "-p", "planner_name:={}".format(config["label"]),
                "-p", "dose_topic:=/radiation/accumulated_dose_usv",
                "-p", "terrain_topic:=/terrain_impedance_map",
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

        contact_pass = bool(contact.get("acceptance", {}).get("overall_pass", False))
        success_conditions = {
            "follower_exit_zero": follower_code == 0,
            "follower_csv_written": bool(follower),
            "path_has_at_least_3_poses": len(points) >= 3,
            "final_goal_error_within_tolerance": goal_error is not None and goal_error <= self.args.goal_tolerance,
            "contact_acceptance_pass": contact_pass,
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
            "terrain": "hard",
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
            "planning_time_wall_s": planning_time,
            "execution_time_wall_s": execution_wall,
            "execution_time_follower_s": self.float_or_none(follower.get("execution_time_s")),
            "executed_path_length_m": self.float_or_none(follower.get("executed_path_length_m")),
            "dose_during_execution_usv": self.float_or_none(follower.get("dose_during_path_following")),
            "executed_terrain_cost": self.float_or_none(follower.get("executed_terrain_cost")),
            "executed_final_coupled_score": self.float_or_none(follower.get("executed_final_coupled_score")),
            "dose_monitor_total_dose_usv": self.float_or_none(follower.get("dose_monitor_total_dose")),
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

    def append_master_csv(self, summary: Dict[str, object]) -> None:
        master = self.ws / "formal_experiments/formal_execution_summary_v2.csv"
        master.parent.mkdir(parents=True, exist_ok=True)
        flattened = {key: value for key, value in summary.items() if key != "success_conditions"}
        exists = master.is_file()
        with master.open("a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=list(flattened.keys()))
            if not exists:
                writer.writeheader()
            writer.writerow(flattened)

    def write_manifest(self, config: Dict[str, str]) -> None:
        manifest = {
            "runner_version": "2.0",
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
        print("FORMAL EXPERIMENT RUNNER V2")
        print("planner  = {}".format(config["label"]))
        print("run_id   = {}".format(self.args.run_id))
        print("seed     = {}".format(self.args.seed))
        print("goal     = ({:.3f}, {:.3f})".format(self.args.goal_x, self.args.goal_y))
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Formal Husky Gazebo experiment runner v2")
    parser.add_argument("--planner", choices=("asd", "tp", "aco", "all"), default="asd")
    parser.add_argument("--run-id", default="001")
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--scenario", default="hard_goal_m1p13_m7p80")
    parser.add_argument("--goal-x", type=float, default=-1.13)
    parser.add_argument("--goal-y", type=float, default=-7.80)
    parser.add_argument("--goal-tolerance", type=float, default=0.50)
    parser.add_argument("--start-x", type=float, default=5.134)
    parser.add_argument("--start-y", type=float, default=5.977)
    parser.add_argument("--start-z", type=float, default=0.448)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--start-tolerance", type=float, default=0.75)
    parser.add_argument("--workspace", default="~/terrain_radiation_ws")
    parser.add_argument("--stack-timeout", type=float, default=90.0)
    parser.add_argument("--planner-timeout", type=float, default=45.0)
    parser.add_argument("--planning-timeout", type=float, default=240.0)
    parser.add_argument("--execution-start-timeout", type=float, default=45.0)
    parser.add_argument("--execution-timeout", type=float, default=300.0)
    parser.add_argument("--settle-time", type=float, default=8.0)
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
            "--goal-x", str(args.goal_x),
            "--goal-y", str(args.goal_y),
            "--goal-tolerance", str(args.goal_tolerance),
            "--start-x", str(args.start_x),
            "--start-y", str(args.start_y),
            "--start-z", str(args.start_z),
            "--start-yaw", str(args.start_yaw),
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
    args = parser.parse_args()
    if args.planner == "all":
        return run_all(args)
    return FormalExperimentRunner(args).run()


if __name__ == "__main__":
    sys.exit(main())
