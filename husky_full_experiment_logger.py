#!/usr/bin/env python3
import csv
import math
import os
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64


NAN = float("nan")


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HuskyFullExperimentLogger(Node):
    """
    ROS 2 Foxy-compatible logger for Husky terrain/radiation experiments.

    Writes one time-aligned CSV containing:
      - Ground-truth pose/twist and derived motion
      - /cmd_vel and post-mux Husky controller command
      - Goal and ASD-RRT* path diagnostics
      - Terrain/radiation map values at the robot
      - Accumulated radiation dose
      - IMU
      - Four wheel joint positions/velocities/efforts
      - Stall and mux-mismatch diagnostic flags
    """

    def __init__(self):
        super().__init__("husky_full_experiment_logger")

        # ---------------- Parameters ----------------
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("experiment_label", "kd100")
        self.declare_parameter("terrain_kd", 100.0)
        self.declare_parameter(
            "output_dir",
            str(Path("~/terrain_radiation_ws/experiment_results").expanduser()),
        )

        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "controller_cmd_topic",
            "/husky_velocity_controller/cmd_vel_unstamped",
        )
        self.declare_parameter("path_topic", "/asd_rrt_star_path")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("terrain_topic", "/terrain_impedance_map")
        self.declare_parameter("radiation_topic", "/radiation_map")
        self.declare_parameter(
            "dose_topic",
            "/radiation/accumulated_dose_usv",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/data_raw")

        self.sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        self.experiment_label = str(self.get_parameter("experiment_label").value)
        self.terrain_kd = float(self.get_parameter("terrain_kd").value)
        self.output_dir = Path(
            os.path.expanduser(str(self.get_parameter("output_dir").value))
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # ---------------- State ----------------
        self.start_ros_time: Optional[float] = None
        self.sample_index = 0

        self.odom: Optional[Odometry] = None
        self.cmd: Optional[Twist] = None
        self.controller_cmd: Optional[Twist] = None
        self.path_msg: Optional[NavPath] = None
        self.path_points: List[Tuple[float, float, float]] = []
        self.path_length_m = NAN
        self.path_seq = 0
        self.progress_path_index = 0
        self.goal: Optional[PoseStamped] = None
        self.terrain_map: Optional[OccupancyGrid] = None
        self.radiation_map: Optional[OccupancyGrid] = None
        self.dose = NAN
        self.joint_state: Optional[JointState] = None
        self.imu: Optional[Imu] = None

        self.last_rx: Dict[str, Optional[float]] = {
            "odom": None,
            "cmd": None,
            "controller_cmd": None,
            "path": None,
            "goal": None,
            "terrain": None,
            "radiation": None,
            "dose": None,
            "joint": None,
            "imu": None,
        }

        self.prev_motion_time: Optional[float] = None
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.prev_z: Optional[float] = None
        self.prev_speed_xy: Optional[float] = None
        self.distance_travelled_2d_m = 0.0
        self.distance_travelled_3d_m = 0.0
        self.accel_from_speed_mps2 = NAN

        # ---------------- QoS ----------------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ---------------- Subscribers ----------------
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_callback,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("controller_cmd_topic").value),
            self.controller_cmd_callback,
            20,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("terrain_topic").value),
            self.terrain_callback,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("radiation_topic").value),
            self.radiation_callback,
            map_qos,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("dose_topic").value),
            self.dose_callback,
            20,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.imu_callback,
            qos_profile_sensor_data,
        )

        # ---------------- CSV ----------------
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / (
            f"husky_full_motion_{self.experiment_label}_{timestamp}.csv"
        )

        self.fieldnames = self.build_fieldnames()
        self.csv_file = open(self.csv_path, "w", newline="", buffering=1)
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()

        period = 1.0 / max(self.sample_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.sample)

        self.get_logger().info("Husky full experiment logger started.")
        self.get_logger().info(f"CSV: {self.csv_path}")
        self.get_logger().info(
            "Start this logger before publishing the Goal. "
            "Stop with Ctrl+C after the run."
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def mark_rx(self, key: str) -> None:
        self.last_rx[key] = self.now_s()

    def age(self, key: str, now: float) -> float:
        stamp = self.last_rx.get(key)
        if stamp is None:
            return NAN
        return max(0.0, now - stamp)

    # ---------------- Callbacks ----------------
    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.mark_rx("odom")

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd = msg
        self.mark_rx("cmd")

    def controller_cmd_callback(self, msg: Twist) -> None:
        self.controller_cmd = msg
        self.mark_rx("controller_cmd")

    def path_callback(self, msg: NavPath) -> None:
        self.path_msg = msg
        self.path_points = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in msg.poses
        ]
        self.path_length_m = 0.0
        for a, b in zip(self.path_points[:-1], self.path_points[1:]):
            self.path_length_m += math.hypot(b[0] - a[0], b[1] - a[1])
        self.path_seq += 1
        self.progress_path_index = 0
        self.mark_rx("path")

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.mark_rx("goal")

    def terrain_callback(self, msg: OccupancyGrid) -> None:
        self.terrain_map = msg
        self.mark_rx("terrain")

    def radiation_callback(self, msg: OccupancyGrid) -> None:
        self.radiation_map = msg
        self.mark_rx("radiation")

    def dose_callback(self, msg: Float64) -> None:
        self.dose = float(msg.data)
        self.mark_rx("dose")

    def joint_callback(self, msg: JointState) -> None:
        self.joint_state = msg
        self.mark_rx("joint")

    def imu_callback(self, msg: Imu) -> None:
        self.imu = msg
        self.mark_rx("imu")

    # ---------------- Helpers ----------------
    @staticmethod
    def grid_value(grid: Optional[OccupancyGrid], x: float, y: float) -> float:
        if grid is None:
            return NAN
        res = grid.info.resolution
        if res <= 0.0:
            return NAN
        gx = int(math.floor((x - grid.info.origin.position.x) / res))
        gy = int(math.floor((y - grid.info.origin.position.y) / res))
        if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
            return NAN
        idx = gy * grid.info.width + gx
        if idx < 0 or idx >= len(grid.data):
            return NAN
        value = int(grid.data[idx])
        return NAN if value < 0 else float(value)

    @staticmethod
    def grid_local_stats(
        grid: Optional[OccupancyGrid], x: float, y: float, radius_cells: int = 2
    ) -> Tuple[float, float, float]:
        if grid is None or grid.info.resolution <= 0.0:
            return NAN, NAN, NAN

        res = grid.info.resolution
        cx = int(math.floor((x - grid.info.origin.position.x) / res))
        cy = int(math.floor((y - grid.info.origin.position.y) / res))
        values = []

        for gy in range(cy - radius_cells, cy + radius_cells + 1):
            for gx in range(cx - radius_cells, cx + radius_cells + 1):
                if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
                    idx = gy * grid.info.width + gx
                    if 0 <= idx < len(grid.data):
                        v = int(grid.data[idx])
                        if v >= 0:
                            values.append(float(v))

        if not values:
            return NAN, NAN, NAN
        return min(values), sum(values) / len(values), max(values)

    def nearest_path_diagnostics(
        self, x: float, y: float
    ) -> Tuple[int, float, float, float, float, int, float, float, float, float]:
        if not self.path_points:
            return -1, NAN, NAN, NAN, NAN, -1, NAN, NAN, NAN, NAN

        best_i = 0
        best_d = float("inf")
        for i, p in enumerate(self.path_points):
            d = math.hypot(p[0] - x, p[1] - y)
            if d < best_d:
                best_d = d
                best_i = i

        # Monotonic progress estimate for diagnostics only.
        self.progress_path_index = max(self.progress_path_index, best_i)
        progress_i = min(self.progress_path_index, len(self.path_points) - 1)

        nearest = self.path_points[best_i]
        next_i = min(progress_i + 1, len(self.path_points) - 1)
        next_p = self.path_points[next_i]
        dist_next = math.hypot(next_p[0] - x, next_p[1] - y)

        remaining = 0.0
        if progress_i < len(self.path_points) - 1:
            remaining += math.hypot(
                self.path_points[progress_i][0] - x,
                self.path_points[progress_i][1] - y,
            )
            for a, b in zip(
                self.path_points[progress_i:-1],
                self.path_points[progress_i + 1:],
            ):
                remaining += math.hypot(b[0] - a[0], b[1] - a[1])

        return (
            best_i,
            best_d,
            nearest[0],
            nearest[1],
            float(progress_i),
            next_i,
            next_p[0],
            next_p[1],
            dist_next,
            remaining,
        )

    def wheel_value(self, position_or_velocity: str, side_token: str) -> float:
        msg = self.joint_state
        if msg is None:
            return NAN

        values = (
            msg.position
            if position_or_velocity == "position"
            else msg.velocity
            if position_or_velocity == "velocity"
            else msg.effort
        )

        for i, name in enumerate(msg.name):
            n = name.lower()
            if "wheel" in n and side_token in n and i < len(values):
                return float(values[i])
        return NAN

    def build_fieldnames(self) -> List[str]:
        return [
            # Experiment/time
            "experiment_label", "terrain_kd", "sample_index",
            "ros_time_s", "elapsed_s",

            # Ground truth pose
            "gt_x_m", "gt_y_m", "gt_z_m",
            "gt_roll_rad", "gt_pitch_rad", "gt_yaw_rad",
            "gt_roll_deg", "gt_pitch_deg", "gt_yaw_deg",

            # Ground truth twist / derived motion
            "gt_vx_mps", "gt_vy_mps", "gt_vz_mps",
            "gt_speed_xy_mps", "gt_speed_3d_mps",
            "gt_wx_radps", "gt_wy_radps", "gt_wz_radps",
            "derived_accel_from_speed_mps2",
            "distance_travelled_2d_m", "distance_travelled_3d_m",

            # Commands
            "cmd_linear_x_mps", "cmd_linear_y_mps", "cmd_angular_z_radps",
            "controller_linear_x_mps", "controller_linear_y_mps",
            "controller_angular_z_radps",
            "cmd_controller_linear_diff_mps",
            "cmd_controller_angular_diff_radps",

            # Goal
            "goal_x_m", "goal_y_m", "goal_distance_m",
            "goal_bearing_rad",
            "visual_front_target_yaw_rad",
            "visual_front_goal_yaw_error_rad",

            # Path
            "path_seq", "path_pose_count", "path_length_m",
            "nearest_path_index", "nearest_path_distance_m",
            "nearest_path_x_m", "nearest_path_y_m",
            "progress_path_index",
            "next_path_index", "next_path_x_m", "next_path_y_m",
            "distance_to_next_path_point_m",
            "estimated_path_remaining_m",

            # Terrain/radiation/dose
            "terrain_value_robot",
            "terrain_local_min_5x5", "terrain_local_mean_5x5",
            "terrain_local_max_5x5",
            "radiation_value_robot",
            "radiation_local_min_5x5", "radiation_local_mean_5x5",
            "radiation_local_max_5x5",
            "accumulated_dose",

            # IMU
            "imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad",
            "imu_wx_radps", "imu_wy_radps", "imu_wz_radps",
            "imu_ax_mps2", "imu_ay_mps2", "imu_az_mps2",

            # Wheels
            "front_left_wheel_pos_rad", "front_left_wheel_vel_radps",
            "front_left_wheel_effort",
            "front_right_wheel_pos_rad", "front_right_wheel_vel_radps",
            "front_right_wheel_effort",
            "rear_left_wheel_pos_rad", "rear_left_wheel_vel_radps",
            "rear_left_wheel_effort",
            "rear_right_wheel_pos_rad", "rear_right_wheel_vel_radps",
            "rear_right_wheel_effort",

            # Diagnostics / freshness
            "odom_age_s", "cmd_age_s", "controller_cmd_age_s",
            "path_age_s", "goal_age_s", "terrain_age_s",
            "radiation_age_s", "dose_age_s", "joint_age_s", "imu_age_s",
            "linear_stall_flag", "rotation_stall_flag",
            "mux_mismatch_flag", "diagnostic_state",
        ]

    def sample(self) -> None:
        now = self.now_s()
        if self.start_ros_time is None:
            self.start_ros_time = now

        row = {key: NAN for key in self.fieldnames}
        row.update({
            "experiment_label": self.experiment_label,
            "terrain_kd": self.terrain_kd,
            "sample_index": self.sample_index,
            "ros_time_s": now,
            "elapsed_s": now - self.start_ros_time,
            "path_seq": self.path_seq,
            "path_pose_count": len(self.path_points),
            "path_length_m": self.path_length_m,
            "accumulated_dose": self.dose,
        })

        x = y = z = NAN
        yaw = NAN
        speed_xy = NAN
        gt_wz = NAN

        if self.odom is not None:
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

            t = self.odom.twist.twist
            speed_xy = math.hypot(t.linear.x, t.linear.y)
            speed_3d = math.sqrt(
                t.linear.x * t.linear.x
                + t.linear.y * t.linear.y
                + t.linear.z * t.linear.z
            )
            gt_wz = t.angular.z
            x, y, z = p.x, p.y, p.z

            if self.prev_motion_time is not None:
                dt = now - self.prev_motion_time
                if dt > 1e-6:
                    if self.prev_x is not None:
                        self.distance_travelled_2d_m += math.hypot(
                            x - self.prev_x, y - self.prev_y
                        )
                        self.distance_travelled_3d_m += math.sqrt(
                            (x - self.prev_x) ** 2
                            + (y - self.prev_y) ** 2
                            + (z - self.prev_z) ** 2
                        )
                    if self.prev_speed_xy is not None:
                        self.accel_from_speed_mps2 = (
                            speed_xy - self.prev_speed_xy
                        ) / dt

            self.prev_motion_time = now
            self.prev_x, self.prev_y, self.prev_z = x, y, z
            self.prev_speed_xy = speed_xy

            row.update({
                "gt_x_m": x, "gt_y_m": y, "gt_z_m": z,
                "gt_roll_rad": roll, "gt_pitch_rad": pitch, "gt_yaw_rad": yaw,
                "gt_roll_deg": math.degrees(roll),
                "gt_pitch_deg": math.degrees(pitch),
                "gt_yaw_deg": math.degrees(yaw),
                "gt_vx_mps": t.linear.x,
                "gt_vy_mps": t.linear.y,
                "gt_vz_mps": t.linear.z,
                "gt_speed_xy_mps": speed_xy,
                "gt_speed_3d_mps": speed_3d,
                "gt_wx_radps": t.angular.x,
                "gt_wy_radps": t.angular.y,
                "gt_wz_radps": t.angular.z,
                "derived_accel_from_speed_mps2": self.accel_from_speed_mps2,
                "distance_travelled_2d_m": self.distance_travelled_2d_m,
                "distance_travelled_3d_m": self.distance_travelled_3d_m,
            })

        cmd_lin = cmd_ang = NAN
        if self.cmd is not None:
            cmd_lin = self.cmd.linear.x
            cmd_ang = self.cmd.angular.z
            row.update({
                "cmd_linear_x_mps": self.cmd.linear.x,
                "cmd_linear_y_mps": self.cmd.linear.y,
                "cmd_angular_z_radps": self.cmd.angular.z,
            })

        ctl_lin = ctl_ang = NAN
        if self.controller_cmd is not None:
            ctl_lin = self.controller_cmd.linear.x
            ctl_ang = self.controller_cmd.angular.z
            row.update({
                "controller_linear_x_mps": self.controller_cmd.linear.x,
                "controller_linear_y_mps": self.controller_cmd.linear.y,
                "controller_angular_z_radps": self.controller_cmd.angular.z,
            })

        if not math.isnan(cmd_lin) and not math.isnan(ctl_lin):
            row["cmd_controller_linear_diff_mps"] = cmd_lin - ctl_lin
        if not math.isnan(cmd_ang) and not math.isnan(ctl_ang):
            row["cmd_controller_angular_diff_radps"] = cmd_ang - ctl_ang

        if self.goal is not None:
            gx = self.goal.pose.position.x
            gy = self.goal.pose.position.y
            row["goal_x_m"] = gx
            row["goal_y_m"] = gy
            if not math.isnan(x):
                dx, dy = gx - x, gy - y
                bearing = math.atan2(dy, dx)
                visual_target = normalize_angle(bearing + math.pi)
                row["goal_distance_m"] = math.hypot(dx, dy)
                row["goal_bearing_rad"] = bearing
                row["visual_front_target_yaw_rad"] = visual_target
                if not math.isnan(yaw):
                    row["visual_front_goal_yaw_error_rad"] = normalize_angle(
                        visual_target - yaw
                    )

        if not math.isnan(x) and self.path_points:
            (
                near_i, near_d, near_x, near_y, progress_i,
                next_i, next_x, next_y, next_d, remaining
            ) = self.nearest_path_diagnostics(x, y)
            row.update({
                "nearest_path_index": near_i,
                "nearest_path_distance_m": near_d,
                "nearest_path_x_m": near_x,
                "nearest_path_y_m": near_y,
                "progress_path_index": progress_i,
                "next_path_index": next_i,
                "next_path_x_m": next_x,
                "next_path_y_m": next_y,
                "distance_to_next_path_point_m": next_d,
                "estimated_path_remaining_m": remaining,
            })

        if not math.isnan(x):
            row["terrain_value_robot"] = self.grid_value(self.terrain_map, x, y)
            tmin, tmean, tmax = self.grid_local_stats(self.terrain_map, x, y, 2)
            row["terrain_local_min_5x5"] = tmin
            row["terrain_local_mean_5x5"] = tmean
            row["terrain_local_max_5x5"] = tmax

            row["radiation_value_robot"] = self.grid_value(
                self.radiation_map, x, y
            )
            rmin, rmean, rmax = self.grid_local_stats(
                self.radiation_map, x, y, 2
            )
            row["radiation_local_min_5x5"] = rmin
            row["radiation_local_mean_5x5"] = rmean
            row["radiation_local_max_5x5"] = rmax

        if self.imu is not None:
            q = self.imu.orientation
            ir, ip, iy = quaternion_to_euler(q.x, q.y, q.z, q.w)
            row.update({
                "imu_roll_rad": ir,
                "imu_pitch_rad": ip,
                "imu_yaw_rad": iy,
                "imu_wx_radps": self.imu.angular_velocity.x,
                "imu_wy_radps": self.imu.angular_velocity.y,
                "imu_wz_radps": self.imu.angular_velocity.z,
                "imu_ax_mps2": self.imu.linear_acceleration.x,
                "imu_ay_mps2": self.imu.linear_acceleration.y,
                "imu_az_mps2": self.imu.linear_acceleration.z,
            })

        wheel_map = {
            "front_left": "front_left",
            "front_right": "front_right",
            "rear_left": "rear_left",
            "rear_right": "rear_right",
        }
        for prefix, token in wheel_map.items():
            row[f"{prefix}_wheel_pos_rad"] = self.wheel_value("position", token)
            row[f"{prefix}_wheel_vel_radps"] = self.wheel_value("velocity", token)
            row[f"{prefix}_wheel_effort"] = self.wheel_value("effort", token)

        for key in self.last_rx:
            row[f"{key}_age_s" if key != "joint" else "joint_age_s"] = self.age(
                key, now
            )

        # Correct the generated names for keys whose CSV headers differ.
        row["controller_cmd_age_s"] = self.age("controller_cmd", now)
        row["radiation_age_s"] = self.age("radiation", now)
        row["terrain_age_s"] = self.age("terrain", now)
        row["goal_age_s"] = self.age("goal", now)
        row["path_age_s"] = self.age("path", now)
        row["dose_age_s"] = self.age("dose", now)
        row["imu_age_s"] = self.age("imu", now)
        row["odom_age_s"] = self.age("odom", now)
        row["cmd_age_s"] = self.age("cmd", now)

        linear_stall = (
            not math.isnan(cmd_lin)
            and abs(cmd_lin) > 0.05
            and not math.isnan(speed_xy)
            and speed_xy < 0.02
        )
        rotation_stall = (
            not math.isnan(cmd_ang)
            and abs(cmd_ang) > 0.10
            and not math.isnan(gt_wz)
            and abs(gt_wz) < 0.03
        )
        mux_mismatch = (
            not math.isnan(cmd_lin)
            and not math.isnan(ctl_lin)
            and (
                abs(cmd_lin - ctl_lin) > 0.03
                or (
                    not math.isnan(cmd_ang)
                    and not math.isnan(ctl_ang)
                    and abs(cmd_ang - ctl_ang) > 0.08
                )
            )
        )

        row["linear_stall_flag"] = int(linear_stall)
        row["rotation_stall_flag"] = int(rotation_stall)
        row["mux_mismatch_flag"] = int(mux_mismatch)

        if self.odom is None:
            state = "WAIT_ODOM"
        elif self.path_msg is None:
            state = "WAIT_PATH"
        elif mux_mismatch:
            state = "MUX_MISMATCH"
        elif rotation_stall:
            state = "ROTATION_STALL"
        elif linear_stall:
            state = "LINEAR_STALL"
        elif not math.isnan(speed_xy) and speed_xy > 0.03:
            state = "MOVING"
        elif (
            (not math.isnan(cmd_lin) and abs(cmd_lin) > 0.02)
            or (not math.isnan(cmd_ang) and abs(cmd_ang) > 0.05)
        ):
            state = "COMMAND_ACTIVE_LOW_MOTION"
        else:
            state = "IDLE"

        row["diagnostic_state"] = state

        self.writer.writerow(row)
        self.sample_index += 1

    def destroy_node(self):
        try:
            if hasattr(self, "csv_file") and self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
                self.get_logger().info(f"CSV saved: {self.csv_path}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HuskyFullExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
