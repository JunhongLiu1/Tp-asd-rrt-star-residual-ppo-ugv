import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64


class ASDPathWaypointFollower(Node):
    def __init__(self):
        super().__init__('formal_path_waypoint_follower')

        self.declare_parameter(
            'planner_name',
            'ASD-Time-Aware RRT*'
        )

        self.declare_parameter(
            'path_topic',
            '/asd_time_aware_rrt_star_path'
        )

        self.declare_parameter(
            'dose_topic',
            '/robot_accumulated_dose'
        )

        self.declare_parameter(
            'terrain_topic',
            '/terrain_cost_map'
        )

        self.declare_parameter(
            'radiation_topic',
            '/radiation_map'
        )

        self.declare_parameter(
            'result_csv',
            '~/terrain_radiation_ws/experiment_results/execution_results.csv'
        )

        self.declare_parameter(
            'shutdown_on_finish',
            True
        )

        self.declare_parameter('odom_to_map_x', 0.0)
        self.declare_parameter('odom_to_map_y', 0.0)
        self.declare_parameter('odom_to_map_yaw', 0.0)

        self.planner_name = (
            self.get_parameter('planner_name')
            .get_parameter_value()
            .string_value
        )

        self.path_topic = (
            self.get_parameter('path_topic')
            .get_parameter_value()
            .string_value
        )

        self.dose_topic = (
            self.get_parameter('dose_topic')
            .get_parameter_value()
            .string_value
        )

        self.terrain_topic = (
            self.get_parameter('terrain_topic')
            .get_parameter_value()
            .string_value
        )

        self.radiation_topic = (
            self.get_parameter('radiation_topic')
            .get_parameter_value()
            .string_value
        )

        self.result_csv = os.path.expanduser(
            self.get_parameter('result_csv')
            .get_parameter_value()
            .string_value
        )

        self.shutdown_on_finish = (
            self.get_parameter('shutdown_on_finish')
            .get_parameter_value()
            .bool_value
        )

        self.odom_to_map_x = (
            self.get_parameter('odom_to_map_x')
            .get_parameter_value()
            .double_value
        )
        self.odom_to_map_y = (
            self.get_parameter('odom_to_map_y')
            .get_parameter_value()
            .double_value
        )
        self.odom_to_map_yaw = (
            self.get_parameter('odom_to_map_yaw')
            .get_parameter_value()
            .double_value
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.original_waypoints = []
        self.execution_waypoints = []

        self.current_waypoint_index = 0
        self.path_received = False
        self.execution_path_initialized = False
        self.finished = False

        self.current_total_dose = 0.0
        self.dose_received = False
        self.start_total_dose = None
        self.finish_reported = False

        self.terrain_map = None
        self.terrain_map_received = False
        self.radiation_map = None
        self.radiation_map_received = False

        self.start_time = None
        self.executed_path_length = 0.0
        self.executed_terrain_cost = 0.0
        self.terrain_valid_sample_count = 0
        self.terrain_out_of_bounds_count = 0
        self.executed_radiation_map_cost = 0.0
        self.radiation_valid_sample_count = 0
        self.radiation_out_of_bounds_count = 0

        self.metric_last_x = None
        self.metric_last_y = None

        # Do not shift the whole path automatically.
        # The planner path and Gazebo odometry should use the same coordinate frame.
        self.shift_path_to_robot_start = False

        # If the robot is far from the first waypoint, start from the nearest waypoint
        # instead of trying to drive back to the path start.
        self.allow_start_from_nearest_waypoint = True
        self.start_distance_warning_threshold = 1.0

        # Tracking parameters
        self.waypoint_tolerance = 0.35
        self.goal_tolerance = 0.40
        self.lookahead_distance = 0.80

        self.max_linear_speed = 0.24
        self.max_angular_speed = 0.70
        self.min_linear_speed = 0.03

        # Husky skid-steer cannot reliably rotate in place on rough DEM.
        # Use a small forward speed during large-heading corrections.
        self.turning_linear_speed = 0.12

        self.k_linear = 0.45
        self.k_angular = 1.20

        # Search window for nearest waypoint update.
        # This prevents the follower from jumping directly to the final waypoint.
        self.nearest_search_window = 6

        # Execution score weights
        self.dose_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'

        self.last_record_x = None
        self.last_record_y = None

        self.last_debug_time = self.get_clock().now()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        self.dose_sub = self.create_subscription(
            Float64,
            self.dose_topic,
            self.dose_callback,
            10
        )

        self.terrain_sub = self.create_subscription(
            OccupancyGrid,
            self.terrain_topic,
            self.terrain_callback,
            map_qos
        )

        self.radiation_sub = self.create_subscription(
            OccupancyGrid,
            self.radiation_topic,
            self.radiation_callback,
            map_qos
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.executed_path_pub = self.create_publisher(
            Path,
            '/executed_path',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Formal path waypoint follower started.')
        self.get_logger().info(f'Planner name: {self.planner_name}')
        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Dose topic: {self.dose_topic}')
        self.get_logger().info(f'Terrain topic: {self.terrain_topic}')
        self.get_logger().info(f'Radiation map topic: {self.radiation_topic}')
        self.get_logger().info(f'Result CSV: {self.result_csv}')
        self.get_logger().info(
            'Waiting for odom, path, dose, terrain map, and radiation map...'
        )

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        cos_yaw = math.cos(self.odom_to_map_yaw)
        sin_yaw = math.sin(self.odom_to_map_yaw)

        new_x = (
            self.odom_to_map_x
            + cos_yaw * odom_x
            - sin_yaw * odom_y
        )
        new_y = (
            self.odom_to_map_y
            + sin_yaw * odom_x
            + cos_yaw * odom_y
        )

        new_yaw = self.normalize_angle(
            odom_yaw + self.odom_to_map_yaw
        )

        if self.execution_path_initialized and not self.finished:
            self.update_execution_metrics(new_x, new_y)

        self.current_x = new_x
        self.current_y = new_y
        self.current_yaw = new_yaw
        self.has_odom = True

        self.record_executed_path()

    def path_callback(self, msg):
        # The planner publishes the path continuously.
        # To avoid resetting the follower repeatedly, only accept the first path.
        if self.path_received:
            return

        if len(msg.poses) == 0:
            return

        self.original_waypoints = []

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.original_waypoints.append((x, y))

        self.path_received = True

        self.get_logger().info(
            f'Received path with {len(self.original_waypoints)} waypoints.'
        )

    def dose_callback(self, msg):
        self.current_total_dose = msg.data
        self.dose_received = True

    def terrain_callback(self, msg):
        self.terrain_map = msg
        self.terrain_map_received = True

    def radiation_callback(self, msg):
        self.radiation_map = msg
        self.radiation_map_received = True

    def initialize_execution_path(self):
        if not self.has_odom or not self.path_received:
            return

        if len(self.original_waypoints) == 0:
            return

        path_start_x, path_start_y = self.original_waypoints[0]

        distance_to_path_start = math.sqrt(
            (self.current_x - path_start_x) ** 2 +
            (self.current_y - path_start_y) ** 2
        )

        if distance_to_path_start > self.start_distance_warning_threshold:
            self.get_logger().warn(
                f'Robot is far from the planned path start. '
                f'Robot: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Path start: ({path_start_x:.2f}, {path_start_y:.2f}).'
            )

        if self.shift_path_to_robot_start:
            offset_x = self.current_x - path_start_x
            offset_y = self.current_y - path_start_y
        else:
            offset_x = 0.0
            offset_y = 0.0

        self.execution_waypoints = []

        for x, y in self.original_waypoints:
            self.execution_waypoints.append(
                (x + offset_x, y + offset_y)
            )

        self.current_waypoint_index = 0

        if self.allow_start_from_nearest_waypoint:
            nearest_index = self.find_nearest_waypoint_index(
                start_index=0,
                end_index=len(self.execution_waypoints)
            )

            self.current_waypoint_index = nearest_index

            self.get_logger().info(
                f'Initial nearest waypoint index: {nearest_index}.'
            )
        else:
            if len(self.execution_waypoints) > 1:
                first_x, first_y = self.execution_waypoints[0]
                distance_to_first = math.sqrt(
                    (self.current_x - first_x) ** 2 +
                    (self.current_y - first_y) ** 2
                )

                if distance_to_first < self.waypoint_tolerance:
                    self.current_waypoint_index = 1

        self.execution_path_initialized = True

        if self.dose_received:
            self.start_total_dose = self.current_total_dose
            self.get_logger().info(
                f'Start dose recorded: {self.start_total_dose:.2f}'
            )
        else:
            self.start_total_dose = 0.0
            self.get_logger().warn(
                'No dose message received yet. Start dose is set to 0.0.'
            )

        if not self.terrain_map_received:
            self.get_logger().warn(
                'No terrain map received yet. Executed terrain cost may be 0.0.'
            )

        if not self.radiation_map_received:
            self.get_logger().warn(
                'No radiation map received yet. Executed radiation-map cost may be 0.0.'
            )

        self.start_time = self.get_clock().now()

        self.executed_path_length = 0.0
        self.executed_terrain_cost = 0.0
        self.terrain_valid_sample_count = 0
        self.terrain_out_of_bounds_count = 0
        self.executed_radiation_map_cost = 0.0
        self.radiation_valid_sample_count = 0
        self.radiation_out_of_bounds_count = 0

        self.metric_last_x = self.current_x
        self.metric_last_y = self.current_y

        self.get_logger().info('Execution path initialized.')
        self.get_logger().info(
            f'Start following from waypoint index '
            f'{self.current_waypoint_index}.'
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def world_to_map_index(self, map_msg, x, y):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return None

        return map_y * width + map_x

    def get_terrain_value(self, x, y):
        if self.terrain_map is None:
            return None

        index = self.world_to_map_index(self.terrain_map, x, y)

        if index is None:
            return None

        value = self.terrain_map.data[index]

        if value < 0:
            value = 100

        return float(value)

    def get_radiation_value(self, x, y):
        if self.radiation_map is None:
            return None

        index = self.world_to_map_index(self.radiation_map, x, y)

        if index is None:
            return None

        value = self.radiation_map.data[index]

        # OccupancyGrid unknown cells are treated conservatively as maximum risk.
        if value < 0:
            value = 100

        return float(value)

    def update_execution_metrics(self, new_x, new_y):
        if self.metric_last_x is None or self.metric_last_y is None:
            self.metric_last_x = new_x
            self.metric_last_y = new_y
            return

        dx = new_x - self.metric_last_x
        dy = new_y - self.metric_last_y

        segment_distance = math.sqrt(dx * dx + dy * dy)

        if segment_distance < 1e-6:
            return

        self.executed_path_length += segment_distance

        midpoint_x = 0.5 * (new_x + self.metric_last_x)
        midpoint_y = 0.5 * (new_y + self.metric_last_y)

        terrain_value = self.get_terrain_value(midpoint_x, midpoint_y)

        if terrain_value is not None:
            self.terrain_valid_sample_count += 1
            segment_terrain_cost = (terrain_value / 10.0) * segment_distance
            self.executed_terrain_cost += segment_terrain_cost
        else:
            self.terrain_out_of_bounds_count += 1

        radiation_value = self.get_radiation_value(midpoint_x, midpoint_y)

        if radiation_value is not None:
            self.radiation_valid_sample_count += 1
            segment_radiation_cost = (
                radiation_value / 100.0
            ) * segment_distance
            self.executed_radiation_map_cost += segment_radiation_cost
        else:
            self.radiation_out_of_bounds_count += 1

        self.metric_last_x = new_x
        self.metric_last_y = new_y

    def report_final_metrics(self):
        if self.finish_reported:
            return

        self.finish_reported = True

        self.get_logger().info('Path following finished.')

        if self.start_time is None:
            execution_time = 0.0
        else:
            now = self.get_clock().now()
            execution_time = (now - self.start_time).nanoseconds / 1e9

        if not self.dose_received:
            self.get_logger().warn(
                'Path following finished, but no dose data was received.'
            )
            path_following_dose = 0.0
        else:
            if self.start_total_dose is None:
                self.start_total_dose = 0.0

            path_following_dose = self.current_total_dose - self.start_total_dose

        executed_final_score = (
            self.dose_weight * path_following_dose +
            self.terrain_weight * self.executed_terrain_cost +
            self.time_weight * execution_time
        )

        self.get_logger().info(
            f'Execution time = {execution_time:.2f} s'
        )

        self.get_logger().info(
            f'Executed path length = {self.executed_path_length:.2f} m'
        )

        self.get_logger().info(
            f'Dose monitor total dose = {self.current_total_dose:.2f}'
        )

        self.get_logger().info(
            f'Dose during path following = {path_following_dose:.2f}'
        )

        self.get_logger().info(
            f'Executed terrain cost = {self.executed_terrain_cost:.2f}'
        )

        self.get_logger().info(
            'Executed radiation-map cost = '
            f'{self.executed_radiation_map_cost:.4f}'
        )

        self.get_logger().info(
            f'Executed final coupled score = {executed_final_score:.2f}'
        )

        self.save_result_to_csv(
            execution_time=execution_time,
            executed_path_length=self.executed_path_length,
            dose_during_path_following=path_following_dose,
            executed_terrain_cost=self.executed_terrain_cost,
            executed_radiation_map_cost=self.executed_radiation_map_cost,
            executed_final_coupled_score=executed_final_score,
            dose_monitor_total_dose=self.current_total_dose
        )

        if self.shutdown_on_finish:
            self.get_logger().info(
                'Shutting down follower after experiment completion.'
            )
            rclpy.shutdown()

    def save_result_to_csv(
        self,
        execution_time,
        executed_path_length,
        dose_during_path_following,
        executed_terrain_cost,
        executed_radiation_map_cost,
        executed_final_coupled_score,
        dose_monitor_total_dose
    ):
        result_dir = os.path.dirname(self.result_csv)

        if result_dir:
            os.makedirs(result_dir, exist_ok=True)

        file_exists = os.path.exists(self.result_csv)

        fieldnames = [
            'timestamp',
            'planner_name',
            'path_topic',
            'execution_time_s',
            'executed_path_length_m',
            'dose_during_path_following',
            'executed_terrain_cost',
            'executed_radiation_map_cost',
            'executed_final_coupled_score',
            'dose_monitor_total_dose',
            'odom_received',
            'path_received',
            'dose_received',
            'terrain_map_received',
            'radiation_map_received',
            'terrain_valid_sample_count',
            'terrain_out_of_bounds_count',
            'radiation_valid_sample_count',
            'radiation_out_of_bounds_count'
        ]

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'planner_name': self.planner_name,
            'path_topic': self.path_topic,
            'execution_time_s': f'{execution_time:.2f}',
            'executed_path_length_m': f'{executed_path_length:.2f}',
            'dose_during_path_following': f'{dose_during_path_following:.6f}',
            'executed_terrain_cost': f'{executed_terrain_cost:.6f}',
            'executed_radiation_map_cost': f'{executed_radiation_map_cost:.6f}',
            'executed_final_coupled_score': f'{executed_final_coupled_score:.6f}',
            'dose_monitor_total_dose': f'{dose_monitor_total_dose:.2f}',
            'odom_received': str(bool(self.has_odom)).lower(),
            'path_received': str(bool(self.path_received)).lower(),
            'dose_received': str(bool(self.dose_received)).lower(),
            'terrain_map_received': str(bool(self.terrain_map_received)).lower(),
            'radiation_map_received': str(bool(self.radiation_map_received)).lower(),
            'terrain_valid_sample_count': str(self.terrain_valid_sample_count),
            'terrain_out_of_bounds_count': str(self.terrain_out_of_bounds_count),
            'radiation_valid_sample_count': str(self.radiation_valid_sample_count),
            'radiation_out_of_bounds_count': str(self.radiation_out_of_bounds_count),
        }

        with open(self.result_csv, mode='a', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if not file_exists:
                writer.writeheader()

            writer.writerow(row)

        self.get_logger().info(
            f'Experiment result saved to: {self.result_csv}'
        )

    def distance_to_point(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y

        return math.sqrt(dx * dx + dy * dy)

    def distance_to_waypoint(self, waypoint):
        target_x, target_y = waypoint
        return self.distance_to_point(target_x, target_y)

    def find_nearest_waypoint_index(self, start_index, end_index):
        if len(self.execution_waypoints) == 0:
            return 0

        start_index = max(0, start_index)
        end_index = min(len(self.execution_waypoints), end_index)

        if start_index >= end_index:
            return min(start_index, len(self.execution_waypoints) - 1)

        nearest_index = start_index
        nearest_distance = self.distance_to_waypoint(
            self.execution_waypoints[start_index]
        )

        for i in range(start_index, end_index):
            distance = self.distance_to_waypoint(self.execution_waypoints[i])

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i

        return nearest_index

    def update_waypoint_progress(self):
        # Mark waypoints as reached only when the robot is physically close.
        while self.current_waypoint_index < len(self.execution_waypoints):
            waypoint = self.execution_waypoints[self.current_waypoint_index]
            distance = self.distance_to_waypoint(waypoint)

            if distance < self.waypoint_tolerance:
                target_x, target_y = waypoint

                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_index}: '
                    f'x={target_x:.2f}, y={target_y:.2f}'
                )

                self.current_waypoint_index += 1
            else:
                break

        if self.current_waypoint_index >= len(self.execution_waypoints):
            return

        # Update progress using a small forward search window.
        # This avoids jumping directly to the final waypoint.
        search_start = self.current_waypoint_index
        search_end = min(
            len(self.execution_waypoints),
            self.current_waypoint_index + self.nearest_search_window
        )

        nearest_index = self.find_nearest_waypoint_index(
            search_start,
            search_end
        )

        if nearest_index > self.current_waypoint_index:
            self.get_logger().info(
                f'Progress updated from waypoint '
                f'{self.current_waypoint_index} to {nearest_index}.'
            )

            self.current_waypoint_index = nearest_index

    def select_lookahead_target(self):
        # Select a lookahead target for smoother tracking.
        # This function must not change current_waypoint_index.
        target_index = self.current_waypoint_index

        while target_index + 1 < len(self.execution_waypoints):
            waypoint = self.execution_waypoints[target_index]
            distance = self.distance_to_waypoint(waypoint)

            if distance >= self.lookahead_distance:
                break

            target_index += 1

        return self.execution_waypoints[target_index], target_index

    def is_goal_reached(self):
        if len(self.execution_waypoints) == 0:
            return False

        goal_x, goal_y = self.execution_waypoints[-1]
        distance_to_goal = self.distance_to_point(goal_x, goal_y)

        return distance_to_goal < self.goal_tolerance

    def publish_debug_info(self, target_x, target_y, target_index, distance, yaw_error):
        now = self.get_clock().now()
        dt = (now - self.last_debug_time).nanoseconds / 1e9

        if dt < 1.0:
            return

        self.last_debug_time = now

        self.get_logger().info(
            f'Robot=({self.current_x:.2f}, {self.current_y:.2f}), '
            f'Target waypoint={target_index}, '
            f'Target=({target_x:.2f}, {target_y:.2f}), '
            f'Distance={distance:.2f}, '
            f'Yaw error={yaw_error:.2f}'
        )

    def control_loop(self):
        if not self.has_odom:
            return

        if not self.path_received:
            return

        # Formal experiments must not begin until all metric inputs are ready.
        # This prevents a valid physical run from being saved with terrain_cost=0
        # merely because the transient map had not reached the follower yet.
        if not self.dose_received:
            return

        if not self.terrain_map_received:
            return

        if not self.radiation_map_received:
            return

        if not self.execution_path_initialized:
            self.initialize_execution_path()
            return

        if self.finished:
            self.stop_robot()
            return

        self.update_waypoint_progress()

        if self.current_waypoint_index >= len(self.execution_waypoints):
            self.finished = True
            self.stop_robot()
            self.report_final_metrics()
            return

        if self.is_goal_reached():
            self.finished = True
            self.stop_robot()
            self.report_final_metrics()
            return

        (target_x, target_y), target_index = self.select_lookahead_target()

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)

        target_yaw = self.normalize_angle(math.atan2(dy, dx) + math.pi)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        # Large heading error protection.
        #
        # Very large heading errors:
        # rotate in place so the robot does not drive in the wrong direction.
        #
        # Medium heading errors:
        # use a slow arc turn because skid-steer rotation in place can stall
        # on high-impedance uneven terrain.
        abs_yaw_error = abs(yaw_error)

        if abs_yaw_error > math.radians(90.0):
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = math.copysign(0.35, yaw_error)

            self.cmd_pub.publish(cmd)
            return

        if abs_yaw_error > math.radians(30.0):
            cmd = Twist()

            # Arc turning is more effective than pure skid-steer rotation
            # on the high-impedance DEM terrain.
            # Negative linear.x corresponds to the Husky visual front.
            cmd.linear.x = -0.07
            cmd.linear.y = 0.0
            cmd.angular.z = math.copysign(0.35, yaw_error)

            self.cmd_pub.publish(cmd)
            return

        angular_speed = self.k_angular * yaw_error
        angular_speed = self.clamp(
            angular_speed,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # Allow slow forward motion even when the heading is not perfect.
        # This helps avoid endless in-place spinning.
        heading_factor = max(0.15, math.cos(yaw_error))

        linear_speed = self.k_linear * distance * heading_factor
        linear_speed = self.clamp(
            linear_speed,
            self.min_linear_speed,
            self.max_linear_speed
        )

        # On rough DEM, pure in-place rotation can make the Husky wheels slip.
        # Use a slow forward arc while correcting a large heading error.
        if abs(yaw_error) > 1.2:
            linear_speed = self.turning_linear_speed
            angular_speed = self.clamp(
                angular_speed,
                -0.20,
                0.20
            )

        cmd = Twist()
        cmd.linear.x = -linear_speed
        cmd.angular.z = angular_speed

        self.cmd_pub.publish(cmd)

        self.executed_path.header.stamp = self.get_clock().now().to_msg()
        self.executed_path_pub.publish(self.executed_path)

        self.publish_debug_info(
            target_x,
            target_y,
            target_index,
            distance,
            yaw_error
        )

    def record_executed_path(self):
        if not self.has_odom:
            return

        if self.last_record_x is None or self.last_record_y is None:
            should_record = True
        else:
            distance = math.sqrt(
                (self.current_x - self.last_record_x) ** 2 +
                (self.current_y - self.last_record_y) ** 2
            )
            should_record = distance > 0.05

        if not should_record:
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y
        pose.pose.position.z = 0.05
        pose.pose.orientation.w = 1.0

        self.executed_path.header.stamp = self.get_clock().now().to_msg()
        self.executed_path.poses.append(pose)

        self.last_record_x = self.current_x
        self.last_record_y = self.current_y


def main(args=None):
    rclpy.init(args=args)

    node = ASDPathWaypointFollower()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.stop_robot()
        node.get_logger().info('ASD path waypoint follower stopped.')

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
