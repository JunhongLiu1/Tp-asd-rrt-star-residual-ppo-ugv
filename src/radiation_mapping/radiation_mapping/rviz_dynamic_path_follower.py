import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Float64


class RVizDynamicPathFollower(Node):
    def __init__(self):
        super().__init__('rviz_dynamic_path_follower')

        self.declare_parameter(
            'planner_name',
            'RViz ASD-Time-Aware RRT*'
        )

        self.declare_parameter(
            'path_topic',
            '/rviz_asd_time_aware_rrt_star_path'
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
            'result_csv',
            '~/terrain_radiation_ws/experiment_results/rviz_navigation_results.csv'
        )

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

        self.result_csv = os.path.expanduser(
            self.get_parameter('result_csv')
            .get_parameter_value()
            .string_value
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.current_total_dose = 0.0
        self.dose_received = False

        self.terrain_map = None
        self.terrain_map_received = False

        self.waypoints = []
        self.current_waypoint_index = 0

        self.active_navigation = False
        self.navigation_finished = True

        self.navigation_id = 0
        self.last_goal_x = None
        self.last_goal_y = None
        self.goal_change_threshold = 0.25

        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.start_time = None
        self.start_total_dose = 0.0

        self.executed_path_length = 0.0
        self.executed_terrain_cost = 0.0

        self.metric_last_x = None
        self.metric_last_y = None

        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'

        self.last_record_x = None
        self.last_record_y = None

        # Tracking parameters
        self.waypoint_tolerance = 0.18
        self.goal_tolerance = 0.25
        self.lookahead_distance = 0.35

        self.max_linear_speed = 0.12
        self.max_angular_speed = 0.80
        self.min_linear_speed = 0.02

        self.k_linear = 0.35
        self.k_angular = 1.60

        # Execution score weights
        self.dose_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        self.last_debug_time = self.get_clock().now()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
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
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.executed_path_pub = self.create_publisher(
            Path,
            '/rviz_executed_path',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RViz dynamic path follower started.')
        self.get_logger().info(f'Planner name: {self.planner_name}')
        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Result CSV: {self.result_csv}')
        self.get_logger().info('Waiting for RViz planned paths...')

    def odom_callback(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        new_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.active_navigation:
            self.update_execution_metrics(new_x, new_y)

        self.current_x = new_x
        self.current_y = new_y
        self.current_yaw = new_yaw
        self.has_odom = True

        if self.active_navigation:
            self.record_executed_path()

    def dose_callback(self, msg):
        self.current_total_dose = msg.data
        self.dose_received = True

    def terrain_callback(self, msg):
        self.terrain_map = msg
        self.terrain_map_received = True

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return

        new_goal_x = msg.poses[-1].pose.position.x
        new_goal_y = msg.poses[-1].pose.position.y

        if self.last_goal_x is not None and self.last_goal_y is not None:
            goal_change = math.sqrt(
                (new_goal_x - self.last_goal_x) ** 2 +
                (new_goal_y - self.last_goal_y) ** 2
            )

            if goal_change < self.goal_change_threshold:
                return

        if not self.has_odom:
            self.get_logger().warn(
                'Path received, but odometry is not ready yet.'
            )
            return

        self.accept_new_path(msg)

    def accept_new_path(self, msg):
        self.stop_robot()

        self.navigation_id += 1

        self.waypoints = []

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.waypoints.append((x, y))

        if len(self.waypoints) == 0:
            self.get_logger().warn('Received an empty path.')
            return

        self.start_x = self.current_x
        self.start_y = self.current_y

        self.goal_x = self.waypoints[-1][0]
        self.goal_y = self.waypoints[-1][1]

        self.last_goal_x = self.goal_x
        self.last_goal_y = self.goal_y

        self.current_waypoint_index = self.find_nearest_waypoint_index(
            start_index=0,
            end_index=len(self.waypoints)
        )

        self.active_navigation = True
        self.navigation_finished = False

        self.start_time = self.get_clock().now()

        if self.dose_received:
            self.start_total_dose = self.current_total_dose
        else:
            self.start_total_dose = 0.0

        self.executed_path_length = 0.0
        self.executed_terrain_cost = 0.0

        self.metric_last_x = self.current_x
        self.metric_last_y = self.current_y

        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'
        self.last_record_x = None
        self.last_record_y = None

        self.get_logger().info(
            f'Accepted new RViz navigation path #{self.navigation_id}.'
        )
        self.get_logger().info(
            f'Start: ({self.start_x:.2f}, {self.start_y:.2f}), '
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}), '
            f'Waypoints: {len(self.waypoints)}'
        )
        self.get_logger().info(
            f'Start following from waypoint index {self.current_waypoint_index}.'
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
            segment_terrain_cost = (terrain_value / 10.0) * segment_distance
            self.executed_terrain_cost += segment_terrain_cost

        self.metric_last_x = new_x
        self.metric_last_y = new_y

    def distance_to_point(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y

        return math.sqrt(dx * dx + dy * dy)

    def distance_to_waypoint(self, waypoint):
        target_x, target_y = waypoint
        return self.distance_to_point(target_x, target_y)

    def find_nearest_waypoint_index(self, start_index, end_index):
        if len(self.waypoints) == 0:
            return 0

        start_index = max(0, start_index)
        end_index = min(len(self.waypoints), end_index)

        if start_index >= end_index:
            return min(start_index, len(self.waypoints) - 1)

        nearest_index = start_index
        nearest_distance = self.distance_to_waypoint(
            self.waypoints[start_index]
        )

        for i in range(start_index, end_index):
            distance = self.distance_to_waypoint(self.waypoints[i])

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i

        return nearest_index

    def update_waypoint_progress(self):
        while self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
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

    def select_lookahead_target(self):
        target_index = self.current_waypoint_index

        while target_index + 1 < len(self.waypoints):
            waypoint = self.waypoints[target_index]
            distance = self.distance_to_waypoint(waypoint)

            if distance >= self.lookahead_distance:
                break

            target_index += 1

        return self.waypoints[target_index], target_index

    def is_goal_reached(self):
        if len(self.waypoints) == 0:
            return False

        distance_to_goal = self.distance_to_point(
            self.goal_x,
            self.goal_y
        )

        return distance_to_goal < self.goal_tolerance

    def report_navigation_result(self):
        if self.start_time is None:
            execution_time = 0.0
        else:
            now = self.get_clock().now()
            execution_time = (now - self.start_time).nanoseconds / 1e9

        if self.dose_received:
            dose_during_navigation = self.current_total_dose - self.start_total_dose
        else:
            dose_during_navigation = 0.0

        final_score = (
            self.dose_weight * dose_during_navigation +
            self.terrain_weight * self.executed_terrain_cost +
            self.time_weight * execution_time
        )

        self.get_logger().info(
            f'RViz navigation #{self.navigation_id} finished.'
        )
        self.get_logger().info(
            f'Execution time = {execution_time:.2f} s'
        )
        self.get_logger().info(
            f'Executed path length = {self.executed_path_length:.2f} m'
        )
        self.get_logger().info(
            f'Dose during navigation = {dose_during_navigation:.2f}'
        )
        self.get_logger().info(
            f'Executed terrain cost = {self.executed_terrain_cost:.2f}'
        )
        self.get_logger().info(
            f'Executed final coupled score = {final_score:.2f}'
        )

        self.save_result_to_csv(
            execution_time=execution_time,
            executed_path_length=self.executed_path_length,
            dose_during_navigation=dose_during_navigation,
            executed_terrain_cost=self.executed_terrain_cost,
            final_score=final_score
        )

    def save_result_to_csv(
        self,
        execution_time,
        executed_path_length,
        dose_during_navigation,
        executed_terrain_cost,
        final_score
    ):
        result_dir = os.path.dirname(self.result_csv)

        if result_dir:
            os.makedirs(result_dir, exist_ok=True)

        file_exists = os.path.exists(self.result_csv)

        fieldnames = [
            'timestamp',
            'navigation_id',
            'planner_name',
            'path_topic',
            'start_x',
            'start_y',
            'goal_x',
            'goal_y',
            'execution_time_s',
            'executed_path_length_m',
            'dose_during_navigation',
            'executed_terrain_cost',
            'executed_final_coupled_score'
        ]

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'navigation_id': self.navigation_id,
            'planner_name': self.planner_name,
            'path_topic': self.path_topic,
            'start_x': f'{self.start_x:.2f}',
            'start_y': f'{self.start_y:.2f}',
            'goal_x': f'{self.goal_x:.2f}',
            'goal_y': f'{self.goal_y:.2f}',
            'execution_time_s': f'{execution_time:.2f}',
            'executed_path_length_m': f'{executed_path_length:.2f}',
            'dose_during_navigation': f'{dose_during_navigation:.2f}',
            'executed_terrain_cost': f'{executed_terrain_cost:.2f}',
            'executed_final_coupled_score': f'{final_score:.2f}',
        }

        with open(self.result_csv, mode='a', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if not file_exists:
                writer.writeheader()

            writer.writerow(row)

        self.get_logger().info(
            f'RViz navigation result saved to: {self.result_csv}'
        )

    def finish_navigation(self):
        self.stop_robot()
        self.active_navigation = False
        self.navigation_finished = True
        self.report_navigation_result()

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

        if not self.active_navigation:
            self.stop_robot()
            return

        if len(self.waypoints) == 0:
            self.stop_robot()
            return

        self.update_waypoint_progress()

        if self.current_waypoint_index >= len(self.waypoints):
            self.finish_navigation()
            return

        if self.is_goal_reached():
            self.finish_navigation()
            return

        (target_x, target_y), target_index = self.select_lookahead_target()

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        angular_speed = self.k_angular * yaw_error
        angular_speed = self.clamp(
            angular_speed,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        heading_factor = max(0.10, math.cos(yaw_error))

        linear_speed = self.k_linear * distance * heading_factor
        linear_speed = self.clamp(
            linear_speed,
            self.min_linear_speed,
            self.max_linear_speed
        )

        if abs(yaw_error) > 2.2:
            linear_speed = 0.0
            angular_speed = self.clamp(
                angular_speed,
                -0.45,
                0.45
            )

        cmd = Twist()
        cmd.linear.x = linear_speed
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
    node = RVizDynamicPathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('RViz dynamic path follower stopped.')

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
