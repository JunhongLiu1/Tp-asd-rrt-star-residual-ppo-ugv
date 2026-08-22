#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Empty


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GroundTruthExecutionPathRecorder(Node):
    """Record Husky ground-truth odometry and publish the completed executed path."""

    def __init__(self) -> None:
        super().__init__('ground_truth_execution_path_recorder')

        self.declare_parameter('odom_topic', '/ground_truth/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('final_path_topic', '/husky_executed_path')
        self.declare_parameter('live_path_topic', '/husky_executed_path_live')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('linear_motion_threshold', 0.01)
        self.declare_parameter('angular_motion_threshold', 0.02)
        self.declare_parameter('cmd_timeout_sec', 0.75)
        self.declare_parameter('stop_stationary_sec', 5.0)

        self.declare_parameter('minimum_sample_distance_m', 0.03)
        self.declare_parameter('minimum_sample_yaw_deg', 2.0)
        self.declare_parameter('maximum_sample_interval_sec', 0.50)
        self.declare_parameter('live_publish_rate_hz', 2.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.final_path_topic = str(
            self.get_parameter('final_path_topic').value
        )
        self.live_path_topic = str(
            self.get_parameter('live_path_topic').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.linear_threshold = float(
            self.get_parameter('linear_motion_threshold').value
        )
        self.angular_threshold = float(
            self.get_parameter('angular_motion_threshold').value
        )
        self.cmd_timeout_sec = float(
            self.get_parameter('cmd_timeout_sec').value
        )
        self.stop_stationary_sec = float(
            self.get_parameter('stop_stationary_sec').value
        )

        self.minimum_sample_distance = float(
            self.get_parameter('minimum_sample_distance_m').value
        )
        self.minimum_sample_yaw = math.radians(
            float(self.get_parameter('minimum_sample_yaw_deg').value)
        )
        self.maximum_sample_interval = float(
            self.get_parameter('maximum_sample_interval_sec').value
        )
        publish_rate = max(
            0.1,
            float(self.get_parameter('live_publish_rate_hz').value),
        )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = QoSReliabilityPolicy.RELIABLE
        transient_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.final_path_pub = self.create_publisher(
            Path,
            self.final_path_topic,
            transient_qos,
        )
        self.live_path_pub = self.create_publisher(
            Path,
            self.live_path_topic,
            10,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            50,
        )
        self.create_subscription(
            Empty,
            '/finish_husky_execution_path',
            self.finish_callback,
            10,
        )
        self.create_subscription(
            Empty,
            '/reset_husky_execution_path',
            self.reset_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd = Twist()
        self.latest_cmd_time_sec: Optional[float] = None
        self.last_motion_time_sec: Optional[float] = None

        self.recording = False
        self.finalized = False
        self.poses: List[PoseStamped] = []
        self.last_sample_time_sec: Optional[float] = None
        self.last_sample_yaw: Optional[float] = None

        self.get_logger().info(
            'Ground-truth execution recorder ready. '
            f'odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'final_path={self.final_path_topic}'
        )
        self.get_logger().info(
            'Recording starts automatically on the first non-zero command. '
            f'It finalizes after {self.stop_stationary_sec:.1f} s stationary.'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def command_is_nonzero(self, msg: Twist) -> bool:
        linear = math.sqrt(
            msg.linear.x * msg.linear.x
            + msg.linear.y * msg.linear.y
            + msg.linear.z * msg.linear.z
        )
        angular = math.sqrt(
            msg.angular.x * msg.angular.x
            + msg.angular.y * msg.angular.y
            + msg.angular.z * msg.angular.z
        )
        return (
            linear >= self.linear_threshold
            or angular >= self.angular_threshold
        )

    def currently_moving(self, now_sec: float) -> bool:
        if self.latest_cmd_time_sec is None:
            return False

        if now_sec - self.latest_cmd_time_sec > self.cmd_timeout_sec:
            return False

        return self.command_is_nonzero(self.latest_cmd)

    def cmd_vel_callback(self, msg: Twist) -> None:
        now_sec = self.now_sec()
        self.latest_cmd = msg
        self.latest_cmd_time_sec = now_sec

        if self.command_is_nonzero(msg):
            self.last_motion_time_sec = now_sec

            if not self.recording:
                if self.finalized or self.poses:
                    self.clear_recording()

                self.recording = True
                self.finalized = False
                self.get_logger().info(
                    'Motion detected: started recording executed path.'
                )

                if self.latest_odom is not None:
                    self.append_pose(
                        self.latest_odom,
                        force=True,
                    )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

        if not self.recording:
            return

        self.append_pose(msg, force=False)

    def append_pose(self, msg: Odometry, force: bool) -> None:
        now_sec = self.now_sec()

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        should_append = force or not self.poses

        if self.poses and not should_append:
            previous = self.poses[-1]
            dx = x - previous.pose.position.x
            dy = y - previous.pose.position.y
            distance = math.hypot(dx, dy)

            yaw_change = abs(
                normalize_angle(yaw - float(self.last_sample_yaw))
            )
            elapsed = (
                now_sec - float(self.last_sample_time_sec)
                if self.last_sample_time_sec is not None
                else float('inf')
            )

            should_append = (
                distance >= self.minimum_sample_distance
                or yaw_change >= self.minimum_sample_yaw
                or elapsed >= self.maximum_sample_interval
            )

        if not should_append:
            return

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        self.poses.append(pose)
        self.last_sample_time_sec = now_sec
        self.last_sample_yaw = yaw

    def build_path(self) -> Path:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.poses = list(self.poses)
        return message

    def publish_live_path(self) -> None:
        if not self.poses:
            return
        self.live_path_pub.publish(self.build_path())

    def finalize_recording(self, reason: str) -> None:
        if not self.recording and self.finalized:
            return

        if self.latest_odom is not None:
            self.append_pose(self.latest_odom, force=True)

        self.recording = False

        if len(self.poses) < 2:
            self.get_logger().warn(
                'Execution recording contains fewer than two poses; '
                'final path was not published.'
            )
            return

        path = self.build_path()
        self.final_path_pub.publish(path)
        self.live_path_pub.publish(path)
        self.finalized = True

        length = 0.0
        for first, second in zip(self.poses[:-1], self.poses[1:]):
            length += math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )

        start = self.poses[0].pose.position
        finish = self.poses[-1].pose.position

        self.get_logger().info(
            f'Executed path finalized ({reason}): '
            f'poses={len(self.poses)}, length={length:.3f} m, '
            f'start=({start.x:.3f},{start.y:.3f}), '
            f'finish=({finish.x:.3f},{finish.y:.3f})'
        )
        self.get_logger().info(
            f'Published completed execution path on {self.final_path_topic}.'
        )

    def clear_recording(self) -> None:
        self.recording = False
        self.finalized = False
        self.poses.clear()
        self.last_sample_time_sec = None
        self.last_sample_yaw = None
        self.last_motion_time_sec = None

    def finish_callback(self, _msg: Empty) -> None:
        self.finalize_recording('manual finish command')

    def reset_callback(self, _msg: Empty) -> None:
        self.clear_recording()
        self.get_logger().info('Execution path recording reset.')

    def timer_callback(self) -> None:
        now_sec = self.now_sec()

        if self.recording:
            self.publish_live_path()

            if self.currently_moving(now_sec):
                self.last_motion_time_sec = now_sec
                return

            if self.last_motion_time_sec is None:
                return

            stationary_duration = now_sec - self.last_motion_time_sec

            if stationary_duration >= self.stop_stationary_sec:
                self.finalize_recording(
                    f'stationary for {stationary_duration:.1f} s'
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthExecutionPathRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.recording:
            node.finalize_recording('node shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
