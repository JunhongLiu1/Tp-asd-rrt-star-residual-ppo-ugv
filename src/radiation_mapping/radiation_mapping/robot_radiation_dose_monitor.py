import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


class RobotRadiationDoseMonitor(Node):
    def __init__(self):
        super().__init__('robot_radiation_dose_monitor')

        self.declare_parameter('radiation_topic', '/radiation_map')

        self.declare_parameter('odom_to_map_x', 0.0)
        self.declare_parameter('odom_to_map_y', 0.0)
        self.declare_parameter('odom_to_map_yaw', 0.0)

        # Radiation map value 100 corresponds to 8 µSv/h.
        self.declare_parameter('max_radiation_rate_usv_h', 8.0)

        self.radiation_topic = (
            self.get_parameter('radiation_topic')
            .get_parameter_value()
            .string_value
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

        self.max_radiation_rate_usv_h = (
            self.get_parameter('max_radiation_rate_usv_h')
            .get_parameter_value()
            .double_value
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.has_odom = False

        self.radiation_map = None
        self.total_dose_usv = 0.0
        self.last_time = None
        self.last_log_time = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.radiation_map_sub = self.create_subscription(
            OccupancyGrid,
            self.radiation_topic,
            self.radiation_map_callback,
            map_qos
        )

        self.radiation_rate_pub = self.create_publisher(
            Float64,
            '/robot_radiation_rate',
            10
        )

        self.accumulated_dose_pub = self.create_publisher(
            Float64,
            '/robot_accumulated_dose',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/robot_radiation_marker',
            10
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            'Map-based robot radiation dose monitor started.'
        )
        self.get_logger().info(
            f'Radiation topic: {self.radiation_topic}'
        )
        self.get_logger().info(
            f'Max radiation rate: '
            f'{self.max_radiation_rate_usv_h:.2f} µSv/h'
        )

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        cos_yaw = math.cos(self.odom_to_map_yaw)
        sin_yaw = math.sin(self.odom_to_map_yaw)

        self.current_x = (
            self.odom_to_map_x
            + cos_yaw * odom_x
            - sin_yaw * odom_y
        )

        self.current_y = (
            self.odom_to_map_y
            + sin_yaw * odom_x
            + cos_yaw * odom_y
        )

        self.has_odom = True

    def radiation_map_callback(self, msg):
        self.radiation_map = msg

    def world_to_map_index(self, map_msg, x, y):
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        if resolution <= 0.0 or width <= 0 or height <= 0:
            return None

        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if (
            map_x < 0
            or map_x >= width
            or map_y < 0
            or map_y >= height
        ):
            return None

        return map_y * width + map_x

    def get_radiation_rate(self, x, y):
        if self.radiation_map is None:
            return None

        index = self.world_to_map_index(
            self.radiation_map,
            x,
            y
        )

        if index is None:
            return None

        map_value = self.radiation_map.data[index]

        if map_value < 0:
            return None

        normalized_value = max(
            0.0,
            min(float(map_value), 100.0)
        )

        return (
            normalized_value / 100.0
        ) * self.max_radiation_rate_usv_h

    def timer_callback(self):
        if not self.has_odom or self.radiation_map is None:
            return

        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            self.last_log_time = now
            return

        dt_seconds = (
            now - self.last_time
        ).nanoseconds / 1e9

        self.last_time = now

        if dt_seconds <= 0.0:
            return

        radiation_rate_usv_h = self.get_radiation_rate(
            self.current_x,
            self.current_y
        )

        if radiation_rate_usv_h is None:
            radiation_rate_usv_h = 0.0

        # Convert µSv/h into µSv accumulated during dt seconds.
        dose_increment_usv = (
            radiation_rate_usv_h
            * dt_seconds
            / 3600.0
        )

        self.total_dose_usv += dose_increment_usv

        rate_msg = Float64()
        rate_msg.data = radiation_rate_usv_h
        self.radiation_rate_pub.publish(rate_msg)

        dose_msg = Float64()
        dose_msg.data = self.total_dose_usv
        self.accumulated_dose_pub.publish(dose_msg)

        self.publish_marker(
            radiation_rate_usv_h
        )

        log_dt = (
            now - self.last_log_time
        ).nanoseconds / 1e9

        if log_dt >= 1.0:
            self.last_log_time = now

            self.get_logger().info(
                f'Robot=({self.current_x:.2f}, '
                f'{self.current_y:.2f}), '
                f'radiation_rate='
                f'{radiation_rate_usv_h:.4f} µSv/h, '
                f'total_dose='
                f'{self.total_dose_usv:.8f} µSv'
            )

    def publish_marker(self, radiation_rate_usv_h):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'robot_radiation_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.current_x
        marker.pose.position.y = self.current_y
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = (
            f'Rate: {radiation_rate_usv_h:.4f} µSv/h\n'
            f'Dose: {self.total_dose_usv:.8f} µSv'
        )

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = RobotRadiationDoseMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
