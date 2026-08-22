#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Week45RvizDemo(Node):
    def __init__(self):
        super().__init__('week4_5_rviz_demo')

        self.min_x = -5.0
        self.min_y = -5.0
        self.max_x = 5.0
        self.max_y = 5.0
        self.resolution = 0.1

        self.width = int((self.max_x - self.min_x) / self.resolution)
        self.height = int((self.max_y - self.min_y) / self.resolution)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        path_qos = QoSProfile(depth=10)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.radiation_pub = self.create_publisher(
            OccupancyGrid,
            '/radiation_map',
            map_qos
        )

        self.terrain_pub = self.create_publisher(
            OccupancyGrid,
            '/terrain_cost_map',
            map_qos
        )

        self.fusion_pub = self.create_publisher(
            OccupancyGrid,
            '/fusion_cost_map',
            map_qos
        )

        self.rrt_path_pub = self.create_publisher(
            Path,
            '/rrt_star/path',
            path_qos
        )

        self.asd_path_pub = self.create_publisher(
            Path,
            '/asd_rrt_star/path',
            path_qos
        )

        self.smooth_path_pub = self.create_publisher(
            Path,
            '/asd_rrt_star_teb/path',
            path_qos
        )

        self.timer = self.create_timer(1.0, self.publish_all)

        self.get_logger().info('Week 4-5 RViz demo node started.')

    def radiation_value(self, x, y):
        hotspots = [
            (3.0, 3.0, 100.0, 0.8),
            (-2.0, 1.0, 80.0, 1.0),
            (1.0, -4.0, 60.0, 0.6),
        ]

        value = 0.0

        for hotspot_x, hotspot_y, amplitude, sigma in hotspots:
            dx = x - hotspot_x
            dy = y - hotspot_y
            distance_squared = dx * dx + dy * dy

            value += amplitude * math.exp(
                -distance_squared / (2.0 * sigma * sigma)
            )

        return min(100.0, value)

    def terrain_cost(self, x, y):
        base_cost = 5.0

        slope_region = 35.0 * math.exp(
            -((x + 1.5) ** 2 + (y - 1.5) ** 2) / (2.0 * 0.9 * 0.9)
        )

        rough_region = 30.0 * math.exp(
            -((x - 2.0) ** 2 + (y + 1.5) ** 2) / (2.0 * 0.8 * 0.8)
        )

        step_region = 0.0
        if -0.5 < x < 2.5 and 1.5 < y < 3.5:
            step_region = 35.0

        return min(100.0, base_cost + slope_region + rough_region + step_region)

    def fusion_cost(self, x, y):
        radiation = self.radiation_value(x, y)
        terrain = self.terrain_cost(x, y)

        return min(100.0, 0.5 * radiation + 0.5 * terrain)

    def create_grid(self, value_function):
        grid = OccupancyGrid()

        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height

        grid.info.origin.position.x = self.min_x
        grid.info.origin.position.y = self.min_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        data = []

        for row in range(self.height):
            for col in range(self.width):
                x = self.min_x + col * self.resolution
                y = self.min_y + row * self.resolution

                value = int(value_function(x, y))
                value = max(0, min(100, value))
                data.append(value)

        grid.data = data

        return grid

    def create_path(self, points):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def publish_all(self):
        radiation_map = self.create_grid(self.radiation_value)
        terrain_map = self.create_grid(self.terrain_cost)
        fusion_map = self.create_grid(self.fusion_cost)

        rrt_points = [
            (-4.0, -4.0),
            (-3.2, -2.6),
            (-2.2, -0.2),
            (-1.4, 1.0),
            (-0.2, 1.8),
            (1.3, 2.2),
            (2.8, 3.0),
            (4.0, 4.0),
        ]

        asd_points = [
            (-4.0, -4.0),
            (-3.4, -3.3),
            (-2.2, -2.5),
            (-0.8, -2.0),
            (0.8, -1.4),
            (2.2, -0.4),
            (3.4, 1.2),
            (4.0, 4.0),
        ]

        smooth_points = [
            (-4.0, -4.0),
            (-3.2, -3.2),
            (-2.0, -2.4),
            (-0.6, -1.8),
            (0.9, -1.2),
            (2.1, -0.2),
            (3.2, 1.4),
            (3.8, 2.8),
            (4.0, 4.0),
        ]

        self.radiation_pub.publish(radiation_map)
        self.terrain_pub.publish(terrain_map)
        self.fusion_pub.publish(fusion_map)

        self.rrt_path_pub.publish(self.create_path(rrt_points))
        self.asd_path_pub.publish(self.create_path(asd_points))
        self.smooth_path_pub.publish(self.create_path(smooth_points))


def main(args=None):
    rclpy.init(args=args)

    node = Week45RvizDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
