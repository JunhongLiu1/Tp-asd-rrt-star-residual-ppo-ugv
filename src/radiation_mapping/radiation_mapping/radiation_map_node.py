import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class RadiationMapNode(Node):
    def __init__(self):
        super().__init__('radiation_map_node')
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(OccupancyGrid, '/radiation_map', map_qos)

        self.width = 100
        self.height = 100
        self.resolution = 0.1
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.hotspots = [
            (3.0, 3.0, 100.0, 0.8),
            (-2.0, 1.0, 80.0, 1.0),
            (1.0, -4.0, 60.0, 0.6),
        ]

        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Radiation map node started.')

    def radiation_intensity(self, x, y):
        total = 0.0
        for hx, hy, amplitude, sigma in self.hotspots:
            dist_sq = (x - hx) ** 2 + (y - hy) ** 2
            total += amplitude * math.exp(-dist_sq / (2.0 * sigma ** 2))
        return min(total, 100.0)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        data = []
        for j in range(self.height):
            for i in range(self.width):
                x = self.origin_x + i * self.resolution
                y = self.origin_y + j * self.resolution
                value = int(self.radiation_intensity(x, y))
                data.append(value)

        msg.data = data
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RadiationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
