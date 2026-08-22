import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class TerrainCostMapNode(Node):
    def __init__(self):
        super().__init__('terrain_cost_map_node')

        self.width = 100
        self.height = 100
        self.resolution = 0.1
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.publisher = self.create_publisher(
            OccupancyGrid,
            'terrain_cost_map',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_terrain_map)

        self.get_logger().info('Terrain cost map node started.')

    def gaussian(self, x, y, cx, cy, amplitude, sigma):
        dist_sq = (x - cx) ** 2 + (y - cy) ** 2
        return amplitude * math.exp(-dist_sq / (2.0 * sigma ** 2))

    def calculate_terrain_cost(self, x, y):
        # Base flat terrain cost
        cost = 5.0

        # Simulated slope area
        cost += self.gaussian(x, y, -1.5, 1.5, 45.0, 1.2)

        # Simulated rough rubble area
        cost += self.gaussian(x, y, 2.0, -1.5, 60.0, 0.8)

        # Simulated random step / uneven field
        if -0.5 < x < 2.5 and 1.5 < y < 3.5:
            cost += 35.0

        return max(0.0, min(cost, 100.0))

    def publish_terrain_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []

        for j in range(self.height):
            for i in range(self.width):
                x = self.origin_x + i * self.resolution
                y = self.origin_y + j * self.resolution

                terrain_cost = self.calculate_terrain_cost(x, y)
                data.append(int(terrain_cost))

        msg.data = data

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainCostMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
