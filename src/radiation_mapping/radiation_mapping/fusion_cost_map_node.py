import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FusionCostMapNode(Node):
    def __init__(self):
        super().__init__('fusion_cost_map_node')

        self.radiation_map = None
        self.terrain_map = None

        self.radiation_weight = 0.5
        self.terrain_weight = 0.5

        self.radiation_sub = self.create_subscription(
            OccupancyGrid,
            'radiation_map',
            self.radiation_callback,
            10
        )

        self.terrain_sub = self.create_subscription(
            OccupancyGrid,
            'terrain_cost_map',
            self.terrain_callback,
            10
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            'fusion_cost_map',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_fusion_map)

        self.get_logger().info('Fusion cost map node started.')

    def radiation_callback(self, msg):
        self.radiation_map = msg

    def terrain_callback(self, msg):
        self.terrain_map = msg

    def publish_fusion_map(self):
        if self.radiation_map is None or self.terrain_map is None:
            self.get_logger().info(
                'Waiting for radiation_map and terrain_cost_map...'
            )
            return

        if len(self.radiation_map.data) != len(self.terrain_map.data):
            self.get_logger().warn('Map sizes do not match.')
            return

        fusion_msg = OccupancyGrid()

        fusion_msg.header.stamp = self.get_clock().now().to_msg()
        fusion_msg.header.frame_id = 'map'

        fusion_msg.info = self.radiation_map.info

        fusion_data = []

        for radiation_value, terrain_value in zip(
            self.radiation_map.data,
            self.terrain_map.data
        ):
            radiation_cost = max(0, radiation_value)
            terrain_cost = max(0, terrain_value)

            fusion_cost = (
                self.radiation_weight * radiation_cost +
                self.terrain_weight * terrain_cost
            )

            fusion_cost = max(0.0, min(fusion_cost, 100.0))
            fusion_data.append(int(fusion_cost))

        fusion_msg.data = fusion_data

        self.publisher.publish(fusion_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionCostMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
