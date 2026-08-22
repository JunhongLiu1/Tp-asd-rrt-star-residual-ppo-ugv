import math
import rclpy
from rclpy.node import Node
from radiation_interfaces.srv import QueryRadiation


class RadiationQueryService(Node):
    def __init__(self):
        super().__init__('radiation_query_service')

        self.hotspots = [
            (3.0, 3.0, 100.0, 0.8),
            (-2.0, 1.0, 80.0, 1.0),
            (1.0, -4.0, 60.0, 0.6),
        ]

        self.service = self.create_service(
            QueryRadiation,
            'query_radiation',
            self.handle_query
        )

        self.get_logger().info('Radiation query service started.')

    def radiation_intensity(self, x, y):
        total = 0.0
        for hx, hy, amplitude, sigma in self.hotspots:
            dist_sq = (x - hx) ** 2 + (y - hy) ** 2
            total += amplitude * math.exp(-dist_sq / (2.0 * sigma ** 2))
        return min(total, 100.0)

    def handle_query(self, request, response):
        x = request.x
        y = request.y
        value = self.radiation_intensity(x, y)

        response.radiation = value
        response.success = True
        response.message = f'Radiation at ({x:.2f}, {y:.2f}) = {value:.2f}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RadiationQueryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
