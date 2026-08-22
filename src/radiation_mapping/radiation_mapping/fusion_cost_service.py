import rclpy
from rclpy.node import Node
from radiation_interfaces.srv import QueryFusionCost


class FusionCostService(Node):
    def __init__(self):
        super().__init__('fusion_cost_service')

        self.radiation_weight = 0.5
        self.terrain_weight = 0.5

        self.service = self.create_service(
            QueryFusionCost,
            'query_fusion_cost',
            self.handle_query
        )

        self.get_logger().info('Fusion cost service started.')

    def calculate_fusion_cost(self, radiation_cost, terrain_cost):
        fusion_cost = (
            self.radiation_weight * radiation_cost +
            self.terrain_weight * terrain_cost
        )

        return max(0.0, min(fusion_cost, 10.0))

    def handle_query(self, request, response):
        radiation_cost = request.radiation_cost
        terrain_cost = request.terrain_cost

        fusion_cost = self.calculate_fusion_cost(
            radiation_cost,
            terrain_cost
        )

        response.fusion_cost = fusion_cost
        response.success = True
        response.message = (
            f'Radiation cost = {radiation_cost:.2f}, '
            f'terrain cost = {terrain_cost:.2f}, '
            f'fusion cost = {fusion_cost:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FusionCostService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
