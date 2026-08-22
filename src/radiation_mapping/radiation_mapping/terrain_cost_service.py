import rclpy
from rclpy.node import Node
from radiation_interfaces.srv import QueryTerrainCost


class TerrainCostService(Node):
    def __init__(self):
        super().__init__('terrain_cost_service')

        self.service = self.create_service(
            QueryTerrainCost,
            'query_terrain_cost',
            self.handle_query
        )

        self.get_logger().info('Terrain cost service started.')

    def calculate_terrain_cost(self, slope, roughness):
        slope_weight = 0.6
        roughness_weight = 0.4

        cost = slope_weight * slope + roughness_weight * roughness
        return max(0.0, min(cost, 10.0))

    def handle_query(self, request, response):
        slope = request.slope
        roughness = request.roughness

        cost = self.calculate_terrain_cost(slope, roughness)

        response.terrain_cost = cost
        response.success = True
        response.message = (
            f'Slope = {slope:.2f}, roughness = {roughness:.2f}, '
            f'terrain cost = {cost:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TerrainCostService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
