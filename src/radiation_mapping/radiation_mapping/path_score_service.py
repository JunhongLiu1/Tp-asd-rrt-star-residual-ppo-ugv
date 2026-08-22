import rclpy
from rclpy.node import Node
from radiation_interfaces.srv import QueryPathScore


class PathScoreService(Node):
    def __init__(self):
        super().__init__('path_score_service')

        self.length_weight = 0.4
        self.radiation_weight = 0.4
        self.terrain_weight = 0.2

        self.service = self.create_service(
            QueryPathScore,
            'query_path_score',
            self.handle_query
        )

        self.get_logger().info('Path score service started.')

    def calculate_score(self, path_length, path_radiation_cost, terrain_cost):
        return (
            self.length_weight * path_length +
            self.radiation_weight * path_radiation_cost +
            self.terrain_weight * terrain_cost
        )

    def handle_query(self, request, response):
        path_length = request.path_length
        path_radiation_cost = request.path_radiation_cost
        terrain_cost = request.terrain_cost

        final_score = self.calculate_score(
            path_length,
            path_radiation_cost,
            terrain_cost
        )

        response.final_score = final_score
        response.success = True
        response.message = (
            f'Path length = {path_length:.2f}, '
            f'path radiation cost = {path_radiation_cost:.2f}, '
            f'terrain cost = {terrain_cost:.2f}, '
            f'final score = {final_score:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathScoreService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
