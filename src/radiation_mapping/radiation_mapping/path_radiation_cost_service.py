import math
import rclpy
from rclpy.node import Node
from radiation_interfaces.srv import QueryPathRadiationCost


class PathRadiationCostService(Node):
    def __init__(self):
        super().__init__('path_radiation_cost_service')

        self.hotspots = [
            (3.0, 3.0, 100.0, 0.8),
            (-2.0, 1.0, 80.0, 1.0),
            (1.0, -4.0, 60.0, 0.6),
        ]

        self.service = self.create_service(
            QueryPathRadiationCost,
            'query_path_radiation_cost',
            self.handle_query
        )

        self.get_logger().info('Path radiation cost service started.')

    def radiation_intensity(self, x, y):
        total = 0.0
        for hx, hy, amplitude, sigma in self.hotspots:
            dist_sq = (x - hx) ** 2 + (y - hy) ** 2
            total += amplitude * math.exp(-dist_sq / (2.0 * sigma ** 2))
        return min(total, 100.0)

    def radiation_to_cost(self, radiation):
        return radiation / 10.0

    def handle_query(self, request, response):
        x_points = list(request.x_points)
        y_points = list(request.y_points)

        if len(x_points) != len(y_points):
            response.total_radiation = 0.0
            response.total_cost = 0.0
            response.success = False
            response.message = 'x_points and y_points must have the same length.'
            return response

        if len(x_points) == 0:
            response.total_radiation = 0.0
            response.total_cost = 0.0
            response.success = False
            response.message = 'Path is empty.'
            return response

        total_radiation = 0.0
        total_cost = 0.0

        for i in range(len(x_points)):
            x = x_points[i]
            y = y_points[i]

            radiation = self.radiation_intensity(x, y)
            cost = self.radiation_to_cost(radiation)

            if i == 0:
                segment_distance = 0.0
            else:
                dx = x_points[i] - x_points[i - 1]
                dy = y_points[i] - y_points[i - 1]
                segment_distance = math.sqrt(dx * dx + dy * dy)

            total_radiation += radiation * segment_distance
            total_cost += cost * segment_distance

        response.total_radiation = total_radiation
        response.total_cost = total_cost
        response.success = True
        response.message = (
            f'Path contains {len(x_points)} points. '
            f'Distance-weighted total radiation = {total_radiation:.2f}, '
            f'distance-weighted total cost = {total_cost:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathRadiationCostService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
