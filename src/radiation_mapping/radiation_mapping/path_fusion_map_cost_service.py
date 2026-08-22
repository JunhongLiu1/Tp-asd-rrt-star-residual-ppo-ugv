import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from radiation_interfaces.srv import QueryPathFusionMapCost


class PathFusionMapCostService(Node):
    def __init__(self):
        super().__init__('path_fusion_map_cost_service')

        self.fusion_map = None

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'fusion_cost_map',
            self.fusion_map_callback,
            10
        )

        self.service = self.create_service(
            QueryPathFusionMapCost,
            'query_path_fusion_map_cost',
            self.handle_query
        )

        self.get_logger().info('Path fusion map cost service started.')

    def fusion_map_callback(self, msg):
        self.fusion_map = msg

    def world_to_map_index(self, x, y):
        origin_x = self.fusion_map.info.origin.position.x
        origin_y = self.fusion_map.info.origin.position.y
        resolution = self.fusion_map.info.resolution
        width = self.fusion_map.info.width
        height = self.fusion_map.info.height

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return None

        return map_y * width + map_x

    def handle_query(self, request, response):
        if self.fusion_map is None:
            response.total_fusion_cost = 0.0
            response.success = False
            response.message = 'Fusion cost map has not been received yet.'
            return response

        x_points = list(request.x_points)
        y_points = list(request.y_points)

        if len(x_points) != len(y_points):
            response.total_fusion_cost = 0.0
            response.success = False
            response.message = 'x_points and y_points must have the same length.'
            return response

        if len(x_points) == 0:
            response.total_fusion_cost = 0.0
            response.success = False
            response.message = 'Path is empty.'
            return response

        total_cost = 0.0

        for i in range(len(x_points)):
            x = x_points[i]
            y = y_points[i]

            index = self.world_to_map_index(x, y)

            if index is None:
                response.total_fusion_cost = total_cost
                response.success = False
                response.message = f'Point ({x:.2f}, {y:.2f}) is outside the map.'
                return response

            cell_value = self.fusion_map.data[index]

            if cell_value < 0:
                cell_value = 100

            normalized_cost = cell_value / 10.0

            if i == 0:
                segment_distance = 0.0
            else:
                dx = x_points[i] - x_points[i - 1]
                dy = y_points[i] - y_points[i - 1]
                segment_distance = math.sqrt(dx * dx + dy * dy)

            total_cost += normalized_cost * segment_distance

        response.total_fusion_cost = total_cost
        response.success = True
        response.message = (
            f'Path contains {len(x_points)} points. '
            f'Distance-weighted fusion map cost = {total_cost:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathFusionMapCostService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
