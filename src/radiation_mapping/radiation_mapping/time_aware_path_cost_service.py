import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from radiation_interfaces.srv import QueryTimeAwarePathCost


class TimeAwarePathCostService(Node):
    def __init__(self):
        super().__init__('time_aware_path_cost_service')

        self.radiation_map = None
        self.terrain_map = None

        # Robot velocity model
        self.base_velocity = 0.5       # m/s on flat terrain
        self.min_velocity = 0.05       # minimum allowed velocity
        self.terrain_velocity_gain = 0.15

        # Final score weights
        self.radiation_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

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

        self.service = self.create_service(
            QueryTimeAwarePathCost,
            'query_time_aware_path_cost',
            self.handle_query
        )

        self.get_logger().info('Time-aware path cost service started.')

    def radiation_callback(self, msg):
        self.radiation_map = msg

    def terrain_callback(self, msg):
        self.terrain_map = msg

    def world_to_map_index(self, map_msg, x, y):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return None

        return map_y * width + map_x

    def get_map_value(self, map_msg, x, y):
        index = self.world_to_map_index(map_msg, x, y)

        if index is None:
            return None

        value = map_msg.data[index]

        if value < 0:
            value = 100

        return float(value)

    def terrain_to_velocity(self, terrain_value):
        # terrain_value is 0-100 from OccupancyGrid
        terrain_cost = terrain_value / 10.0

        velocity = self.base_velocity / (
            1.0 + self.terrain_velocity_gain * terrain_cost
        )

        return max(self.min_velocity, velocity)

    def handle_query(self, request, response):
        if self.radiation_map is None:
            response.success = False
            response.message = 'Radiation map has not been received yet.'
            return response

        if self.terrain_map is None:
            response.success = False
            response.message = 'Terrain cost map has not been received yet.'
            return response

        x_points = list(request.x_points)
        y_points = list(request.y_points)

        if len(x_points) != len(y_points):
            response.success = False
            response.message = 'x_points and y_points must have the same length.'
            return response

        if len(x_points) < 2:
            response.success = False
            response.message = 'Path must contain at least two points.'
            return response

        total_length = 0.0
        total_time = 0.0
        total_radiation_dose = 0.0
        total_terrain_cost = 0.0

        for i in range(1, len(x_points)):
            x0 = x_points[i - 1]
            y0 = y_points[i - 1]
            x1 = x_points[i]
            y1 = y_points[i]

            dx = x1 - x0
            dy = y1 - y0
            segment_distance = math.sqrt(dx * dx + dy * dy)

            # Use midpoint of the segment to sample map values
            mx = (x0 + x1) / 2.0
            my = (y0 + y1) / 2.0

            radiation_value = self.get_map_value(
                self.radiation_map,
                mx,
                my
            )

            terrain_value = self.get_map_value(
                self.terrain_map,
                mx,
                my
            )

            if radiation_value is None:
                response.success = False
                response.message = (
                    f'Radiation sample point ({mx:.2f}, {my:.2f}) '
                    f'is outside the map.'
                )
                return response

            if terrain_value is None:
                response.success = False
                response.message = (
                    f'Terrain sample point ({mx:.2f}, {my:.2f}) '
                    f'is outside the map.'
                )
                return response

            velocity = self.terrain_to_velocity(terrain_value)
            segment_time = segment_distance / velocity

            terrain_cost = terrain_value / 10.0
            radiation_dose = radiation_value * segment_time

            total_length += segment_distance
            total_time += segment_time
            total_radiation_dose += radiation_dose
            total_terrain_cost += terrain_cost * segment_distance

        final_score = (
            self.radiation_weight * total_radiation_dose +
            self.terrain_weight * total_terrain_cost +
            self.time_weight * total_time
        )

        response.path_length = total_length
        response.travel_time = total_time
        response.radiation_dose = total_radiation_dose
        response.terrain_cost = total_terrain_cost
        response.final_score = final_score
        response.success = True
        response.message = (
            f'Path length = {total_length:.2f} m, '
            f'travel time = {total_time:.2f} s, '
            f'radiation dose = {total_radiation_dose:.2f}, '
            f'terrain cost = {total_terrain_cost:.2f}, '
            f'final score = {final_score:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TimeAwarePathCostService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
