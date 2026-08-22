import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from radiation_interfaces.srv import SelectBestTimeAwarePath


class BestTimeAwarePathSelectorService(Node):
    def __init__(self):
        super().__init__('best_time_aware_path_selector_service')

        self.radiation_map = None
        self.terrain_map = None

        self.base_velocity = 0.5
        self.min_velocity = 0.05
        self.terrain_velocity_gain = 0.15

        self.radiation_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        self.paths = [
            {
                'name': 'Path A: radiation-risk path',
                'x_points': [-4.0, -2.0, 0.0, 2.0, 4.0],
                'y_points': [0.0, 1.0, 1.5, 2.5, 0.0],
            },
            {
                'name': 'Path B: terrain-risk path',
                'x_points': [-4.0, -2.0, 0.0, 2.0, 4.0],
                'y_points': [0.0, -0.5, -1.0, -1.5, 0.0],
            },
            {
                'name': 'Path C: safe detour path',
                'x_points': [-4.0, -4.0, 0.0, 4.0, 4.0],
                'y_points': [0.0, -3.0, -4.5, -2.0, 0.0],
            },
        ]

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
            SelectBestTimeAwarePath,
            'select_best_time_aware_path',
            self.handle_query
        )

        self.get_logger().info('Best time-aware path selector service started.')

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
        terrain_cost = terrain_value / 10.0

        velocity = self.base_velocity / (
            1.0 + self.terrain_velocity_gain * terrain_cost
        )

        return max(self.min_velocity, velocity)

    def evaluate_path(self, x_points, y_points):
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

            if radiation_value is None or terrain_value is None:
                return None

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

        return {
            'path_length': total_length,
            'travel_time': total_time,
            'radiation_dose': total_radiation_dose,
            'terrain_cost': total_terrain_cost,
            'final_score': final_score,
        }

    def handle_query(self, request, response):
        if self.radiation_map is None:
            response.success = False
            response.message = 'Radiation map has not been received yet.'
            return response

        if self.terrain_map is None:
            response.success = False
            response.message = 'Terrain cost map has not been received yet.'
            return response

        path_lengths = []
        travel_times = []
        radiation_doses = []
        terrain_costs = []
        final_scores = []

        best_index = -1
        best_score = float('inf')
        best_name = ''

        for index, path in enumerate(self.paths):
            result = self.evaluate_path(
                path['x_points'],
                path['y_points']
            )

            if result is None:
                response.success = False
                response.message = f'{path["name"]} contains points outside the map.'
                return response

            path_lengths.append(result['path_length'])
            travel_times.append(result['travel_time'])
            radiation_doses.append(result['radiation_dose'])
            terrain_costs.append(result['terrain_cost'])
            final_scores.append(result['final_score'])

            if result['final_score'] < best_score:
                best_score = result['final_score']
                best_index = index
                best_name = path['name']

        response.best_path_name = best_name
        response.best_path_index = best_index
        response.best_final_score = best_score
        response.path_lengths = path_lengths
        response.travel_times = travel_times
        response.radiation_doses = radiation_doses
        response.terrain_costs = terrain_costs
        response.final_scores = final_scores
        response.success = True
        response.message = (
            f'Best path is {best_name}, '
            f'index = {best_index}, '
            f'final score = {best_score:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BestTimeAwarePathSelectorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
