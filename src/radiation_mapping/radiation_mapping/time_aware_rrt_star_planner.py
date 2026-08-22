import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class RRTStarNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class TimeAwareRRTStarPlanner(Node):
    def __init__(self):
        super().__init__('time_aware_rrt_star_planner')

        random.seed(11)

        self.radiation_map = None
        self.terrain_map = None
        self.has_planned = False

        self.start = RRTStarNode(-4.0, 0.0)
        self.goal = RRTStarNode(4.0, 0.0)

        self.min_x = -5.0
        self.max_x = 5.0
        self.min_y = -5.0
        self.max_y = 5.0

        self.step_size = 0.45
        self.search_radius = 1.0
        self.goal_radius = 0.5
        self.max_iterations = 1600
        self.goal_sample_rate = 0.10
        self.edge_sample_resolution = 0.2

        self.base_velocity = 0.5
        self.min_velocity = 0.05
        self.terrain_velocity_gain = 0.15

        self.radiation_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        self.nodes = []
        self.final_path = []
        self.final_score = 0.0

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

        self.path_pub = self.create_publisher(
            Path,
            'time_aware_rrt_star_path',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'time_aware_rrt_star_markers',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Time-aware RRT* planner started.')

    def radiation_callback(self, msg):
        self.radiation_map = msg

    def terrain_callback(self, msg):
        self.terrain_map = msg

    def timer_callback(self):
        if self.radiation_map is None or self.terrain_map is None:
            self.get_logger().info(
                'Waiting for radiation_map and terrain_cost_map...'
            )
            return

        if not self.has_planned:
            self.plan_path()
            self.has_planned = True

        self.publish_result()

    def geometric_distance(self, node_a, node_b):
        dx = node_a.x - node_b.x
        dy = node_a.y - node_b.y
        return math.sqrt(dx * dx + dy * dy)

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

    def edge_cost(self, from_node, to_node):
        distance = self.geometric_distance(from_node, to_node)

        if distance <= 0.0:
            return 0.0

        steps = max(1, int(distance / self.edge_sample_resolution))
        sub_distance = distance / steps

        total_edge_score = 0.0

        for i in range(steps):
            ratio = (i + 0.5) / steps

            x = from_node.x + ratio * (to_node.x - from_node.x)
            y = from_node.y + ratio * (to_node.y - from_node.y)

            radiation_value = self.get_map_value(
                self.radiation_map,
                x,
                y
            )

            terrain_value = self.get_map_value(
                self.terrain_map,
                x,
                y
            )

            if radiation_value is None or terrain_value is None:
                return float('inf')

            velocity = self.terrain_to_velocity(terrain_value)
            segment_time = sub_distance / velocity

            terrain_cost = (terrain_value / 10.0) * sub_distance
            radiation_dose = radiation_value * segment_time

            segment_score = (
                self.radiation_weight * radiation_dose +
                self.terrain_weight * terrain_cost +
                self.time_weight * segment_time
            )

            total_edge_score += segment_score

        return total_edge_score

    def sample_random_node(self):
        if random.random() < self.goal_sample_rate:
            return RRTStarNode(self.goal.x, self.goal.y)

        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)

        return RRTStarNode(x, y)

    def get_nearest_node_index(self, random_node):
        distances = [
            self.geometric_distance(node, random_node)
            for node in self.nodes
        ]

        return distances.index(min(distances))

    def steer(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.step_size:
            new_node = RRTStarNode(to_node.x, to_node.y)
        else:
            theta = math.atan2(dy, dx)
            new_node = RRTStarNode(
                from_node.x + self.step_size * math.cos(theta),
                from_node.y + self.step_size * math.sin(theta)
            )

        new_node.parent = from_node
        new_node.cost = from_node.cost + self.edge_cost(from_node, new_node)

        return new_node

    def is_inside_map(self, node):
        return (
            self.min_x <= node.x <= self.max_x and
            self.min_y <= node.y <= self.max_y
        )

    def is_valid_edge(self, from_node, to_node):
        if not self.is_inside_map(to_node):
            return False

        cost = self.edge_cost(from_node, to_node)

        return math.isfinite(cost)

    def find_near_nodes(self, new_node):
        near_indices = []

        for i, node in enumerate(self.nodes):
            if self.geometric_distance(node, new_node) <= self.search_radius:
                near_indices.append(i)

        return near_indices

    def choose_best_parent(self, new_node, near_indices):
        if not near_indices:
            return new_node

        best_cost = new_node.cost
        best_parent = new_node.parent

        for index in near_indices:
            near_node = self.nodes[index]

            if not self.is_valid_edge(near_node, new_node):
                continue

            candidate_cost = near_node.cost + self.edge_cost(
                near_node,
                new_node
            )

            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_parent = near_node

        new_node.parent = best_parent
        new_node.cost = best_cost

        return new_node

    def rewire(self, new_node, near_indices):
        for index in near_indices:
            near_node = self.nodes[index]

            if not self.is_valid_edge(new_node, near_node):
                continue

            candidate_cost = new_node.cost + self.edge_cost(
                new_node,
                near_node
            )

            if candidate_cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = candidate_cost

    def extract_path(self, goal_node):
        path = []
        current = goal_node

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        path.reverse()
        return path

    def plan_path(self):
        self.nodes = [self.start]

        best_goal_node = None
        best_goal_cost = float('inf')

        for _ in range(self.max_iterations):
            random_node = self.sample_random_node()
            nearest_index = self.get_nearest_node_index(random_node)
            nearest_node = self.nodes[nearest_index]

            new_node = self.steer(nearest_node, random_node)

            if not self.is_valid_edge(nearest_node, new_node):
                continue

            if not math.isfinite(new_node.cost):
                continue

            near_indices = self.find_near_nodes(new_node)
            new_node = self.choose_best_parent(new_node, near_indices)

            self.nodes.append(new_node)
            self.rewire(new_node, near_indices)

            if self.geometric_distance(new_node, self.goal) <= self.goal_radius:
                goal_node = RRTStarNode(self.goal.x, self.goal.y)
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + self.edge_cost(
                    new_node,
                    goal_node
                )

                if goal_node.cost < best_goal_cost:
                    best_goal_node = goal_node
                    best_goal_cost = goal_node.cost

        if best_goal_node is None:
            nearest_to_goal_index = self.get_nearest_node_index(self.goal)
            nearest_to_goal = self.nodes[nearest_to_goal_index]

            best_goal_node = RRTStarNode(self.goal.x, self.goal.y)
            best_goal_node.parent = nearest_to_goal
            best_goal_node.cost = nearest_to_goal.cost + self.edge_cost(
                nearest_to_goal,
                best_goal_node
            )

        self.final_path = self.extract_path(best_goal_node)
        self.final_score = best_goal_node.cost

        x_points = [round(p[0], 2) for p in self.final_path]
        y_points = [round(p[1], 2) for p in self.final_path]

        self.get_logger().info(
            f'Time-aware RRT* path found with {len(self.final_path)} points.'
        )
        self.get_logger().info(f'x_points: {x_points}')
        self.get_logger().info(f'y_points: {y_points}')
        self.get_logger().info(
            f'Time-aware path score = {self.final_score:.2f}'
        )

    def create_path_msg(self):
        path_msg = Path()

        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.final_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.1
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

    def create_tree_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'time_aware_rrt_star_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.015

        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.4

        marker.pose.orientation.w = 1.0

        for node in self.nodes:
            if node.parent is None:
                continue

            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.05

            p2 = Point()
            p2.x = node.parent.x
            p2.y = node.parent.y
            p2.z = 0.05

            marker.points.append(p1)
            marker.points.append(p2)

        return marker

    def create_final_path_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'time_aware_rrt_star_path'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.10

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for x, y in self.final_path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.25
            marker.points.append(p)

        return marker

    def create_text_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'time_aware_rrt_star_label'
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.text = 'Time-aware RRT*'

        marker.pose.position.x = 0.0
        marker.pose.position.y = -0.6
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.35

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def publish_result(self):
        if not self.final_path:
            return

        self.path_pub.publish(self.create_path_msg())

        marker_array = MarkerArray()
        marker_array.markers.append(self.create_tree_marker())
        marker_array.markers.append(self.create_final_path_marker())
        marker_array.markers.append(self.create_text_marker())

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TimeAwareRRTStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
