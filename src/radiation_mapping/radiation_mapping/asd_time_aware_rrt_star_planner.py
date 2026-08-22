import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class RRTStarNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class ASDTimeAwareRRTStarPlanner(Node):
    def __init__(self):
        super().__init__('asd_time_aware_rrt_star_planner')

        random.seed(23)
        
        self.goal_received = False
        
        self.radiation_map = None
        self.terrain_map = None
        self.has_planned = False

        self.start = RRTStarNode(-2.0, -0.5)
        self.goal = RRTStarNode(4.0, 0.0)
        
        self.current_x = -2.0
        self.current_y = -0.5
        self.has_odom = False
        
        self.min_x = -5.0
        self.max_x = 5.0
        self.min_y = -5.0
        self.max_y = 5.0

        self.step_size = 0.45
        self.search_radius = 1.0
        self.goal_radius = 0.5
        self.max_iterations = 1800
        self.goal_sample_rate = 0.12
        self.edge_sample_resolution = 0.2

        # Robot velocity model
        self.base_velocity = 0.5
        self.min_velocity = 0.05
        self.terrain_velocity_gain = 0.15

        # Time-aware path cost weights
        self.radiation_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        # APF weights
        self.random_weight = 0.35
        self.goal_attractive_weight = 0.45
        self.risk_repulsive_weight = 0.20
        self.gradient_epsilon = 0.25

        # Subregional dynamic sampling parameters
        self.region_rows = 5
        self.region_cols = 5
        self.regions = []

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
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            'asd_time_aware_rrt_star_path',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'asd_time_aware_rrt_star_markers',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('ASD time-aware RRT* planner started.')

    def radiation_callback(self, msg):
        self.radiation_map = msg

    def terrain_callback(self, msg):
        self.terrain_map = msg
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.has_odom = True 
           
    def goal_callback(self, msg):
        if self.has_odom:
            self.start = RRTStarNode(self.current_x, self.current_y)
        else:
            self.start = RRTStarNode(-2.0, -0.5)

        self.goal = RRTStarNode(
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.goal_received = True
        self.has_planned = False
        self.final_path = []
        self.final_score = 0.0

        self.get_logger().info(
            f'Received RViz goal: x={self.goal.x:.2f}, y={self.goal.y:.2f}'
        )

        self.get_logger().info(
            f'Planning from start: x={self.start.x:.2f}, y={self.start.y:.2f}'
        )

        if self.radiation_map is not None and self.terrain_map is not None:
            self.build_sampling_regions()
            self.plan_path()
            self.has_planned = True
            self.publish_result()
            
    def timer_callback(self):
        if self.radiation_map is None or self.terrain_map is None:
            self.get_logger().info(
                'Waiting for radiation_map and terrain_cost_map...'
            )
            return
            
        if not self.goal_received:
            return

        if not self.has_planned:
            self.build_sampling_regions()
            self.plan_path()
            self.has_planned = True

        self.publish_result()

    def geometric_distance(self, node_a, node_b):
        dx = node_a.x - node_b.x
        dy = node_a.y - node_b.y
        return math.sqrt(dx * dx + dy * dy)

    def normalize_vector(self, vx, vy):
        norm = math.sqrt(vx * vx + vy * vy)

        if norm < 1e-6:
            return 0.0, 0.0

        return vx / norm, vy / norm

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

    def get_combined_risk(self, x, y):
        radiation_value = self.get_map_value(self.radiation_map, x, y)
        terrain_value = self.get_map_value(self.terrain_map, x, y)

        if radiation_value is None or terrain_value is None:
            return None

        radiation_risk = radiation_value / 100.0
        terrain_risk = terrain_value / 100.0

        return 0.5 * radiation_risk + 0.5 * terrain_risk

    def build_sampling_regions(self):
        self.regions = []

        region_width = (self.max_x - self.min_x) / self.region_cols
        region_height = (self.max_y - self.min_y) / self.region_rows

        for row in range(self.region_rows):
            for col in range(self.region_cols):
                x_min = self.min_x + col * region_width
                x_max = x_min + region_width
                y_min = self.min_y + row * region_height
                y_max = y_min + region_height

                center_x = (x_min + x_max) / 2.0
                center_y = (y_min + y_max) / 2.0

                risk = self.get_combined_risk(center_x, center_y)

                if risk is None:
                    risk = 1.0

                distance_to_goal = math.sqrt(
                    (center_x - self.goal.x) ** 2 +
                    (center_y - self.goal.y) ** 2
                )

                low_risk_weight = max(0.05, (1.0 - risk) ** 3)
                goal_weight = 1.0 / (1.0 + 0.25 * distance_to_goal)

                sampling_weight = low_risk_weight * goal_weight

                self.regions.append({
                    'x_min': x_min,
                    'x_max': x_max,
                    'y_min': y_min,
                    'y_max': y_max,
                    'risk': risk,
                    'weight': sampling_weight,
                })

        self.get_logger().info(
            f'Subregional dynamic sampling initialized with '
            f'{len(self.regions)} regions.'
        )

    def sample_from_regions(self):
        total_weight = sum(region['weight'] for region in self.regions)

        if total_weight <= 0.0:
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            return RRTStarNode(x, y)

        threshold = random.uniform(0.0, total_weight)
        cumulative = 0.0

        selected_region = self.regions[-1]

        for region in self.regions:
            cumulative += region['weight']
            if cumulative >= threshold:
                selected_region = region
                break

        x = random.uniform(selected_region['x_min'], selected_region['x_max'])
        y = random.uniform(selected_region['y_min'], selected_region['y_max'])

        return RRTStarNode(x, y)

    def get_risk_repulsive_direction(self, x, y):
        eps = self.gradient_epsilon

        risk_x_plus = self.get_combined_risk(x + eps, y)
        risk_x_minus = self.get_combined_risk(x - eps, y)
        risk_y_plus = self.get_combined_risk(x, y + eps)
        risk_y_minus = self.get_combined_risk(x, y - eps)

        if (
            risk_x_plus is None or
            risk_x_minus is None or
            risk_y_plus is None or
            risk_y_minus is None
        ):
            return 0.0, 0.0

        grad_x = (risk_x_plus - risk_x_minus) / (2.0 * eps)
        grad_y = (risk_y_plus - risk_y_minus) / (2.0 * eps)

        repulsive_x = -grad_x
        repulsive_y = -grad_y

        return self.normalize_vector(repulsive_x, repulsive_y)

    def apply_apf_guidance(self, current_node, sampled_node):
        random_dx = sampled_node.x - current_node.x
        random_dy = sampled_node.y - current_node.y
        random_dx, random_dy = self.normalize_vector(random_dx, random_dy)

        goal_dx = self.goal.x - current_node.x
        goal_dy = self.goal.y - current_node.y
        goal_dx, goal_dy = self.normalize_vector(goal_dx, goal_dy)

        repulse_dx, repulse_dy = self.get_risk_repulsive_direction(
            current_node.x,
            current_node.y
        )

        guided_dx = (
            self.random_weight * random_dx +
            self.goal_attractive_weight * goal_dx +
            self.risk_repulsive_weight * repulse_dx
        )

        guided_dy = (
            self.random_weight * random_dy +
            self.goal_attractive_weight * goal_dy +
            self.risk_repulsive_weight * repulse_dy
        )

        guided_dx, guided_dy = self.normalize_vector(guided_dx, guided_dy)

        if abs(guided_dx) < 1e-6 and abs(guided_dy) < 1e-6:
            return sampled_node

        return RRTStarNode(
            current_node.x + self.step_size * guided_dx,
            current_node.y + self.step_size * guided_dy
        )

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

        return self.sample_from_regions()

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
            sampled_node = self.sample_random_node()
            nearest_index = self.get_nearest_node_index(sampled_node)
            nearest_node = self.nodes[nearest_index]

            guided_target = self.apply_apf_guidance(
                nearest_node,
                sampled_node
            )

            new_node = self.steer(nearest_node, guided_target)

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
            f'ASD time-aware RRT* path found with '
            f'{len(self.final_path)} points.'
        )
        self.get_logger().info(f'x_points: {x_points}')
        self.get_logger().info(f'y_points: {y_points}')
        self.get_logger().info(
            f'ASD time-aware path score = {self.final_score:.2f}'
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

        marker.ns = 'asd_time_aware_rrt_star_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.015

        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.35

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

        marker.ns = 'asd_time_aware_rrt_star_path'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.13

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for x, y in self.final_path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.35
            marker.points.append(p)

        return marker

    def create_text_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'asd_time_aware_rrt_star_label'
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.text = 'ASD-Time-Aware RRT*'

        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.2
        marker.pose.position.z = 0.9
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.35

        marker.color.r = 0.0
        marker.color.g = 1.0
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
    node = ASDTimeAwareRRTStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
