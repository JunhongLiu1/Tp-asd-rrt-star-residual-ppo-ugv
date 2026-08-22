import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


import pathlib
from ament_index_python.packages import get_package_share_directory
from radiation_mapping.common_cost_model import CommonCostModel

class RRTStarNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RVizASDTimeAwareRRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rviz_asd_time_aware_rrt_star_planner')

        # Final-work shared cost-model parameters.
        self.declare_parameter(
            'cost_model_config',
            ''
        )

        self.declare_parameter(
            'cost_profile',
            'balanced'
        )

        self.declare_parameter(
            'include_time_penalty',
            False
        )

        self.declare_parameter(
            'random_seed',
            31
        )

        self.declare_parameter(
            'terrain_input_max',
            100.0
        )

        self.declare_parameter(
            'radiation_input_mode',
            'normalized_occupancy'
        )

        self.declare_parameter(
            'radiation_input_max',
            100.0
        )

        self.declare_parameter(
            'odom_to_map_x',
            0.0
        )

        self.declare_parameter(
            'odom_to_map_y',
            0.0
        )

        self.declare_parameter(
            'odom_to_map_yaw',
            0.0
        )

        cost_config_text = str(
            self.get_parameter(
                'cost_model_config'
            ).value
        ).strip()

        if cost_config_text:
            self.cost_model_config = pathlib.Path(
                cost_config_text
            ).expanduser().resolve()
        else:
            self.cost_model_config = (
                pathlib.Path(
                    get_package_share_directory(
                        'radiation_mapping'
                    )
                )
                / 'config'
                / 'final_cost_model_v1.json'
            )

        self.cost_profile = str(
            self.get_parameter(
                'cost_profile'
            ).value
        ).strip()

        self.include_time_penalty = bool(
            self.get_parameter(
                'include_time_penalty'
            ).value
        )

        self.random_seed = int(
            self.get_parameter(
                'random_seed'
            ).value
        )

        self.terrain_input_max = float(
            self.get_parameter(
                'terrain_input_max'
            ).value
        )

        self.radiation_input_mode = str(
            self.get_parameter(
                'radiation_input_mode'
            ).value
        ).strip().lower()

        self.radiation_input_max = float(
            self.get_parameter(
                'radiation_input_max'
            ).value
        )

        self.odom_to_map_x = float(
            self.get_parameter(
                'odom_to_map_x'
            ).value
        )

        self.odom_to_map_y = float(
            self.get_parameter(
                'odom_to_map_y'
            ).value
        )

        self.odom_to_map_yaw = float(
            self.get_parameter(
                'odom_to_map_yaw'
            ).value
        )

        if self.terrain_input_max <= 0.0:
            raise ValueError(
                'terrain_input_max must be positive'
            )

        if self.radiation_input_max <= 0.0:
            raise ValueError(
                'radiation_input_max must be positive'
            )

        if self.radiation_input_mode not in {
            'normalized_occupancy',
            'dose_rate_usv_h',
        }:
            raise ValueError(
                'radiation_input_mode must be '
                'normalized_occupancy or dose_rate_usv_h'
            )

        self.common_cost_model = CommonCostModel(
            self.cost_model_config
        )

        if self.cost_profile not in (
            self.common_cost_model.profile_names()
        ):
            raise ValueError(
                f'Unknown cost profile: '
                f'{self.cost_profile}'
            )

        self.get_logger().info(
            f'Cost model config: '
            f'{self.common_cost_model.config_path}'
        )

        self.get_logger().info(
            f'Common cost profile: '
            f'{self.cost_profile}'
        )

        self.get_logger().info(
            f'Include time penalty: '
            f'{self.include_time_penalty}'
        )

        self.get_logger().info(
            f'Time penalty lambda: '
            f'{self.common_cost_model.time_penalty_lambda}'
        )

        random.seed(self.random_seed)

        self.get_logger().info(
            f'Random seed: {self.random_seed}'
        )

        self.radiation_map = None
        self.terrain_map = None
        self.traversability_map = None

        self.has_radiation_map = False
        self.has_terrain_map = False
        self.has_traversability_map = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.has_odom = False

        self.start = None
        self.goal = None

        self.min_x = -5.0
        self.max_x = 5.0
        self.min_y = -5.0
        self.max_y = 5.0

        # RRT* parameters
        self.step_size = 0.45
        self.search_radius = 1.0
        self.goal_radius = 0.5
        self.max_iterations = 1800
        self.goal_sample_rate = 0.12
        self.edge_sample_resolution = 0.2

        # Terrain-time model parameters
        self.base_velocity = 0.5
        self.min_velocity = 0.05
        self.terrain_velocity_gain = 0.15

        # Coupled objective weights
        self.radiation_weight = 0.5
        self.terrain_weight = 0.3
        self.time_weight = 0.2

        # APF-guided sampling parameters
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
        self.path_counter = 0

        # Path post-processing parameters
        self.enable_path_smoothing = True
        self.shortcut_smoothing_iterations = 60
        self.minimum_shortcut_waypoints = 6
        self.shortcut_cost_ratio = 0.98
        self.max_shortcut_risk = 0.45
        self.shortcut_risk_sample_spacing = 0.15

        # TEB-inspired elastic band smoothing parameters
        self.enable_teb_smoothing = True
        self.teb_anchor_spacing = 0.18
        self.final_densify_spacing = 0.08

        self.teb_iterations = 140
        self.teb_update_step = 0.10
        self.teb_max_point_shift = 0.06
        self.teb_max_deviation_from_anchor = 0.35

        self.teb_smooth_weight = 0.70
        self.teb_anchor_weight = 0.55
        self.teb_risk_weight = 0.20
        self.teb_curvature_weight = 0.25

        self.teb_gradient_epsilon = 0.18
        self.teb_local_risk_limit = 0.75
        self.teb_path_risk_limit = 0.75
        self.teb_score_tolerance = 1.12
        self.teb_risk_sample_spacing = 0.10

        # Endpoint protection prevents start/goal local collapse or hook shapes.
        self.lock_start_points = 4
        self.lock_end_points = 6
        self.endpoint_anchor_gain = 2.4

        # Turning-angle protection rejects local updates that create sharp hooks.
        self.max_turn_angle_deg = 80.0

        # Goal-tail simplification removes terminal overshoot and backtracking.
        self.goal_tail_simplification_enabled = True
        self.goal_tail_window = 18
        self.goal_tail_max_risk = 0.45
        self.goal_tail_score_ratio = 1.03

        radiation_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.radiation_sub = self.create_subscription(
            OccupancyGrid,
            '/radiation_map',
            self.radiation_callback,
            radiation_qos
        )

        self.terrain_sub = self.create_subscription(
            OccupancyGrid,
            '/terrain_cost_map',
            self.terrain_callback,
            radiation_qos
        )

        self.traversability_sub = self.create_subscription(
            OccupancyGrid,
            '/terrain_traversability_mask',
            self.traversability_callback,
            radiation_qos
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
            '/rviz_asd_time_aware_rrt_star_path',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/rviz_asd_time_aware_rrt_star_markers',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_result)

        self.get_logger().info(
            'RViz goal-driven ASD-Time-Aware RRT* planner started.'
        )
        self.get_logger().info(
            'Click a goal in RViz using the 2D Goal Pose tool.'
        )

    def radiation_callback(self, msg):
        self.radiation_map = msg
        self.has_radiation_map = True

    def traversability_callback(self, msg):
        self.traversability_map = msg
        self.has_traversability_map = True

    def terrain_callback(self, msg):
        self.terrain_map = msg
        self.has_terrain_map = True

        resolution = float(msg.info.resolution)
        width = int(msg.info.width)
        height = int(msg.info.height)

        if (
            resolution <= 0.0
            or width <= 0
            or height <= 0
        ):
            self.get_logger().error(
                'Invalid terrain map geometry; '
                'search bounds were not updated.'
            )
            return

        origin_x = float(
            msg.info.origin.position.x
        )
        origin_y = float(
            msg.info.origin.position.y
        )

        half_cell = 0.5 * resolution

        self.min_x = origin_x + half_cell
        self.max_x = (
            origin_x
            + width * resolution
            - half_cell
        )
        self.min_y = origin_y + half_cell
        self.max_y = (
            origin_y
            + height * resolution
            - half_cell
        )

        self.get_logger().info(
            'Planner search bounds updated from terrain map: '
            f'x=[{self.min_x:.3f}, {self.max_x:.3f}], '
            f'y=[{self.min_y:.3f}, {self.max_y:.3f}]'
        )

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        cos_yaw = math.cos(self.odom_to_map_yaw)
        sin_yaw = math.sin(self.odom_to_map_yaw)

        self.current_x = (
            self.odom_to_map_x
            + cos_yaw * odom_x
            - sin_yaw * odom_y
        )

        self.current_y = (
            self.odom_to_map_y
            + sin_yaw * odom_x
            + cos_yaw * odom_y
        )

        self.has_odom = True

    def goal_callback(self, msg):
        if (
            self.radiation_map is None
            or self.terrain_map is None
            or self.traversability_map is None
        ):
            self.get_logger().warn(
                'Radiation map, terrain map, or traversability mask '
                'not received yet. Cannot plan.'
            )
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        if self.has_odom:
            self.start = RRTStarNode(self.current_x, self.current_y)
        else:
            self.get_logger().warn(
                'No odometry received yet. Using default Module 31 start.'
            )
            self.start = RRTStarNode(-2.0, -0.5)

        self.goal = RRTStarNode(goal_x, goal_y)

        self.path_counter += 1

        self.get_logger().info(
            f'New RViz goal received. Planning path #{self.path_counter}.'
        )
        self.get_logger().info(
            f'Start: ({self.start.x:.2f}, {self.start.y:.2f}), '
            f'Goal: ({self.goal.x:.2f}, {self.goal.y:.2f})'
        )

        self.build_sampling_regions()
        self.plan_path()

    def is_inside_bounds(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def clamp_to_bounds(self, x, y):
        clamped_x = min(max(x, self.min_x), self.max_x)
        clamped_y = min(max(y, self.min_y), self.max_y)
        return clamped_x, clamped_y

    def geometric_distance(self, node_a, node_b):
        dx = node_a.x - node_b.x
        dy = node_a.y - node_b.y
        return math.sqrt(dx * dx + dy * dy)

    def point_distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def normalize_vector(self, vx, vy):
        norm = math.sqrt(vx * vx + vy * vy)

        if norm < 1e-6:
            return 0.0, 0.0

        return vx / norm, vy / norm

    def limit_vector_length(self, vx, vy, max_length):
        length = math.sqrt(vx * vx + vy * vy)

        if length <= max_length or length < 1e-6:
            return vx, vy

        scale = max_length / length
        return vx * scale, vy * scale

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
        if map_msg is None:
            return None

        index = self.world_to_map_index(map_msg, x, y)

        if index is None:
            return None

        value = map_msg.data[index]

        if value < 0:
            value = 100

        return float(value)

    def get_raw_map_value(self, map_msg, x, y):
        """Read an OccupancyGrid value without converting -1 to 100."""

        if map_msg is None:
            return None

        index = self.world_to_map_index(map_msg, x, y)

        if index is None:
            return None

        return float(map_msg.data[index])

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

    def get_risk_repulsive_direction(self, x, y, epsilon):
        risk_x_plus = self.get_combined_risk(x + epsilon, y)
        risk_x_minus = self.get_combined_risk(x - epsilon, y)
        risk_y_plus = self.get_combined_risk(x, y + epsilon)
        risk_y_minus = self.get_combined_risk(x, y - epsilon)

        if (
            risk_x_plus is None or
            risk_x_minus is None or
            risk_y_plus is None or
            risk_y_minus is None
        ):
            return 0.0, 0.0

        grad_x = (risk_x_plus - risk_x_minus) / (2.0 * epsilon)
        grad_y = (risk_y_plus - risk_y_minus) / (2.0 * epsilon)

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
            current_node.y,
            self.gradient_epsilon
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

    def normalize_terrain_input(self, terrain_value):
        """Convert the terrain OccupancyGrid value to [0, 1]."""

        if terrain_value is None:
            return None

        terrain_value = float(terrain_value)

        if (
            not math.isfinite(terrain_value)
            or terrain_value < 0.0
        ):
            return None

        return self.common_cost_model.clamp01(
            terrain_value
            / self.terrain_input_max
        )

    def radiation_value_to_dose_rate(
        self,
        radiation_value
    ):
        """Convert the planner radiation map to simulated uSv/h."""

        if radiation_value is None:
            return None

        radiation_value = float(
            radiation_value
        )

        if (
            not math.isfinite(radiation_value)
            or radiation_value < 0.0
        ):
            return None

        if (
            self.radiation_input_mode
            == 'dose_rate_usv_h'
        ):
            return radiation_value

        normalized_value = (
            self.common_cost_model.clamp01(
                radiation_value
                / self.radiation_input_max
            )
        )

        return (
            normalized_value
            * self.common_cost_model
            .radiation_reference_usv_h
        )

    def terrain_to_velocity(self, terrain_value):
        """Retained for compatibility with existing planner code."""

        terrain_impedance = (
            self.normalize_terrain_input(
                terrain_value
            )
        )

        if terrain_impedance is None:
            return (
                self.common_cost_model
                .nominal_speed_m_s
            )

        return (
            self.common_cost_model
            .estimate_speed_m_s(
                terrain_impedance
            )
        )

    def is_traversable_point(self, x, y):
        """Return True only for known traversable terrain cells."""

        value = self.get_raw_map_value(
            self.traversability_map,
            x,
            y
        )

        return (
            value is not None
            and value >= 50.0
        )

    def is_traversable_segment(self, from_node, to_node):
        """Check both endpoints and densely sample the complete edge."""

        if self.traversability_map is None:
            return False

        distance = self.geometric_distance(
            from_node,
            to_node
        )

        map_resolution = float(
            self.traversability_map.info.resolution
        )

        if map_resolution <= 0.0:
            return False

        # Half-cell sampling prevents a narrow blocked cell from being
        # skipped between two RRT* nodes.
        sample_spacing = min(
            0.05,
            0.5 * map_resolution
        )

        if distance <= 1e-9:
            return self.is_traversable_point(
                from_node.x,
                from_node.y
            )

        steps = max(
            1,
            int(math.ceil(distance / sample_spacing))
        )

        # Include index 0 and index steps so both edge endpoints are checked.
        for index in range(steps + 1):
            ratio = index / steps

            x = (
                from_node.x
                + ratio * (to_node.x - from_node.x)
            )
            y = (
                from_node.y
                + ratio * (to_node.y - from_node.y)
            )

            if not self.is_traversable_point(x, y):
                return False

        return True

    def edge_cost(self, from_node, to_node):
        """Evaluate an edge using the shared final-work cost model."""

        if not self.is_traversable_segment(
            from_node,
            to_node
        ):
            return float('inf')

        distance = self.geometric_distance(
            from_node,
            to_node
        )

        if distance <= 0.0:
            return 0.0

        steps = max(
            1,
            int(
                math.ceil(
                    distance
                    / self.edge_sample_resolution
                )
            )
        )

        sub_distance = distance / steps
        total_edge_cost = 0.0

        for index in range(steps):
            ratio = (
                index + 0.5
            ) / steps

            x = (
                from_node.x
                + ratio
                * (
                    to_node.x
                    - from_node.x
                )
            )

            y = (
                from_node.y
                + ratio
                * (
                    to_node.y
                    - from_node.y
                )
            )

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

            traversability_value = self.get_raw_map_value(
                self.traversability_map,
                x,
                y
            )

            # Traversability mask encoding:
            #   100 = traversable
            #     0 = non-traversable
            #    -1 = unknown
            #
            # Unknown and non-traversable cells are hard constraints.
            if (
                traversability_value is None
                or traversability_value < 50.0
            ):
                return float('inf')

            # OccupancyGrid values at or above 100 represent terrain
            # classified as non-traversable by the terrain pipeline.
            # They must be rejected rather than treated as merely
            # high but finite terrain impedance.
            if (
                terrain_value is None
                or terrain_value >= 100.0
            ):
                return float('inf')

            terrain_impedance = (
                self.normalize_terrain_input(
                    terrain_value
                )
            )

            dose_rate_usv_h = (
                self.radiation_value_to_dose_rate(
                    radiation_value
                )
            )

            if (
                terrain_impedance is None
                or dose_rate_usv_h is None
            ):
                return float('inf')

            result = (
                self.common_cost_model
                .evaluate_edge(
                    distance_m=sub_distance,
                    terrain_impedance=(
                        terrain_impedance
                    ),
                    dose_rate_usv_h=(
                        dose_rate_usv_h
                    ),
                    profile_name=(
                        self.cost_profile
                    ),
                    include_time_penalty=(
                        self.include_time_penalty
                    ),
                )
            )

            total_edge_cost += result[
                'total_cost'
            ]

        return total_edge_cost

    def sample_random_node(self):
        if random.random() < self.goal_sample_rate:
            return RRTStarNode(self.goal.x, self.goal.y)

        return self.sample_from_regions()

    def get_nearest_node_index(self, sampled_node):
        distances = [
            self.geometric_distance(node, sampled_node)
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
        return self.is_inside_bounds(node.x, node.y)

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

    def is_valid_segment_xy(self, x1, y1, x2, y2):
        from_node = RRTStarNode(x1, y1)
        to_node = RRTStarNode(x2, y2)
        edge_score = self.edge_cost(from_node, to_node)
        return math.isfinite(edge_score)

    def evaluate_path_score(self, path):
        if len(path) < 2:
            return 0.0

        total_score = 0.0

        for i in range(len(path) - 1):
            from_node = RRTStarNode(path[i][0], path[i][1])
            to_node = RRTStarNode(path[i + 1][0], path[i + 1][1])

            edge_score = self.edge_cost(from_node, to_node)

            if not math.isfinite(edge_score):
                return float('inf')

            total_score += edge_score

        return total_score

    def evaluate_subpath_score(self, path, index_a, index_b):
        if index_b <= index_a:
            return 0.0

        total_score = 0.0

        for i in range(index_a, index_b):
            from_node = RRTStarNode(path[i][0], path[i][1])
            to_node = RRTStarNode(path[i + 1][0], path[i + 1][1])

            edge_score = self.edge_cost(from_node, to_node)

            if not math.isfinite(edge_score):
                return float('inf')

            total_score += edge_score

        return total_score

    def get_segment_max_risk(self, x1, y1, x2, y2, sample_spacing):
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= 1e-6:
            risk = self.get_combined_risk(x1, y1)

            if risk is None:
                return 1.0

            return risk

        steps = max(1, int(distance / sample_spacing))
        max_risk = 0.0

        for i in range(steps + 1):
            ratio = i / steps

            x = x1 + ratio * dx
            y = y1 + ratio * dy

            risk = self.get_combined_risk(x, y)

            if risk is None:
                return 1.0

            if risk > max_risk:
                max_risk = risk

        return max_risk

    def get_path_max_risk(self, path, sample_spacing):
        if len(path) < 2:
            return 0.0

        max_risk = 0.0

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            segment_risk = self.get_segment_max_risk(
                x1,
                y1,
                x2,
                y2,
                sample_spacing
            )

            if segment_risk > max_risk:
                max_risk = segment_risk

        return max_risk

    def simplify_goal_tail(self, path):
        if not self.goal_tail_simplification_enabled:
            return path

        if len(path) < 6:
            return path

        goal_point = path[-1]
        goal_x, goal_y = goal_point
        start_index = max(1, len(path) - self.goal_tail_window)

        original_total_score = self.evaluate_path_score(path)

        if not math.isfinite(original_total_score):
            return path

        best_path = path
        best_score = original_total_score

        for i in range(start_index, len(path) - 3):
            x1, y1 = path[i]

            if not self.is_valid_segment_xy(x1, y1, goal_x, goal_y):
                continue

            tail_risk = self.get_segment_max_risk(
                x1,
                y1,
                goal_x,
                goal_y,
                self.teb_risk_sample_spacing
            )

            if tail_risk > self.goal_tail_max_risk:
                continue

            original_tail_score = self.evaluate_subpath_score(
                path,
                i,
                len(path) - 1
            )

            shortcut_tail_score = self.edge_cost(
                RRTStarNode(x1, y1),
                RRTStarNode(goal_x, goal_y)
            )

            if not math.isfinite(original_tail_score):
                continue

            if not math.isfinite(shortcut_tail_score):
                continue

            if shortcut_tail_score > original_tail_score * self.goal_tail_score_ratio:
                continue

            candidate_path = path[:i + 1] + [goal_point]
            candidate_score = self.evaluate_path_score(candidate_path)

            if not math.isfinite(candidate_score):
                continue

            if candidate_score < best_score:
                best_score = candidate_score
                best_path = candidate_path

        if len(best_path) < len(path):
            self.get_logger().info(
                f'Goal-tail simplification accepted. '
                f'Points: {len(path)} -> {len(best_path)}, '
                f'Score: {original_total_score:.2f} -> {best_score:.2f}'
            )

        return best_path

    def shortcut_smooth_path(self, raw_path):
        if len(raw_path) <= 2:
            return raw_path

        smoothed_path = list(raw_path)

        for _ in range(self.shortcut_smoothing_iterations):
            if len(smoothed_path) <= 3:
                break

            index_a = random.randint(0, len(smoothed_path) - 3)
            index_b = random.randint(index_a + 2, len(smoothed_path) - 1)

            x1, y1 = smoothed_path[index_a]
            x2, y2 = smoothed_path[index_b]

            if not self.is_valid_segment_xy(x1, y1, x2, y2):
                continue

            shortcut_risk = self.get_segment_max_risk(
                x1,
                y1,
                x2,
                y2,
                self.shortcut_risk_sample_spacing
            )

            if shortcut_risk > self.max_shortcut_risk:
                continue

            original_score = self.evaluate_subpath_score(
                smoothed_path,
                index_a,
                index_b
            )

            shortcut_score = self.edge_cost(
                RRTStarNode(x1, y1),
                RRTStarNode(x2, y2)
            )

            if not math.isfinite(original_score):
                continue

            if not math.isfinite(shortcut_score):
                continue

            if shortcut_score <= original_score * self.shortcut_cost_ratio:
                smoothed_path = (
                    smoothed_path[:index_a + 1] +
                    smoothed_path[index_b:]
                )

        return smoothed_path

    def densify_path(self, path, spacing):
        dense_path = []

        if len(path) == 0:
            return dense_path

        dense_path.append(path[0])

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx * dx + dy * dy)

            steps = max(1, int(distance / spacing))

            for step in range(1, steps + 1):
                ratio = step / steps
                x = x1 + ratio * dx
                y = y1 + ratio * dy
                dense_path.append((x, y))

        return dense_path

    def resample_path_uniformly(self, path, spacing):
        if len(path) < 2:
            return path

        cumulative_lengths = [0.0]

        for i in range(1, len(path)):
            segment_length = self.point_distance(path[i - 1], path[i])
            cumulative_lengths.append(cumulative_lengths[-1] + segment_length)

        total_length = cumulative_lengths[-1]

        if total_length < 1e-6:
            return path

        sample_distances = []
        current_distance = 0.0

        while current_distance < total_length:
            sample_distances.append(current_distance)
            current_distance += spacing

        sample_distances.append(total_length)

        resampled_path = []
        segment_index = 0

        for target_distance in sample_distances:
            while (
                segment_index < len(cumulative_lengths) - 2 and
                cumulative_lengths[segment_index + 1] < target_distance
            ):
                segment_index += 1

            d1 = cumulative_lengths[segment_index]
            d2 = cumulative_lengths[segment_index + 1]
            p1 = path[segment_index]
            p2 = path[segment_index + 1]

            if abs(d2 - d1) < 1e-9:
                resampled_path.append(p1)
                continue

            ratio = (target_distance - d1) / (d2 - d1)
            x = p1[0] + ratio * (p2[0] - p1[0])
            y = p1[1] + ratio * (p2[1] - p1[1])

            resampled_path.append((x, y))

        return resampled_path

    def compute_turn_angle_deg(self, previous_point, current_point, next_point):
        v1x = current_point[0] - previous_point[0]
        v1y = current_point[1] - previous_point[1]
        v2x = next_point[0] - current_point[0]
        v2y = next_point[1] - current_point[1]

        n1 = math.sqrt(v1x * v1x + v1y * v1y)
        n2 = math.sqrt(v2x * v2x + v2y * v2y)

        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        dot = (v1x * v2x + v1y * v2y) / (n1 * n2)
        dot = max(-1.0, min(1.0, dot))

        return math.degrees(math.acos(dot))

    def get_anchor_weight_scale(self, index, path_length):
        if index < self.lock_start_points:
            return self.endpoint_anchor_gain

        if index >= path_length - self.lock_end_points:
            return self.endpoint_anchor_gain

        if index < self.lock_start_points + 2:
            return 1.5

        if index >= path_length - self.lock_end_points - 2:
            return 1.5

        return 1.0

    def is_local_point_safe(
        self,
        previous_point,
        candidate_point,
        next_point,
        anchor_point
    ):
        candidate_x, candidate_y = candidate_point

        if not self.is_inside_bounds(candidate_x, candidate_y):
            return False

        if self.point_distance(candidate_point, anchor_point) > self.teb_max_deviation_from_anchor:
            return False

        previous_x, previous_y = previous_point
        next_x, next_y = next_point

        previous_risk = self.get_segment_max_risk(
            previous_x,
            previous_y,
            candidate_x,
            candidate_y,
            self.teb_risk_sample_spacing
        )

        next_risk = self.get_segment_max_risk(
            candidate_x,
            candidate_y,
            next_x,
            next_y,
            self.teb_risk_sample_spacing
        )

        if previous_risk > self.teb_local_risk_limit:
            return False

        if next_risk > self.teb_local_risk_limit:
            return False

        previous_node = RRTStarNode(previous_x, previous_y)
        candidate_node = RRTStarNode(candidate_x, candidate_y)
        next_node = RRTStarNode(next_x, next_y)

        if not self.is_valid_edge(previous_node, candidate_node):
            return False

        if not self.is_valid_edge(candidate_node, next_node):
            return False

        turn_angle = self.compute_turn_angle_deg(
            previous_point,
            candidate_point,
            next_point
        )

        if turn_angle > self.max_turn_angle_deg:
            return False

        return True

    def teb_elastic_band_smooth_path(self, input_path):
        if len(input_path) <= 2:
            return input_path

        anchor_path = list(input_path)
        smoothed_path = list(input_path)
        path_length = len(smoothed_path)

        for _ in range(self.teb_iterations):
            new_path = [smoothed_path[0]]

            for i in range(1, path_length - 1):
                current_point = smoothed_path[i]
                anchor_point = anchor_path[i]

                if i < self.lock_start_points or i >= path_length - self.lock_end_points:
                    new_path.append(anchor_point)
                    continue

                previous_x, previous_y = smoothed_path[i - 1]
                current_x, current_y = current_point
                next_x, next_y = smoothed_path[i + 1]
                anchor_x, anchor_y = anchor_point

                midpoint_x = 0.5 * (previous_x + next_x)
                midpoint_y = 0.5 * (previous_y + next_y)

                smooth_force_x = midpoint_x - current_x
                smooth_force_y = midpoint_y - current_y

                curvature_force_x = previous_x - 2.0 * current_x + next_x
                curvature_force_y = previous_y - 2.0 * current_y + next_y

                anchor_force_x = anchor_x - current_x
                anchor_force_y = anchor_y - current_y

                risk_force_x, risk_force_y = self.get_risk_repulsive_direction(
                    current_x,
                    current_y,
                    self.teb_gradient_epsilon
                )

                anchor_scale = self.get_anchor_weight_scale(i, path_length)

                update_x = self.teb_update_step * (
                    self.teb_smooth_weight * smooth_force_x +
                    self.teb_curvature_weight * curvature_force_x +
                    self.teb_anchor_weight * anchor_scale * anchor_force_x +
                    self.teb_risk_weight * risk_force_x
                )

                update_y = self.teb_update_step * (
                    self.teb_smooth_weight * smooth_force_y +
                    self.teb_curvature_weight * curvature_force_y +
                    self.teb_anchor_weight * anchor_scale * anchor_force_y +
                    self.teb_risk_weight * risk_force_y
                )

                update_x, update_y = self.limit_vector_length(
                    update_x,
                    update_y,
                    self.teb_max_point_shift
                )

                candidate_x = current_x + update_x
                candidate_y = current_y + update_y
                candidate_x, candidate_y = self.clamp_to_bounds(
                    candidate_x,
                    candidate_y
                )

                previous_point = (previous_x, previous_y)
                candidate_point = (candidate_x, candidate_y)
                next_point = (next_x, next_y)

                if self.is_local_point_safe(
                    previous_point,
                    candidate_point,
                    next_point,
                    anchor_point
                ):
                    new_path.append(candidate_point)
                else:
                    new_path.append(current_point)

            new_path.append(smoothed_path[-1])
            smoothed_path = new_path

        return smoothed_path

    def post_process_path(self, raw_path):
        if not self.enable_path_smoothing:
            return raw_path

        shortcut_path = self.shortcut_smooth_path(raw_path)

        if (
            len(shortcut_path) < self.minimum_shortcut_waypoints and
            len(raw_path) >= self.minimum_shortcut_waypoints
        ):
            self.get_logger().warn(
                'Shortcut smoothing removed too many waypoints. '
                'Using raw path before TEB-inspired smoothing.'
            )
            shortcut_path = raw_path

        base_path = self.resample_path_uniformly(
            shortcut_path,
            self.teb_anchor_spacing
        )

        base_path = self.simplify_goal_tail(base_path)

        if not self.enable_teb_smoothing:
            return self.densify_path(
                base_path,
                self.final_densify_spacing
            )

        teb_candidate = self.teb_elastic_band_smooth_path(base_path)
        teb_candidate = self.simplify_goal_tail(teb_candidate)

        teb_candidate = self.densify_path(
            teb_candidate,
            self.final_densify_spacing
        )

        base_dense_path = self.densify_path(
            base_path,
            self.final_densify_spacing
        )

        base_score = self.evaluate_path_score(base_dense_path)
        teb_score = self.evaluate_path_score(teb_candidate)
        teb_risk = self.get_path_max_risk(
            teb_candidate,
            self.teb_risk_sample_spacing
        )

        if not math.isfinite(teb_score):
            self.get_logger().warn(
                'TEB-inspired smoothing produced invalid path. '
                'Using resampled shortcut path instead.'
            )
            return base_dense_path

        if teb_score > base_score * self.teb_score_tolerance:
            self.get_logger().warn(
                'TEB-inspired smoothing increased coupled cost too much. '
                'Using resampled shortcut path instead.'
            )
            return base_dense_path

        if teb_risk > self.teb_path_risk_limit:
            self.get_logger().warn(
                'TEB-inspired smoothing entered high-risk regions. '
                'Using resampled shortcut path instead.'
            )
            return base_dense_path

        self.get_logger().info(
            f'TEB-inspired smoothing accepted. '
            f'Base score: {base_score:.2f}, '
            f'TEB score: {teb_score:.2f}, '
            f'TEB max risk: {teb_risk:.2f}'
        )

        return teb_candidate

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

                if not self.is_valid_edge(new_node, goal_node):
                    continue

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

        raw_path = self.extract_path(best_goal_node)
        processed_path = self.post_process_path(raw_path)

        processed_score = self.evaluate_path_score(processed_path)
        raw_score = self.evaluate_path_score(raw_path)

        if math.isfinite(processed_score):
            self.final_path = processed_path
            self.final_score = processed_score
        elif math.isfinite(raw_score):
            self.get_logger().warn(
                'Post-processed path failed final traversability '
                'validation. Falling back to the raw RRT* path.'
            )
            raw_dense_path = self.densify_path(
                raw_path,
                self.final_densify_spacing
            )
            raw_dense_score = self.evaluate_path_score(
                raw_dense_path
            )

            if not math.isfinite(raw_dense_score):
                self.get_logger().error(
                    'Densified raw RRT* path also failed final '
                    'traversability validation. Path will not be published.'
                )
                self.final_path = []
                self.final_score = float('inf')
                return

            self.final_path = raw_dense_path
            self.final_score = raw_dense_score
        else:
            self.get_logger().error(
                'Both processed and raw paths failed final '
                'traversability validation. Path will not be published.'
            )
            self.final_path = []
            self.final_score = float('inf')
            return

        self.get_logger().info(
            f'Path post-processing finished. '
            f'Raw points: {len(raw_path)}, '
            f'final points: {len(self.final_path)}'
        )

        self.get_logger().info(
            f'Raw path score: {raw_score:.2f}, '
            f'final smooth path score: {self.final_score:.2f}'
        )

        x_points = [round(p[0], 2) for p in self.final_path]
        y_points = [round(p[1], 2) for p in self.final_path]

        self.get_logger().info(
            f'RViz ASD-Time-Aware RRT* path found with '
            f'{len(self.final_path)} points.'
        )
       # self.get_logger().info(f'x_points: {x_points}')
       # self.get_logger().info(f'y_points: {y_points}')
        self.get_logger().info(
            f'RViz ASD TEB-smoothed path score = {self.final_score:.2f}'
        )

        self.publish_result()

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

        marker.ns = 'rviz_asd_rrt_star_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.015

        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.25

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

        marker.ns = 'rviz_asd_rrt_star_path'
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

    def create_start_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'rviz_asd_start'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.start.x
        marker.pose.position.y = self.start.y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_goal_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'rviz_asd_goal'
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.goal.x
        marker.pose.position.y = self.goal.y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_text_marker(self):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'rviz_asd_rrt_star_label'
        marker.id = 4
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.text = 'ASD-RRT* + optimized TEB-inspired smoothing'

        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.2
        marker.pose.position.z = 0.9
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.32

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
        marker_array.markers.append(self.create_start_marker())
        marker_array.markers.append(self.create_goal_marker())
        marker_array.markers.append(self.create_text_marker())

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RVizASDTimeAwareRRTStarPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'RViz ASD-Time-Aware RRT* planner stopped.'
        )

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
