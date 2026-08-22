#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from radiation_mapping.aco_planner import ACOPlanner


class ACOVehicleSafePlanner(ACOPlanner):
    """ACO with a hard Husky-footprint clearance constraint."""

    def __init__(self):
        super().__init__()

        self.declare_parameter(
            'aco_vehicle_impedance_topic',
            '/husky_vehicle_impedance_map',
        )
        self.declare_parameter(
            'aco_vehicle_risk_topic',
            '/husky_vehicle_collision_risk_map',
        )
        self.declare_parameter('aco_require_vehicle_map', True)
        self.declare_parameter('aco_vehicle_block_threshold', 99.5)
        self.declare_parameter('aco_vehicle_risk_hard_threshold', 90.0)
        self.declare_parameter('aco_vehicle_warning_threshold', 70.0)
        self.declare_parameter('aco_vehicle_risk_penalty_weight', 1.25)
        self.declare_parameter('aco_vehicle_path_sample_spacing_m', 0.04)

        self.vehicle_impedance_topic = str(
            self.get_parameter('aco_vehicle_impedance_topic').value
        )
        self.vehicle_risk_topic = str(
            self.get_parameter('aco_vehicle_risk_topic').value
        )
        self.require_vehicle_map = bool(
            self.get_parameter('aco_require_vehicle_map').value
        )
        self.vehicle_block_threshold = float(
            self.get_parameter('aco_vehicle_block_threshold').value
        )
        self.vehicle_risk_hard_threshold = float(
            self.get_parameter('aco_vehicle_risk_hard_threshold').value
        )
        self.vehicle_warning_threshold = float(
            self.get_parameter('aco_vehicle_warning_threshold').value
        )
        self.vehicle_risk_penalty_weight = float(
            self.get_parameter('aco_vehicle_risk_penalty_weight').value
        )
        self.vehicle_path_sample_spacing = float(
            self.get_parameter('aco_vehicle_path_sample_spacing_m').value
        )

        if not 0.0 <= self.vehicle_warning_threshold < 100.0:
            raise ValueError(
                'aco_vehicle_warning_threshold must be in [0, 100)'
            )
        if not (
            self.vehicle_warning_threshold
            < self.vehicle_risk_hard_threshold
            <= 100.0
        ):
            raise ValueError(
                'aco_vehicle_risk_hard_threshold must be greater than '
                'the warning threshold and at most 100'
            )
        if self.vehicle_block_threshold <= 0.0:
            raise ValueError('aco_vehicle_block_threshold must be positive')
        if self.vehicle_risk_penalty_weight < 0.0:
            raise ValueError(
                'aco_vehicle_risk_penalty_weight cannot be negative'
            )
        if self.vehicle_path_sample_spacing <= 0.0:
            raise ValueError(
                'aco_vehicle_path_sample_spacing_m must be positive'
            )

        self.vehicle_impedance_map = None
        self.vehicle_collision_risk_map = None
        self.pending_vehicle_goal = None
        self.vehicle_impedance_logged = False
        self.vehicle_risk_logged = False

        vehicle_qos = QoSProfile(depth=1)
        vehicle_qos.reliability = QoSReliabilityPolicy.RELIABLE
        vehicle_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.vehicle_impedance_sub = self.create_subscription(
            OccupancyGrid,
            self.vehicle_impedance_topic,
            self.vehicle_impedance_callback,
            vehicle_qos,
        )
        self.vehicle_risk_sub = self.create_subscription(
            OccupancyGrid,
            self.vehicle_risk_topic,
            self.vehicle_risk_callback,
            vehicle_qos,
        )

        self.get_logger().info(
            'ACO Vehicle-Safe V7 enabled: '
            f'grid={self.aco_grid_step:.2f} m, '
            f'ants={self.ant_count}, iterations={self.aco_iterations}, '
            f'vehicle_block={self.vehicle_block_threshold:.1f}, '
            f'hard_risk={self.vehicle_risk_hard_threshold:.1f}'
        )

    @staticmethod
    def same_map_geometry(first, second):
        if first is None or second is None:
            return True
        return (
            int(first.info.width) == int(second.info.width)
            and int(first.info.height) == int(second.info.height)
            and math.isclose(
                float(first.info.resolution),
                float(second.info.resolution),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            and math.isclose(
                float(first.info.origin.position.x),
                float(second.info.origin.position.x),
                rel_tol=0.0,
                abs_tol=1e-6,
            )
            and math.isclose(
                float(first.info.origin.position.y),
                float(second.info.origin.position.y),
                rel_tol=0.0,
                abs_tol=1e-6,
            )
        )

    def vehicle_impedance_callback(self, msg):
        if not self.same_map_geometry(self.terrain_map, msg):
            self.get_logger().error(
                'Vehicle impedance map geometry does not match terrain map.'
            )
            return
        self.vehicle_impedance_map = msg
        if not self.vehicle_impedance_logged:
            self.get_logger().info(
                f'Received vehicle impedance map: '
                f'{msg.info.width}x{msg.info.height}'
            )
            self.vehicle_impedance_logged = True
        self.try_pending_goal()

    def vehicle_risk_callback(self, msg):
        if not self.same_map_geometry(self.terrain_map, msg):
            self.get_logger().error(
                'Vehicle collision-risk map geometry does not match terrain map.'
            )
            return
        self.vehicle_collision_risk_map = msg
        if not self.vehicle_risk_logged:
            self.get_logger().info(
                f'Received vehicle collision-risk map: '
                f'{msg.info.width}x{msg.info.height}'
            )
            self.vehicle_risk_logged = True
        self.try_pending_goal()

    def vehicle_maps_ready(self):
        if not self.require_vehicle_map:
            return True
        return (
            self.vehicle_impedance_map is not None
            and self.vehicle_collision_risk_map is not None
        )

    def try_pending_goal(self):
        if self.pending_vehicle_goal is None:
            return
        if not self.vehicle_maps_ready():
            return
        if self.radiation_map is None or self.terrain_map is None:
            return

        goal = self.pending_vehicle_goal
        self.pending_vehicle_goal = None
        self.get_logger().info(
            'Vehicle maps are ready; processing the queued formal goal.'
        )
        super().goal_callback(goal)

    def goal_callback(self, msg):
        if self.require_vehicle_map and not self.vehicle_maps_ready():
            self.pending_vehicle_goal = msg
            self.get_logger().warn(
                'Vehicle-aware terrain maps are not ready. '
                'The latest formal goal has been queued.'
            )
            return
        super().goal_callback(msg)

    def edge_cost(self, from_node, to_node):
        distance = self.geometric_distance(from_node, to_node)
        if distance <= 0.0:
            return 0.0

        if self.require_vehicle_map and not self.vehicle_maps_ready():
            return float('inf')

        spacing = max(
            0.01,
            min(
                self.edge_sample_resolution,
                self.vehicle_path_sample_spacing,
                0.05,
            ),
        )
        steps = max(1, int(math.ceil(distance / spacing)))
        risk_values = []

        for index in range(steps + 1):
            ratio = index / steps
            x = from_node.x + ratio * (to_node.x - from_node.x)
            y = from_node.y + ratio * (to_node.y - from_node.y)

            if self.vehicle_maps_ready():
                vehicle_value = self.get_map_value(
                    self.vehicle_impedance_map,
                    x,
                    y,
                )
                vehicle_risk = self.get_map_value(
                    self.vehicle_collision_risk_map,
                    x,
                    y,
                )

                if vehicle_value is None or vehicle_risk is None:
                    return float('inf')
                if vehicle_value >= self.vehicle_block_threshold:
                    return float('inf')
                if vehicle_risk >= self.vehicle_risk_hard_threshold:
                    return float('inf')

                risk_values.append(
                    max(0.0, min(1.0, vehicle_risk / 100.0))
                )

        base_cost = super().edge_cost(from_node, to_node)
        if not math.isfinite(base_cost):
            return float('inf')
        if not risk_values:
            return base_cost

        mean_risk = sum(risk_values) / len(risk_values)
        max_risk = max(risk_values)
        warning = self.vehicle_warning_threshold / 100.0
        excess = max(0.0, max_risk - warning) / max(
            1.0 - warning,
            1e-6,
        )
        distance_term = distance / max(
            self.common_cost_model.reference_length_m,
            1e-9,
        )
        safety_penalty = (
            self.vehicle_risk_penalty_weight
            * distance_term
            * (
                0.35 * mean_risk
                + 0.65 * max_risk
                + 1.50 * excess
            )
        )
        return base_cost + safety_penalty

    def path_vehicle_safe(self, points):
        if not points or len(points) < 2:
            return False, None, None
        if self.require_vehicle_map and not self.vehicle_maps_ready():
            return False, None, None

        maximum_impedance = 0.0
        maximum_risk = 0.0

        for index in range(len(points) - 1):
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
            distance = math.hypot(x1 - x0, y1 - y0)
            steps = max(
                1,
                int(math.ceil(distance / self.vehicle_path_sample_spacing)),
            )

            for sample_index in range(steps + 1):
                ratio = sample_index / steps
                x = x0 + ratio * (x1 - x0)
                y = y0 + ratio * (y1 - y0)
                vehicle_value = self.get_map_value(
                    self.vehicle_impedance_map,
                    x,
                    y,
                )
                vehicle_risk = self.get_map_value(
                    self.vehicle_collision_risk_map,
                    x,
                    y,
                )

                if vehicle_value is None or vehicle_risk is None:
                    return False, None, None

                maximum_impedance = max(maximum_impedance, vehicle_value)
                maximum_risk = max(maximum_risk, vehicle_risk)

                if vehicle_value >= self.vehicle_block_threshold:
                    return False, maximum_impedance, maximum_risk
                if vehicle_risk >= self.vehicle_risk_hard_threshold:
                    return False, maximum_impedance, maximum_risk

        return True, maximum_impedance, maximum_risk

    def post_process_path(self, raw_path):
        candidate = super().post_process_path(raw_path)
        safe, vehicle_max, risk_max = self.path_vehicle_safe(candidate)
        if safe:
            self.get_logger().info(
                'Vehicle-safe post-processing accepted: '
                f'max_vehicle={vehicle_max:.1f}, max_risk={risk_max:.1f}'
            )
            return candidate

        self.get_logger().warn(
            'Smoothed ACO path entered a vehicle-clearance risk region. '
            'Using the densified raw lattice path instead. '
            f'max_vehicle={vehicle_max}, max_risk={risk_max}'
        )
        fallback = self.densify_path(
            raw_path,
            self.final_densify_spacing,
        )
        fallback_safe, fallback_vehicle, fallback_risk = (
            self.path_vehicle_safe(fallback)
        )
        if fallback_safe:
            self.get_logger().info(
                'Vehicle-safe raw fallback accepted: '
                f'max_vehicle={fallback_vehicle:.1f}, '
                f'max_risk={fallback_risk:.1f}'
            )
            return fallback

        self.get_logger().error(
            'Raw ACO lattice path unexpectedly failed the final '
            'vehicle-safety check. No unsafe path will be published.'
        )
        return []


def main(args=None):
    rclpy.init(args=args)
    node = ACOVehicleSafePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ACO Vehicle-Safe V7 stopped.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
