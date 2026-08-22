#!/usr/bin/env python3

import math

import rclpy

from radiation_mapping.aco_planner import ACOPlanner
from radiation_mapping.aco_vehicle_safe_planner import (
    ACOVehicleSafePlanner,
)
from radiation_mapping.final_asd_rrt_star_planner import RRTStarNode


class ACOTrackableSafePlanner(ACOVehicleSafePlanner):
    """Vehicle-aware ACO whose final path must also be trackable.

    V7 checked only the planned centreline and could fall back to a dense,
    jagged lattice path. V8 checks a lateral execution corridor, restores the
    original trackable lattice scale and refuses to publish an unsafe fallback.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter(
            'aco_execution_corridor_half_width_m',
            0.20,
        )
        self.declare_parameter(
            'aco_execution_corridor_lateral_samples',
            5,
        )
        self.declare_parameter(
            'aco_corridor_penalty_weight',
            0.20,
        )
        self.declare_parameter(
            'aco_trackability_resample_spacing_m',
            0.35,
        )
        self.declare_parameter(
            'aco_trackability_max_turn_deg',
            55.0,
        )
        self.declare_parameter(
            'aco_max_path_to_direct_ratio',
            1.30,
        )

        self.execution_corridor_half_width = float(
            self.get_parameter(
                'aco_execution_corridor_half_width_m'
            ).value
        )
        self.execution_corridor_lateral_samples = int(
            self.get_parameter(
                'aco_execution_corridor_lateral_samples'
            ).value
        )
        self.corridor_penalty_weight = float(
            self.get_parameter('aco_corridor_penalty_weight').value
        )
        self.trackability_resample_spacing = float(
            self.get_parameter(
                'aco_trackability_resample_spacing_m'
            ).value
        )
        self.trackability_max_turn_deg = float(
            self.get_parameter('aco_trackability_max_turn_deg').value
        )
        self.max_path_to_direct_ratio = float(
            self.get_parameter('aco_max_path_to_direct_ratio').value
        )

        if self.execution_corridor_half_width < 0.0:
            raise ValueError(
                'aco_execution_corridor_half_width_m cannot be negative'
            )
        if self.execution_corridor_lateral_samples < 1:
            raise ValueError(
                'aco_execution_corridor_lateral_samples must be positive'
            )
        if self.corridor_penalty_weight < 0.0:
            raise ValueError('aco_corridor_penalty_weight cannot be negative')
        if self.trackability_resample_spacing <= 0.0:
            raise ValueError(
                'aco_trackability_resample_spacing_m must be positive'
            )
        if not 0.0 < self.trackability_max_turn_deg < 180.0:
            raise ValueError(
                'aco_trackability_max_turn_deg must be in (0, 180)'
            )
        if self.max_path_to_direct_ratio <= 1.0:
            raise ValueError(
                'aco_max_path_to_direct_ratio must be greater than 1'
            )

        self.get_logger().info(
            'ACO Trackable-Safe V8 enabled: '
            f'corridor=+/-{self.execution_corridor_half_width:.2f} m, '
            f'lateral_samples={self.execution_corridor_lateral_samples}, '
            f'max_turn={self.trackability_max_turn_deg:.1f} deg, '
            f'max_length_ratio={self.max_path_to_direct_ratio:.2f}'
        )

    def _corridor_offsets(self):
        count = self.execution_corridor_lateral_samples
        half_width = self.execution_corridor_half_width

        if count <= 1 or half_width <= 1e-9:
            return [0.0]

        return [
            -half_width + 2.0 * half_width * index / (count - 1)
            for index in range(count)
        ]

    def corridor_segment_assessment(self, from_node, to_node):
        distance = self.geometric_distance(from_node, to_node)
        if distance <= 1e-9:
            return True, 0.0, 0.0, 0.0

        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        normal_x = -dy / distance
        normal_y = dx / distance

        spacing = max(
            0.01,
            min(
                self.vehicle_path_sample_spacing,
                self.edge_sample_resolution,
                0.05,
            ),
        )
        steps = max(1, int(math.ceil(distance / spacing)))

        maximum_vehicle = 0.0
        maximum_risk = 0.0
        risk_sum = 0.0
        risk_count = 0

        for index in range(steps + 1):
            ratio = index / steps
            centre_x = from_node.x + ratio * dx
            centre_y = from_node.y + ratio * dy

            for offset in self._corridor_offsets():
                x = centre_x + offset * normal_x
                y = centre_y + offset * normal_y

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
                    return False, maximum_vehicle, maximum_risk, 1.0

                maximum_vehicle = max(maximum_vehicle, vehicle_value)
                maximum_risk = max(maximum_risk, vehicle_risk)

                if vehicle_value >= self.vehicle_block_threshold:
                    return False, maximum_vehicle, maximum_risk, 1.0
                if vehicle_risk >= self.vehicle_risk_hard_threshold:
                    return False, maximum_vehicle, maximum_risk, 1.0

                risk_sum += max(0.0, min(1.0, vehicle_risk / 100.0))
                risk_count += 1

        mean_risk = risk_sum / risk_count if risk_count else 0.0
        return True, maximum_vehicle, maximum_risk, mean_risk

    def edge_cost(self, from_node, to_node):
        # V7 still supplies the shared terrain/radiation cost and centreline
        # vehicle-risk penalty. V8 adds a hard lateral execution corridor.
        base_cost = super().edge_cost(from_node, to_node)
        if not math.isfinite(base_cost):
            return float('inf')

        safe, _, maximum_risk, mean_risk = self.corridor_segment_assessment(
            from_node,
            to_node,
        )
        if not safe:
            return float('inf')

        distance = self.geometric_distance(from_node, to_node)
        reference_length = max(
            self.common_cost_model.reference_length_m,
            1e-9,
        )
        corridor_penalty = (
            self.corridor_penalty_weight
            * distance
            / reference_length
            * (0.60 * mean_risk + 0.40 * maximum_risk / 100.0)
        )
        return base_cost + corridor_penalty

    def path_corridor_safe(self, points):
        if not points or len(points) < 2:
            return False, None, None

        maximum_vehicle = 0.0
        maximum_risk = 0.0

        for index in range(len(points) - 1):
            from_node = RRTStarNode(points[index][0], points[index][1])
            to_node = RRTStarNode(points[index + 1][0], points[index + 1][1])
            safe, vehicle_value, risk, _ = self.corridor_segment_assessment(
                from_node,
                to_node,
            )
            maximum_vehicle = max(maximum_vehicle, vehicle_value)
            maximum_risk = max(maximum_risk, risk)
            if not safe:
                return False, maximum_vehicle, maximum_risk

        return True, maximum_vehicle, maximum_risk

    @staticmethod
    def path_length(points):
        return sum(
            math.hypot(x1 - x0, y1 - y0)
            for (x0, y0), (x1, y1) in zip(points[:-1], points[1:])
        )

    def maximum_resampled_turn(self, points):
        sampled = self.resample_path_uniformly(
            points,
            self.trackability_resample_spacing,
        )
        if len(sampled) < 3:
            return 0.0

        return max(
            self.compute_turn_angle_deg(
                sampled[index - 1],
                sampled[index],
                sampled[index + 1],
            )
            for index in range(1, len(sampled) - 1)
        )

    def candidate_trackable_and_safe(self, points, label):
        safe, vehicle_max, risk_max = self.path_corridor_safe(points)
        if not safe:
            self.get_logger().warn(
                f'{label} rejected by execution-corridor check: '
                f'max_vehicle={vehicle_max}, max_risk={risk_max}'
            )
            return False

        maximum_turn = self.maximum_resampled_turn(points)
        direct_distance = math.hypot(
            self.goal.x - self.start.x,
            self.goal.y - self.start.y,
        )
        length = self.path_length(points)
        length_ratio = length / max(direct_distance, 1e-9)

        if maximum_turn > self.trackability_max_turn_deg:
            self.get_logger().warn(
                f'{label} rejected as too sharp: '
                f'max_turn={maximum_turn:.1f} deg'
            )
            return False

        if length_ratio > self.max_path_to_direct_ratio:
            self.get_logger().warn(
                f'{label} rejected as excessive detour: '
                f'length={length:.2f} m, ratio={length_ratio:.3f}'
            )
            return False

        self.get_logger().info(
            f'{label} accepted: length={length:.2f} m, '
            f'ratio={length_ratio:.3f}, max_turn={maximum_turn:.1f} deg, '
            f'max_vehicle={vehicle_max:.1f}, max_risk={risk_max:.1f}'
        )
        return True

    def greedy_corridor_shortcut(self, raw_path):
        if len(raw_path) <= 2:
            return list(raw_path)

        result = [raw_path[0]]
        current = 0
        final_index = len(raw_path) - 1

        while current < final_index:
            selected = None
            for candidate in range(final_index, current, -1):
                from_node = RRTStarNode(
                    raw_path[current][0],
                    raw_path[current][1],
                )
                to_node = RRTStarNode(
                    raw_path[candidate][0],
                    raw_path[candidate][1],
                )
                if math.isfinite(self.edge_cost(from_node, to_node)):
                    selected = candidate
                    break

            if selected is None or selected <= current:
                return []

            result.append(raw_path[selected])
            current = selected

        return result

    def post_process_path(self, raw_path):
        # Explicitly bypass V7's unsafe dense raw-lattice fallback.
        standard_candidate = ACOPlanner.post_process_path(self, raw_path)
        if self.candidate_trackable_and_safe(
            standard_candidate,
            'Standard smoothed ACO path',
        ):
            return standard_candidate

        shortcut = self.greedy_corridor_shortcut(raw_path)
        if shortcut:
            shortcut_candidate = ACOPlanner.post_process_path(self, shortcut)
            if self.candidate_trackable_and_safe(
                shortcut_candidate,
                'Corridor-shortcut ACO path',
            ):
                return shortcut_candidate

        self.get_logger().error(
            'ACO found no path that is both corridor-safe and trackable. '
            'No unsafe raw lattice fallback will be published.'
        )
        return []


def main(args=None):
    rclpy.init(args=args)
    node = ACOTrackableSafePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ACO Trackable-Safe V8 stopped.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
