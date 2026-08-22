import math
import random

import rclpy

from radiation_mapping.final_asd_rrt_star_planner import (
    RRTStarNode,
    RVizASDTimeAwareRRTStarPlanner,
)


class ACOPlanner(RVizASDTimeAwareRRTStarPlanner):
    """Grid-lattice Ant Colony Optimization path planner.

    Map handling, edge validity, shared cost evaluation, path
    post-processing and ROS publication are inherited from the
    final ASD-RRT* planner so all planners use the same cost model.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('aco_ant_count', 50)
        self.declare_parameter('aco_iterations', 70)
        self.declare_parameter('aco_alpha', 1.0)
        self.declare_parameter('aco_beta', 4.0)
        self.declare_parameter('aco_evaporation_rate', 0.25)
        self.declare_parameter('aco_deposit_scale', 1.0)
        self.declare_parameter('aco_elite_weight', 3.0)
        self.declare_parameter('aco_grid_step_m', 0.45)
        self.declare_parameter('aco_goal_connection_radius_m', 0.75)
        self.declare_parameter('aco_goal_heuristic_weight', 0.35)
        self.declare_parameter('aco_max_steps', 240)
        self.declare_parameter('aco_seed', 31)
        self.declare_parameter(
            'aco_nontraversable_threshold',
            99.5,
        )

        self.ant_count = int(
            self.get_parameter('aco_ant_count').value
        )
        self.aco_iterations = int(
            self.get_parameter('aco_iterations').value
        )
        self.aco_alpha = float(
            self.get_parameter('aco_alpha').value
        )
        self.aco_beta = float(
            self.get_parameter('aco_beta').value
        )
        self.evaporation_rate = float(
            self.get_parameter(
                'aco_evaporation_rate'
            ).value
        )
        self.deposit_scale = float(
            self.get_parameter(
                'aco_deposit_scale'
            ).value
        )
        self.elite_weight = float(
            self.get_parameter(
                'aco_elite_weight'
            ).value
        )
        self.aco_grid_step = float(
            self.get_parameter('aco_grid_step_m').value
        )
        self.goal_connection_radius = float(
            self.get_parameter(
                'aco_goal_connection_radius_m'
            ).value
        )
        self.goal_heuristic_weight = float(
            self.get_parameter(
                'aco_goal_heuristic_weight'
            ).value
        )
        self.aco_max_steps = int(
            self.get_parameter('aco_max_steps').value
        )
        self.aco_seed = int(
            self.get_parameter('aco_seed').value
        )
        self.nontraversable_threshold = float(
            self.get_parameter(
                'aco_nontraversable_threshold'
            ).value
        )

        if self.ant_count <= 0:
            raise ValueError('aco_ant_count must be positive')

        if self.aco_iterations <= 0:
            raise ValueError('aco_iterations must be positive')

        if self.aco_grid_step <= 0.0:
            raise ValueError('aco_grid_step_m must be positive')

        if not 0.0 < self.evaporation_rate < 1.0:
            raise ValueError(
                'aco_evaporation_rate must be in (0, 1)'
            )

        if self.goal_connection_radius <= 0.0:
            raise ValueError(
                'aco_goal_connection_radius_m must be positive'
            )

        self.rng = random.Random(self.aco_seed)

        self._grid_columns = 0
        self._grid_rows = 0
        self._pheromone = {}
        self._edge_cost_cache = {}

        self.get_logger().info(
            'ACO planner initialized: '
            f'ants={self.ant_count}, '
            f'iterations={self.aco_iterations}, '
            f'grid_step={self.aco_grid_step:.3f} m, '
            f'seed={self.aco_seed}'
        )

    def edge_cost(self, from_node, to_node):
        """Reject blocked terrain before shared cost evaluation."""

        distance = self.geometric_distance(
            from_node,
            to_node,
        )

        if distance <= 0.0:
            return 0.0

        sample_spacing = max(
            0.01,
            min(
                self.edge_sample_resolution,
                0.05,
            ),
        )

        steps = max(
            1,
            int(
                math.ceil(
                    distance / sample_spacing
                )
            ),
        )

        for index in range(steps + 1):
            ratio = index / steps

            x = (
                from_node.x
                + ratio
                * (to_node.x - from_node.x)
            )
            y = (
                from_node.y
                + ratio
                * (to_node.y - from_node.y)
            )

            terrain_value = self.get_map_value(
                self.terrain_map,
                x,
                y,
            )

            if (
                terrain_value is None
                or terrain_value
                >= self.nontraversable_threshold
            ):
                return float('inf')

        return super().edge_cost(
            from_node,
            to_node,
        )

    def _initialize_lattice(self):
        self._grid_columns = (
            int(
                math.floor(
                    (self.max_x - self.min_x)
                    / self.aco_grid_step
                )
            )
            + 1
        )

        self._grid_rows = (
            int(
                math.floor(
                    (self.max_y - self.min_y)
                    / self.aco_grid_step
                )
            )
            + 1
        )

        self._pheromone = {}
        self._edge_cost_cache = {}

    def _cell_center(self, key):
        column, row = key

        x = self.min_x + column * self.aco_grid_step
        y = self.min_y + row * self.aco_grid_step

        return x, y

    def _nearest_cell(self, x, y):
        column = int(
            round(
                (x - self.min_x)
                / self.aco_grid_step
            )
        )
        row = int(
            round(
                (y - self.min_y)
                / self.aco_grid_step
            )
        )

        column = min(
            max(column, 0),
            self._grid_columns - 1,
        )
        row = min(
            max(row, 0),
            self._grid_rows - 1,
        )

        return column, row

    def _neighbor_cells(self, key):
        column, row = key

        for delta_column in (-1, 0, 1):
            for delta_row in (-1, 0, 1):
                if delta_column == 0 and delta_row == 0:
                    continue

                neighbor_column = column + delta_column
                neighbor_row = row + delta_row

                if (
                    0 <= neighbor_column
                    < self._grid_columns
                    and 0 <= neighbor_row
                    < self._grid_rows
                ):
                    yield (
                        neighbor_column,
                        neighbor_row,
                    )

    def _lattice_edge_cost(self, from_key, to_key):
        cache_key = (from_key, to_key)

        if cache_key in self._edge_cost_cache:
            return self._edge_cost_cache[cache_key]

        from_x, from_y = self._cell_center(from_key)
        to_x, to_y = self._cell_center(to_key)

        cost = self.edge_cost(
            RRTStarNode(from_x, from_y),
            RRTStarNode(to_x, to_y),
        )

        self._edge_cost_cache[cache_key] = cost
        self._edge_cost_cache[
            (to_key, from_key)
        ] = cost

        return cost

    def _roulette_select(self, candidates):
        total_weight = sum(
            candidate[2]
            for candidate in candidates
        )

        if (
            not math.isfinite(total_weight)
            or total_weight <= 0.0
        ):
            return min(
                candidates,
                key=lambda candidate: candidate[1],
            )

        threshold = self.rng.random() * total_weight
        cumulative = 0.0

        for candidate in candidates:
            cumulative += candidate[2]

            if cumulative >= threshold:
                return candidate

        return candidates[-1]

    def _construct_ant_path(
        self,
        start_key,
        start_prefix_cost,
    ):
        current_key = start_key
        path_keys = [start_key]
        visited = {start_key}
        total_cost = start_prefix_cost

        reference_length = max(
            1e-9,
            self.common_cost_model.reference_length_m,
        )

        for _ in range(self.aco_max_steps):
            current_x, current_y = self._cell_center(
                current_key
            )

            distance_to_goal = math.hypot(
                self.goal.x - current_x,
                self.goal.y - current_y,
            )

            if (
                distance_to_goal
                <= self.goal_connection_radius
            ):
                goal_edge_cost = self.edge_cost(
                    RRTStarNode(
                        current_x,
                        current_y,
                    ),
                    self.goal,
                )

                if math.isfinite(goal_edge_cost):
                    return (
                        path_keys,
                        total_cost + goal_edge_cost,
                    )

            candidates = []

            for neighbor_key in self._neighbor_cells(
                current_key
            ):
                if neighbor_key in visited:
                    continue

                edge_cost = self._lattice_edge_cost(
                    current_key,
                    neighbor_key,
                )

                if not math.isfinite(edge_cost):
                    continue

                neighbor_x, neighbor_y = (
                    self._cell_center(neighbor_key)
                )

                remaining_distance = math.hypot(
                    self.goal.x - neighbor_x,
                    self.goal.y - neighbor_y,
                )

                heuristic_cost = (
                    edge_cost
                    + self.goal_heuristic_weight
                    * remaining_distance
                    / reference_length
                )

                heuristic = (
                    1.0
                    / max(heuristic_cost, 1e-12)
                )

                pheromone = self._pheromone.get(
                    (current_key, neighbor_key),
                    1.0,
                )

                transition_weight = (
                    pheromone ** self.aco_alpha
                    * heuristic ** self.aco_beta
                )

                candidates.append((
                    neighbor_key,
                    edge_cost,
                    transition_weight,
                ))

            if not candidates:
                return None

            (
                next_key,
                transition_cost,
                _,
            ) = self._roulette_select(candidates)

            path_keys.append(next_key)
            visited.add(next_key)

            total_cost += transition_cost
            current_key = next_key

        return None

    def _evaporate_pheromone(self):
        retention = 1.0 - self.evaporation_rate

        for edge_key in list(self._pheromone):
            self._pheromone[edge_key] = max(
                1e-8,
                self._pheromone[edge_key]
                * retention,
            )

    def _deposit_path(self, path_keys, amount):
        for index in range(len(path_keys) - 1):
            forward_edge = (
                path_keys[index],
                path_keys[index + 1],
            )
            reverse_edge = (
                path_keys[index + 1],
                path_keys[index],
            )

            self._pheromone[forward_edge] = min(
                1e6,
                self._pheromone.get(
                    forward_edge,
                    1.0,
                )
                + amount,
            )

            self._pheromone[reverse_edge] = min(
                1e6,
                self._pheromone.get(
                    reverse_edge,
                    1.0,
                )
                + amount,
            )

    @staticmethod
    def _deduplicate_points(points):
        result = []

        for point in points:
            if not result:
                result.append(point)
                continue

            if math.hypot(
                point[0] - result[-1][0],
                point[1] - result[-1][1],
            ) > 1e-9:
                result.append(point)

        return result

    def _build_marker_nodes(self, raw_path):
        self.nodes = []

        previous_node = None
        cumulative_cost = 0.0

        for x, y in raw_path:
            node = RRTStarNode(x, y)
            node.parent = previous_node

            if previous_node is not None:
                cumulative_cost += self.edge_cost(
                    previous_node,
                    node,
                )

            node.cost = cumulative_cost
            self.nodes.append(node)
            previous_node = node

    def plan_path(self):
        if not self.is_inside_bounds(
            self.start.x,
            self.start.y,
        ):
            self.get_logger().error(
                'ACO start is outside terrain-map bounds.'
            )
            self.final_path = []
            return

        if not self.is_inside_bounds(
            self.goal.x,
            self.goal.y,
        ):
            self.get_logger().error(
                'ACO goal is outside terrain-map bounds.'
            )
            self.final_path = []
            return

        self._initialize_lattice()

        self.rng.seed(
            self.aco_seed + self.path_counter
        )

        start_key = self._nearest_cell(
            self.start.x,
            self.start.y,
        )

        start_cell_x, start_cell_y = (
            self._cell_center(start_key)
        )

        start_prefix_cost = self.edge_cost(
            self.start,
            RRTStarNode(
                start_cell_x,
                start_cell_y,
            ),
        )

        if not math.isfinite(start_prefix_cost):
            self.get_logger().error(
                'ACO could not connect the odometry start '
                'to the search lattice.'
            )
            self.final_path = []
            return

        global_best_keys = None
        global_best_cost = float('inf')

        self.get_logger().info(
            'ACO search started: '
            f'lattice={self._grid_columns}x'
            f'{self._grid_rows}, '
            f'ants={self.ant_count}, '
            f'iterations={self.aco_iterations}'
        )

        for iteration in range(self.aco_iterations):
            successful_paths = []

            for _ in range(self.ant_count):
                result = self._construct_ant_path(
                    start_key,
                    start_prefix_cost,
                )

                if result is None:
                    continue

                path_keys, path_cost = result

                if not math.isfinite(path_cost):
                    continue

                successful_paths.append((
                    path_keys,
                    path_cost,
                ))

                if path_cost < global_best_cost:
                    global_best_keys = list(path_keys)
                    global_best_cost = path_cost

            self._evaporate_pheromone()

            successful_paths.sort(
                key=lambda item: item[1]
            )

            for path_keys, path_cost in (
                successful_paths[:5]
            ):
                deposit = (
                    self.deposit_scale
                    / max(path_cost, 1e-12)
                )

                self._deposit_path(
                    path_keys,
                    deposit,
                )

            if global_best_keys is not None:
                elite_deposit = (
                    self.elite_weight
                    * self.deposit_scale
                    / max(global_best_cost, 1e-12)
                )

                self._deposit_path(
                    global_best_keys,
                    elite_deposit,
                )

            if (
                iteration == 0
                or (iteration + 1) % 10 == 0
                or iteration + 1
                == self.aco_iterations
            ):
                best_text = (
                    f'{global_best_cost:.6f}'
                    if math.isfinite(global_best_cost)
                    else 'none'
                )

                self.get_logger().info(
                    f'ACO iteration '
                    f'{iteration + 1}/'
                    f'{self.aco_iterations}: '
                    f'successful_ants='
                    f'{len(successful_paths)}, '
                    f'best_cost={best_text}'
                )

        if global_best_keys is None:
            self.get_logger().error(
                'ACO failed to find a valid route. '
                'No fallback goal connection was used.'
            )
            self.final_path = []
            return

        raw_path = [
            (self.start.x, self.start.y)
        ]

        raw_path.extend(
            self._cell_center(key)
            for key in global_best_keys
        )

        raw_path.append(
            (self.goal.x, self.goal.y)
        )

        raw_path = self._deduplicate_points(
            raw_path
        )

        raw_score = self.evaluate_path_score(
            raw_path
        )

        if not math.isfinite(raw_score):
            self.get_logger().error(
                'ACO generated a non-finite raw path.'
            )
            self.final_path = []
            return

        self._build_marker_nodes(raw_path)

        processed_path = self.post_process_path(
            raw_path
        )

        processed_score = self.evaluate_path_score(
            processed_path
        )

        if not math.isfinite(processed_score):
            self.get_logger().warn(
                'ACO post-processing produced an invalid '
                'path. Publishing the raw ACO path instead.'
            )
            processed_path = raw_path
            processed_score = raw_score

        self.final_path = processed_path
        self.final_score = processed_score

        self.get_logger().info(
            'ACO path post-processing finished. '
            f'Raw points: {len(raw_path)}, '
            f'final points: {len(self.final_path)}'
        )

        self.get_logger().info(
            f'ACO raw path score: {raw_score:.6f}, '
            f'final path score: '
            f'{self.final_score:.6f}'
        )

        self.get_logger().info(
            f'ACO path found with '
            f'{len(self.final_path)} points.'
        )

        self.publish_result()

    def create_text_marker(self):
        marker = super().create_text_marker()

        marker.ns = 'aco_planner_label'
        marker.text = ''
        marker.scale.z = 0.0
        marker.color.a = 0.0

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ACOPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'ACO planner stopped.'
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
