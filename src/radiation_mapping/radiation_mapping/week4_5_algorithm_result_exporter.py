#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node


class Week45AlgorithmResultExporter(Node):
    def __init__(self):
        super().__init__('week4_5_algorithm_result_exporter')

        self.output_dir = os.path.expanduser(
            '~/terrain_radiation_ws/week4_5_algorithm_results'
        )
        os.makedirs(self.output_dir, exist_ok=True)

        self.common_start = (-4.0, 0.0)
        self.common_goal = (4.0, 0.0)

        self.radiation_map = None
        self.terrain_map = None
        self.fusion_map = None

        self.rrt_path = None
        self.asd_path = None
        self.apf_path = None

        self.saved = False

        self.create_subscription(
            OccupancyGrid,
            '/radiation_map',
            self.radiation_map_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/terrain_cost_map',
            self.terrain_map_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/fusion_cost_map',
            self.fusion_map_callback,
            10
        )

        self.create_subscription(
            Path,
            '/rrt_star_baseline_path',
            self.rrt_path_callback,
            10
        )

        self.create_subscription(
            Path,
            '/asd_time_aware_rrt_star_path',
            self.asd_path_callback,
            10
        )

        self.create_subscription(
            Path,
            '/apf_time_aware_rrt_star_path',
            self.apf_path_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.try_save_results)

        self.get_logger().info('Week 4-5 algorithm result exporter started.')
        self.get_logger().info(
            'Waiting for /radiation_map, /terrain_cost_map, '
            '/fusion_cost_map, /rrt_star_baseline_path, '
            '/asd_time_aware_rrt_star_path, and '
            '/apf_time_aware_rrt_star_path ...'
        )

    def radiation_map_callback(self, msg):
        self.radiation_map = msg

    def terrain_map_callback(self, msg):
        self.terrain_map = msg

    def fusion_map_callback(self, msg):
        self.fusion_map = msg

    def rrt_path_callback(self, msg):
        self.rrt_path = msg

    def asd_path_callback(self, msg):
        self.asd_path = msg

    def apf_path_callback(self, msg):
        self.apf_path = msg

    def try_save_results(self):
        if self.saved:
            return

        required_messages = [
            self.radiation_map,
            self.terrain_map,
            self.fusion_map,
            self.rrt_path,
            self.asd_path,
            self.apf_path,
        ]

        if any(message is None for message in required_messages):
            self.get_logger().info(
                'Still waiting for required maps and planner paths ...'
            )
            return

        self.save_all_figures()
        self.save_metrics_csv()

        self.saved = True

        self.get_logger().info('Week 4-5 algorithm results saved successfully.')
        self.get_logger().info(f'Output directory: {self.output_dir}')

    def occupancy_grid_to_array(self, grid_msg):
        data = np.array(grid_msg.data, dtype=float)
        data[data < 0] = np.nan

        return data.reshape(
            (grid_msg.info.height, grid_msg.info.width)
        )

    def get_grid_extent(self, grid_msg):
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        width = grid_msg.info.width * grid_msg.info.resolution
        height = grid_msg.info.height * grid_msg.info.resolution

        return [
            origin_x,
            origin_x + width,
            origin_y,
            origin_y + height,
        ]

    def path_to_points(self, path_msg):
        points = []

        for pose_stamped in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            points.append((x, y))

        return points

    def path_to_common_start_goal_points(self, path_msg):
        points = self.path_to_points(path_msg)

        if len(points) == 0:
            return [self.common_start, self.common_goal]

        start_distance = math.sqrt(
            (points[0][0] - self.common_start[0]) ** 2 +
            (points[0][1] - self.common_start[1]) ** 2
        )

        goal_distance = math.sqrt(
            (points[-1][0] - self.common_goal[0]) ** 2 +
            (points[-1][1] - self.common_goal[1]) ** 2
        )

        if start_distance > 0.05:
            points.insert(0, self.common_start)
        else:
            points[0] = self.common_start

        if goal_distance > 0.05:
            points.append(self.common_goal)
        else:
            points[-1] = self.common_goal

        return points

    def plot_map(self, map_msg, title, filename):
        grid = self.occupancy_grid_to_array(map_msg)
        extent = self.get_grid_extent(map_msg)

        plt.figure(figsize=(7, 6))
        plt.imshow(
            grid,
            origin='lower',
            extent=extent,
            cmap='gray_r',
            vmin=0,
            vmax=100
        )
        plt.colorbar(label='Cost value')
        plt.title(title)
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')
        plt.grid(alpha=0.25)

        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=200, bbox_inches='tight')
        plt.close()

    def plot_paths_on_fusion_map(self, paths, title, filename):
        fusion_grid = self.occupancy_grid_to_array(self.fusion_map)
        extent = self.get_grid_extent(self.fusion_map)

        plt.figure(figsize=(7, 6))
        plt.imshow(
            fusion_grid,
            origin='lower',
            extent=extent,
            cmap='gray_r',
            vmin=0,
            vmax=100
        )
        plt.colorbar(label='Fusion cost')

        for label, path_msg, style in paths:
            points = self.path_to_common_start_goal_points(path_msg)

            if len(points) == 0:
                continue

            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]

            plt.plot(
                x_values,
                y_values,
                style,
                linewidth=2.5,
                marker='o',
                markersize=3,
                label=label
            )

        plt.scatter(
            [self.common_start[0]],
            [self.common_start[1]],
            s=80,
            c='cyan',
            edgecolors='black',
            label='Start'
        )

        plt.scatter(
            [self.common_goal[0]],
            [self.common_goal[1]],
            s=80,
            c='yellow',
            edgecolors='black',
            label='Goal'
        )

        plt.title(title)
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')
        plt.grid(alpha=0.25)
        plt.legend()

        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=200, bbox_inches='tight')
        plt.close()

    def save_all_figures(self):
        self.plot_map(
            self.fusion_map,
            'Fusion Cost Map Used for Planner Evaluation',
            'figure1_fusion_cost_map_week4_5.png'
        )

        self.plot_paths_on_fusion_map(
            [
                ('Baseline RRT* path', self.rrt_path, 'r-'),
            ],
            'Baseline RRT* Path on Fusion Cost Map',
            'figure2_rrt_star_path_week4_5.png'
        )

        self.plot_paths_on_fusion_map(
            [
                ('ASD-RRT* path', self.asd_path, 'g-'),
            ],
            'ASD-RRT* Path on Fusion Cost Map',
            'figure3_asd_rrt_star_path_week4_5.png'
        )

        self.plot_paths_on_fusion_map(
            [
                ('Baseline RRT* path', self.rrt_path, 'r-'),
                ('ASD-RRT* path', self.asd_path, 'g-'),
            ],
            'Baseline RRT* and ASD-RRT* Path Comparison',
            'figure4_rrt_vs_asd_comparison_week4_5.png'
        )

        self.plot_paths_on_fusion_map(
            [
                ('ASD-RRT* path', self.asd_path, 'g-'),
                ('APF-enhanced time-aware RRT* path', self.apf_path, 'b-'),
            ],
            'ASD-RRT* and APF-enhanced Time-aware RRT* Path Comparison',
            'figure5_asd_vs_apf_comparison_week4_5.png'
        )

    def compute_path_length(self, points):
        total_length = 0.0

        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            total_length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        return total_length

    def sample_map_value(self, map_msg, x, y):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)

        if col < 0 or row < 0:
            return 0.0

        if col >= map_msg.info.width or row >= map_msg.info.height:
            return 0.0

        index = row * map_msg.info.width + col
        value = map_msg.data[index]

        if value < 0:
            return 0.0

        return float(value)

    def compute_accumulated_cost(self, path_msg, map_msg):
        points = self.path_to_common_start_goal_points(path_msg)

        total_cost = 0.0
        sample_spacing = 0.05

        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < 1e-6:
                continue

            steps = max(1, int(distance / sample_spacing))

            for step in range(steps):
                ratio = (step + 0.5) / steps
                sample_x = x1 + ratio * dx
                sample_y = y1 + ratio * dy

                map_value = self.sample_map_value(
                    map_msg,
                    sample_x,
                    sample_y
                )

                total_cost += (map_value / 10.0) * (distance / steps)

        return total_cost

    def evaluate_path(self, planner_name, path_msg):
        points = self.path_to_common_start_goal_points(path_msg)

        path_length = self.compute_path_length(points)

        radiation_cost = self.compute_accumulated_cost(
            path_msg,
            self.radiation_map
        )

        terrain_cost = self.compute_accumulated_cost(
            path_msg,
            self.terrain_map
        )

        fusion_cost = self.compute_accumulated_cost(
            path_msg,
            self.fusion_map
        )

        final_score = (
            0.4 * path_length +
            0.4 * radiation_cost +
            0.2 * terrain_cost
        )

        return {
            'planner_name': planner_name,
            'pose_count': len(points),
            'path_length': path_length,
            'radiation_cost': radiation_cost,
            'terrain_cost': terrain_cost,
            'fusion_cost': fusion_cost,
            'final_score': final_score,
        }

    def save_metrics_csv(self):
        rows = [
            self.evaluate_path('Baseline RRT*', self.rrt_path),
            self.evaluate_path('ASD-RRT*', self.asd_path),
            self.evaluate_path('APF-enhanced time-aware RRT*', self.apf_path),
        ]

        rows = sorted(rows, key=lambda row: row['final_score'])

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = os.path.join(
            self.output_dir,
            f'week4_5_algorithm_planner_comparison_{timestamp}.csv'
        )

        fieldnames = [
            'rank',
            'planner_name',
            'pose_count',
            'path_length',
            'radiation_cost',
            'terrain_cost',
            'fusion_cost',
            'final_score',
        ]

        with open(output_path, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for rank, row in enumerate(rows, start=1):
                output_row = {'rank': rank}
                output_row.update(row)
                writer.writerow(output_row)


def main(args=None):
    rclpy.init(args=args)

    node = Week45AlgorithmResultExporter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
