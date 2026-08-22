#!/usr/bin/env python3

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class PlannerComparisonRecorder(Node):
    def __init__(self):
        super().__init__('planner_comparison_recorder')

        self.declare_parameter(
            'planner_names',
            'rrt_star,asd_rrt_star,asd_rrt_star_teb'
        )
        self.declare_parameter(
            'path_topics',
            '/rrt_star/path,/asd_rrt_star/path,/asd_rrt_star_teb/path'
        )
        self.declare_parameter(
            'output_dir',
            os.path.expanduser('~/terrain_radiation_ws/module31_experiment_results')
        )
        self.declare_parameter('record_duration_sec', 60.0)

        self.planner_names = self.parse_list_parameter('planner_names')
        self.path_topics = self.parse_list_parameter('path_topics')
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.record_duration_sec = float(self.get_parameter('record_duration_sec').value)

        if len(self.planner_names) != len(self.path_topics):
            raise ValueError('planner_names and path_topics must have the same length')

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.record_csv = os.path.join(
            self.output_dir,
            f'planner_comparison_records_{timestamp}.csv'
        )
        self.summary_csv = os.path.join(
            self.output_dir,
            f'planner_comparison_summary_{timestamp}.csv'
        )

        self.start_time = time.monotonic()
        self.records = []
        self.latest_results = {}

        for planner_name, topic_name in zip(self.planner_names, self.path_topics):
            self.create_subscription(
                Path,
                topic_name,
                self.create_path_callback(planner_name, topic_name),
                10
            )

            self.get_logger().info(
                f'Recording planner: {planner_name}, topic: {topic_name}'
            )

        self.timer = self.create_timer(1.0, self.timer_callback)

    def parse_list_parameter(self, parameter_name):
        value = self.get_parameter(parameter_name).value

        if isinstance(value, str):
            return [item.strip() for item in value.split(',') if item.strip()]

        return list(value)

    def create_path_callback(self, planner_name, topic_name):
        def callback(path_msg):
            result = self.evaluate_path(planner_name, topic_name, path_msg)

            self.records.append(result)
            self.latest_results[planner_name] = result

            self.write_csv_files()

            self.get_logger().info(
                f"{planner_name}: "
                f"length={result['path_length']:.2f}, "
                f"radiation={result['radiation_dose']:.2f}, "
                f"terrain={result['terrain_cost']:.2f}, "
                f"fusion={result['fusion_cost']:.2f}, "
                f"score={result['final_score']:.2f}"
            )

        return callback

    def timer_callback(self):
        elapsed_time = time.monotonic() - self.start_time

        if elapsed_time >= self.record_duration_sec:
            self.write_csv_files()

            self.get_logger().info('Planner comparison recording finished.')
            self.get_logger().info(f'Record CSV: {self.record_csv}')
            self.get_logger().info(f'Summary CSV: {self.summary_csv}')

            self.timer.cancel()

    def evaluate_path(self, planner_name, topic_name, path_msg):
        points = []

        for pose_stamped in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            points.append((x, y))

        path_length = self.compute_path_length(points)
        radiation_dose, terrain_cost, fusion_cost = self.compute_path_costs(points)

        final_score = (
            0.25 * path_length +
            0.55 * radiation_dose +
            0.20 * terrain_cost
        )

        received_time = time.monotonic() - self.start_time

        return {
            'planner_name': planner_name,
            'path_topic': topic_name,
            'received_time_sec': received_time,
            'pose_count': len(points),
            'path_length': path_length,
            'radiation_dose': radiation_dose,
            'terrain_cost': terrain_cost,
            'fusion_cost': fusion_cost,
            'final_score': final_score,
        }

    def compute_path_length(self, points):
        total_length = 0.0

        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            total_length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        return total_length

    def compute_path_costs(self, points):
        total_radiation = 0.0
        total_terrain = 0.0
        total_fusion = 0.0

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
                segment_length = distance / steps

                radiation_cost = self.compute_radiation_cost(sample_x, sample_y)
                terrain_cost = self.compute_terrain_cost(sample_x, sample_y)
                fusion_cost = self.compute_fusion_cost(sample_x, sample_y)

                total_radiation += radiation_cost * segment_length
                total_terrain += (terrain_cost / 10.0) * segment_length
                total_fusion += (fusion_cost / 10.0) * segment_length

        return total_radiation, total_terrain, total_fusion

    def compute_radiation_value(self, x, y):
        hotspots = [
            (3.0, 3.0, 100.0, 0.8),
            (-2.0, 1.0, 80.0, 1.0),
            (1.0, -4.0, 60.0, 0.6),
        ]

        radiation = 0.0

        for hotspot_x, hotspot_y, amplitude, sigma in hotspots:
            dx = x - hotspot_x
            dy = y - hotspot_y
            distance_squared = dx * dx + dy * dy

            radiation += amplitude * math.exp(
                -distance_squared / (2.0 * sigma * sigma)
            )

        return radiation

    def compute_radiation_cost(self, x, y):
        radiation_value = self.compute_radiation_value(x, y)
        return min(10.0, radiation_value / 10.0)

    def compute_terrain_cost(self, x, y):
        base_cost = 5.0

        slope_region = 35.0 * math.exp(
            -((x + 1.5) ** 2 + (y - 1.5) ** 2) / (2.0 * 0.9 * 0.9)
        )

        rough_region = 30.0 * math.exp(
            -((x - 2.0) ** 2 + (y + 1.5) ** 2) / (2.0 * 0.8 * 0.8)
        )

        step_region = 0.0

        if -0.5 < x < 2.5 and 1.5 < y < 3.5:
            step_region = 35.0

        return min(100.0, base_cost + slope_region + rough_region + step_region)

    def compute_fusion_cost(self, x, y):
        radiation_value = min(100.0, self.compute_radiation_value(x, y))
        terrain_value = min(100.0, self.compute_terrain_cost(x, y))

        return min(100.0, 0.5 * radiation_value + 0.5 * terrain_value)

    def write_csv_files(self):
        fieldnames = [
            'planner_name',
            'path_topic',
            'received_time_sec',
            'pose_count',
            'path_length',
            'radiation_dose',
            'terrain_cost',
            'fusion_cost',
            'final_score',
        ]

        with open(self.record_csv, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for row in self.records:
                writer.writerow(row)

        summary_rows = sorted(
            self.latest_results.values(),
            key=lambda row: row['final_score']
        )

        with open(self.summary_csv, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=['rank'] + fieldnames)
            writer.writeheader()

            for rank, row in enumerate(summary_rows, start=1):
                output_row = {'rank': rank}
                output_row.update(row)
                writer.writerow(output_row)


def main(args=None):
    rclpy.init(args=args)

    node = PlannerComparisonRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
