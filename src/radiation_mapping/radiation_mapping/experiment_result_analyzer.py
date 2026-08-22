import csv
import math
import os
from collections import defaultdict

import rclpy
from rclpy.node import Node


class ExperimentResultAnalyzer(Node):
    def __init__(self):
        super().__init__('experiment_result_analyzer')

        self.declare_parameter(
            'result_csv',
            '~/terrain_radiation_ws/experiment_results/execution_results.csv'
        )

        self.declare_parameter(
            'summary_csv',
            '~/terrain_radiation_ws/experiment_results/execution_summary.csv'
        )

        self.result_csv = os.path.expanduser(
            self.get_parameter('result_csv')
            .get_parameter_value()
            .string_value
        )

        self.summary_csv = os.path.expanduser(
            self.get_parameter('summary_csv')
            .get_parameter_value()
            .string_value
        )

        self.metrics = [
            'execution_time_s',
            'executed_path_length_m',
            'dose_during_path_following',
            'executed_terrain_cost',
            'executed_final_coupled_score',
        ]

        self.get_logger().info('Experiment result analyzer started.')
        self.get_logger().info(f'Result CSV: {self.result_csv}')
        self.get_logger().info(f'Summary CSV: {self.summary_csv}')

    def safe_float(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def mean(self, values):
        if not values:
            return 0.0

        return sum(values) / len(values)

    def std(self, values):
        if len(values) < 2:
            return 0.0

        avg = self.mean(values)
        variance = sum((value - avg) ** 2 for value in values) / (len(values) - 1)

        return math.sqrt(variance)

    def load_results(self):
        if not os.path.exists(self.result_csv):
            self.get_logger().error(
                f'Result CSV does not exist: {self.result_csv}'
            )
            return {}

        grouped_results = defaultdict(lambda: defaultdict(list))

        with open(self.result_csv, mode='r', newline='') as csv_file:
            reader = csv.DictReader(csv_file)

            for row in reader:
                planner_name = row.get('planner_name', 'Unknown Planner')

                for metric in self.metrics:
                    value = self.safe_float(row.get(metric))

                    if value is not None:
                        grouped_results[planner_name][metric].append(value)

        return grouped_results

    def compute_summary(self, grouped_results):
        summary_rows = []

        for planner_name, metric_dict in grouped_results.items():
            row = {
                'planner_name': planner_name,
                'num_trials': 0,
            }

            num_trials = 0

            for metric in self.metrics:
                values = metric_dict.get(metric, [])

                if len(values) > num_trials:
                    num_trials = len(values)

                row[f'{metric}_mean'] = self.mean(values)
                row[f'{metric}_std'] = self.std(values)

            row['num_trials'] = num_trials
            summary_rows.append(row)

        return summary_rows

    def save_summary(self, summary_rows):
        result_dir = os.path.dirname(self.summary_csv)

        if result_dir:
            os.makedirs(result_dir, exist_ok=True)

        fieldnames = [
            'planner_name',
            'num_trials',
        ]

        for metric in self.metrics:
            fieldnames.append(f'{metric}_mean')
            fieldnames.append(f'{metric}_std')

        with open(self.summary_csv, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for row in summary_rows:
                output_row = {}

                for field in fieldnames:
                    value = row.get(field, '')

                    if isinstance(value, float):
                        output_row[field] = f'{value:.2f}'
                    else:
                        output_row[field] = value

                writer.writerow(output_row)

        self.get_logger().info(
            f'Summary saved to: {self.summary_csv}'
        )

    def find_planner_row(self, summary_rows, target_name):
        for row in summary_rows:
            planner_name = row.get('planner_name', '')

            if planner_name == target_name:
                return row

        return None

    def report_reduction(self, summary_rows):
        baseline_row = self.find_planner_row(summary_rows, 'Baseline RRT*')
        asd_row = self.find_planner_row(summary_rows, 'ASD-Time-Aware RRT*')

        if baseline_row is None or asd_row is None:
            self.get_logger().warn(
                'Baseline RRT* and ASD-Time-Aware RRT* results are both required '
                'to compute reduction percentages.'
            )
            return

        comparison_metrics = [
            ('dose_during_path_following', 'Dose'),
            ('executed_terrain_cost', 'Terrain cost'),
            ('executed_final_coupled_score', 'Final coupled score'),
        ]

        self.get_logger().info(
            'Reduction from Baseline RRT* to ASD-Time-Aware RRT*:'
        )

        for metric, label in comparison_metrics:
            baseline_value = baseline_row.get(f'{metric}_mean', 0.0)
            asd_value = asd_row.get(f'{metric}_mean', 0.0)

            if baseline_value <= 0.0:
                self.get_logger().warn(
                    f'Cannot compute reduction for {label}: baseline value is zero.'
                )
                continue

            reduction = (baseline_value - asd_value) / baseline_value * 100.0

            self.get_logger().info(
                f'{label} reduction = {reduction:.2f}% '
                f'({baseline_value:.2f} -> {asd_value:.2f})'
            )

    def print_summary(self, summary_rows):
        self.get_logger().info('Experiment summary:')

        for row in summary_rows:
            planner_name = row.get('planner_name', 'Unknown Planner')
            num_trials = row.get('num_trials', 0)

            self.get_logger().info(
                f'Planner: {planner_name}, trials: {num_trials}'
            )

            for metric in self.metrics:
                mean_value = row.get(f'{metric}_mean', 0.0)
                std_value = row.get(f'{metric}_std', 0.0)

                self.get_logger().info(
                    f'  {metric}: mean={mean_value:.2f}, std={std_value:.2f}'
                )

    def analyze_results(self):
        grouped_results = self.load_results()

        if not grouped_results:
            self.get_logger().warn('No experiment results were loaded.')
            return

        summary_rows = self.compute_summary(grouped_results)

        self.print_summary(summary_rows)
        self.report_reduction(summary_rows)
        self.save_summary(summary_rows)


def main(args=None):
    rclpy.init(args=args)

    node = ExperimentResultAnalyzer()
    node.analyze_results()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
