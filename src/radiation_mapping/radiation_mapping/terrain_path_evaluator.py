#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

from radiation_mapping.common_cost_model import CommonCostModel


class TerrainPathEvaluator(Node):

    VALID_LEVELS = ('easy', 'medium', 'hard')

    CSV_FIELDS = [
        'timestamp',
        'terrain_level',
        'planner_name',
        'path_topic',
        'frame_id',
        'path_points',
        'sample_count',
        'invalid_samples',
        'nontraversable_samples',
        'valid_sample_fraction',
        'traversable_sample_fraction',
        'path_length_m',
        'radiation_sample_count',
        'invalid_radiation_samples',
        'saturated_radiation_samples',
        'valid_radiation_sample_fraction',
        'estimated_traversal_time_s',
        'mean_dose_rate_usv_h',
        'max_dose_rate_usv_h',
        'accumulated_dose_usv',
        'radiation_cost_integral',
        'terrain_cost_integral',
        'mean_impedance',
        'max_impedance',
        'mean_slope_deg',
        'max_slope_deg',
        'mean_roughness_m',
        'max_roughness_m',
        'path_valid',
        'fully_traversable',
        'status',
    ]

    def __init__(self):
        super().__init__('terrain_path_evaluator')

        self.declare_parameter('terrain_level', 'easy')
        self.declare_parameter('data_directory', '')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter(
            'radiation_topic',
            '/radiation_map'
        )
        self.declare_parameter(
            'cost_model_config',
            ''
        )
        self.declare_parameter(
            'cost_profile',
            'balanced'
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
            'metrics_topic',
            '/terrain_path_metrics'
        )
        self.declare_parameter('planner_name', 'planner')
        self.declare_parameter('sample_step_m', 0.10)
        self.declare_parameter('invalid_cell_cost', 1.0)
        self.declare_parameter('deduplicate_paths', True)
        self.declare_parameter(
            'duplicate_tolerance_m',
            0.001
        )
        self.declare_parameter(
            'csv_path',
            '~/terrain_radiation_ws/results/'
            'path_metrics/terrain_radiation_path_metrics.csv'
        )

        self.terrain_level = str(
            self.get_parameter('terrain_level').value
        ).strip().lower()

        self.path_topic = str(
            self.get_parameter('path_topic').value
        ).strip()

        self.radiation_topic = str(
            self.get_parameter(
                'radiation_topic'
            ).value
        ).strip()

        self.cost_profile = str(
            self.get_parameter(
                'cost_profile'
            ).value
        ).strip()

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

        self.metrics_topic = str(
            self.get_parameter('metrics_topic').value
        ).strip()

        self.planner_name = str(
            self.get_parameter('planner_name').value
        ).strip()

        self.sample_step = float(
            self.get_parameter('sample_step_m').value
        )

        self.invalid_cell_cost = float(
            self.get_parameter('invalid_cell_cost').value
        )

        self.deduplicate_paths = bool(
            self.get_parameter(
                'deduplicate_paths'
            ).value
        )

        self.duplicate_tolerance = float(
            self.get_parameter(
                'duplicate_tolerance_m'
            ).value
        )

        if self.duplicate_tolerance <= 0.0:
            raise RuntimeError(
                'duplicate_tolerance_m must be positive'
            )

        self.last_path_signature = None
        self.radiation_map = None

        if not self.radiation_topic:
            raise RuntimeError(
                'radiation_topic cannot be empty'
            )

        if self.radiation_input_max <= 0.0:
            raise RuntimeError(
                'radiation_input_max must be positive'
            )

        if self.radiation_input_mode not in {
            'normalized_occupancy',
            'dose_rate_usv_h',
        }:
            raise RuntimeError(
                'radiation_input_mode must be '
                'normalized_occupancy or dose_rate_usv_h'
            )

        cost_config_text = str(
            self.get_parameter(
                'cost_model_config'
            ).value
        ).strip()

        if cost_config_text:
            self.cost_model_config = Path(
                cost_config_text
            ).expanduser().resolve()
        else:
            self.cost_model_config = (
                Path(
                    get_package_share_directory(
                        'radiation_mapping'
                    )
                )
                / 'config'
                / 'final_cost_model_v1.json'
            )

        self.common_cost_model = CommonCostModel(
            self.cost_model_config
        )

        if (
            self.cost_profile
            not in self.common_cost_model.profile_names()
        ):
            raise RuntimeError(
                f'Unknown cost profile: {self.cost_profile}'
            )

        csv_path_text = str(
            self.get_parameter('csv_path').value
        ).strip()

        if self.terrain_level not in self.VALID_LEVELS:
            raise RuntimeError(
                'terrain_level must be easy, medium, or hard'
            )

        if self.sample_step <= 0.0:
            raise RuntimeError(
                'sample_step_m must be positive'
            )

        if not self.path_topic:
            raise RuntimeError('path_topic cannot be empty')

        data_directory = str(
            self.get_parameter('data_directory').value
        ).strip()

        if data_directory:
            self.data_directory = Path(
                data_directory
            ).expanduser().resolve()
        else:
            package_share = Path(
                get_package_share_directory(
                    'radiation_mapping'
                )
            )

            self.data_directory = (
                package_share / 'dem' / 'processed'
            )

        prefix = (
            self.data_directory
            / f'terrain_layers_{self.terrain_level}'
        )

        self.npz_path = Path(str(prefix) + '.npz')
        self.metadata_path = Path(
            str(prefix) + '_metadata.json'
        )

        if csv_path_text:
            self.csv_path = Path(
                csv_path_text
            ).expanduser().resolve()

            self.csv_path.parent.mkdir(
                parents=True,
                exist_ok=True
            )
        else:
            self.csv_path = None

        self.load_terrain_data()

        self.metrics_publisher = self.create_publisher(
            String,
            self.metrics_topic,
            10
        )

        radiation_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.radiation_subscription = (
            self.create_subscription(
                OccupancyGrid,
                self.radiation_topic,
                self.radiation_callback,
                radiation_qos
            )
        )

        self.path_subscription = self.create_subscription(
            PathMessage,
            self.path_topic,
            self.path_callback,
            10
        )

        self.get_logger().info(
            'Terrain path evaluator started'
        )
        self.get_logger().info(
            f'Terrain: {self.terrain_level}'
        )
        self.get_logger().info(
            f'Planner: {self.planner_name}'
        )
        self.get_logger().info(
            f'Path topic: {self.path_topic}'
        )
        self.get_logger().info(
            f'Radiation topic: {self.radiation_topic}'
        )
        self.get_logger().info(
            'Radiation input: '
            f'{self.radiation_input_mode}, '
            f'max={self.radiation_input_max:.3f}'
        )
        self.get_logger().info(
            f'Cost profile: {self.cost_profile}'
        )
        self.get_logger().info(
            f'Metrics topic: {self.metrics_topic}'
        )
        self.get_logger().info(
            f'Sample step: {self.sample_step:.3f} m'
        )

        if self.csv_path is not None:
            self.get_logger().info(
                f'CSV: {self.csv_path}'
            )

    def load_terrain_data(self):
        if not self.npz_path.is_file():
            raise FileNotFoundError(
                f'NPZ not found: {self.npz_path}'
            )

        if not self.metadata_path.is_file():
            raise FileNotFoundError(
                f'Metadata not found: {self.metadata_path}'
            )

        with self.metadata_path.open(
            'r',
            encoding='utf-8'
        ) as file:
            metadata = json.load(file)

        grid = metadata['grid']

        self.resolution = float(
            grid['resolution_m']
        )
        self.width = int(
            grid['width_cells']
        )
        self.height = int(
            grid['height_cells']
        )
        self.origin_x = float(
            grid['origin_x_m']
        )
        self.origin_y = float(
            grid['origin_y_m']
        )

        self.maximum_x = (
            self.origin_x
            + self.width * self.resolution
        )
        self.maximum_y = (
            self.origin_y
            + self.height * self.resolution
        )

        with np.load(self.npz_path) as data:
            self.slope = np.asarray(
                data['slope_deg'],
                dtype=np.float64
            )
            self.roughness = np.asarray(
                data['roughness_m'],
                dtype=np.float64
            )
            self.impedance = np.asarray(
                data['terrain_impedance'],
                dtype=np.float64
            )
            self.validity = np.asarray(
                data['validity_mask'],
                dtype=bool
            )
            self.traversability = np.asarray(
                data['traversability_mask'],
                dtype=bool
            )

        expected_shape = (
            self.height,
            self.width
        )

        for name, array in {
            'slope': self.slope,
            'roughness': self.roughness,
            'impedance': self.impedance,
            'validity': self.validity,
            'traversability': self.traversability,
        }.items():
            if array.shape != expected_shape:
                raise RuntimeError(
                    f'{name} shape {array.shape} '
                    f'does not match {expected_shape}'
                )

    def query_cell(self, x, y):
        inside = (
            self.origin_x <= x < self.maximum_x
            and self.origin_y <= y < self.maximum_y
        )

        if not inside:
            return None

        column = int(
            math.floor(
                (x - self.origin_x)
                / self.resolution
            )
        )

        grid_row = int(
            math.floor(
                (y - self.origin_y)
                / self.resolution
            )
        )

        source_row = (
            self.height - 1 - grid_row
        )

        if not self.validity[source_row, column]:
            return {
                'valid': False,
                'traversable': False,
            }

        return {
            'valid': True,
            'traversable': bool(
                self.traversability[
                    source_row,
                    column
                ]
            ),
            'slope': float(
                self.slope[source_row, column]
            ),
            'roughness': float(
                self.roughness[
                    source_row,
                    column
                ]
            ),
            'impedance': float(
                np.clip(
                    self.impedance[
                        source_row,
                        column
                    ],
                    0.0,
                    1.0
                )
            ),
        }

    def radiation_callback(self, message):
        self.radiation_map = message

    @staticmethod
    def world_to_map_index(map_message, x, y):
        origin_x = (
            map_message.info.origin.position.x
        )
        origin_y = (
            map_message.info.origin.position.y
        )
        resolution = float(
            map_message.info.resolution
        )
        width = int(map_message.info.width)
        height = int(map_message.info.height)

        if (
            resolution <= 0.0
            or width <= 0
            or height <= 0
        ):
            return None

        map_x = int(
            (x - origin_x) / resolution
        )
        map_y = int(
            (y - origin_y) / resolution
        )

        if (
            map_x < 0
            or map_x >= width
            or map_y < 0
            or map_y >= height
        ):
            return None

        return map_y * width + map_x

    def query_radiation_value(self, x, y):
        if self.radiation_map is None:
            return None

        index = self.world_to_map_index(
            self.radiation_map,
            x,
            y
        )

        if index is None:
            return None

        if index >= len(self.radiation_map.data):
            return None

        value = float(
            self.radiation_map.data[index]
        )

        if (
            not math.isfinite(value)
            or value < 0.0
        ):
            return None

        return value

    def radiation_value_to_dose_rate(
        self,
        radiation_value
    ):
        if radiation_value is None:
            return None

        value = float(radiation_value)

        if (
            not math.isfinite(value)
            or value < 0.0
        ):
            return None

        if (
            self.radiation_input_mode
            == 'dose_rate_usv_h'
        ):
            return value

        normalized = (
            self.common_cost_model.clamp01(
                value / self.radiation_input_max
            )
        )

        return (
            normalized
            * self.common_cost_model
            .radiation_reference_usv_h
        )

    @staticmethod
    def safe_ratio(numerator, denominator):
        if denominator <= 0:
            return 0.0

        return float(numerator) / float(denominator)

    @staticmethod
    def optional_round(value, digits=6):
        if value is None:
            return None

        return round(float(value), digits)

    def path_callback(self, message):
        poses = message.poses

        quantisation = self.duplicate_tolerance

        path_signature = tuple(
            (
                int(round(
                    pose.pose.position.x
                    / quantisation
                )),
                int(round(
                    pose.pose.position.y
                    / quantisation
                ))
            )
            for pose in poses
        )

        if (
            self.deduplicate_paths
            and path_signature
            == self.last_path_signature
        ):
            self.get_logger().debug(
                'Duplicate path geometry ignored.'
            )
            return

        self.last_path_signature = path_signature

        if len(poses) < 2:
            metrics = self.empty_metrics(
                message,
                'Path must contain at least two poses.'
            )
            self.publish_metrics(metrics)
            return

        total_length = 0.0
        terrain_cost = 0.0

        estimated_traversal_time = 0.0
        radiation_cost = 0.0
        accumulated_dose = 0.0
        weighted_dose_rate_time = 0.0
        valid_radiation_time = 0.0
        max_dose_rate = None

        radiation_sample_count = 0
        invalid_radiation_samples = 0
        saturated_radiation_samples = 0

        sample_count = 0
        valid_samples = 0
        traversable_samples = 0
        invalid_samples = 0
        nontraversable_samples = 0

        valid_length = 0.0
        weighted_impedance = 0.0
        weighted_slope = 0.0
        weighted_roughness = 0.0

        max_impedance = None
        max_slope = None
        max_roughness = None

        for index in range(len(poses) - 1):
            first = poses[index].pose.position
            second = poses[index + 1].pose.position

            dx = float(second.x - first.x)
            dy = float(second.y - first.y)

            segment_length = math.hypot(dx, dy)

            if segment_length <= 1e-9:
                continue

            total_length += segment_length

            segment_samples = max(
                1,
                int(
                    math.ceil(
                        segment_length
                        / self.sample_step
                    )
                )
            )

            sample_length = (
                segment_length / segment_samples
            )

            for sample_index in range(segment_samples):
                interpolation = (
                    sample_index + 0.5
                ) / segment_samples

                x = first.x + interpolation * dx
                y = first.y + interpolation * dy

                result = self.query_cell(x, y)

                sample_count += 1

                if (
                    result is None
                    or not result['valid']
                ):
                    invalid_samples += 1
                    terrain_cost += (
                        self.invalid_cell_cost
                        * sample_length
                    )
                    continue

                valid_samples += 1
                valid_length += sample_length

                impedance = result['impedance']
                slope = result['slope']
                roughness = result['roughness']

                speed_m_s = (
                    self.common_cost_model
                    .estimate_speed_m_s(
                        impedance
                    )
                )

                sample_time_s = (
                    sample_length / speed_m_s
                )

                estimated_traversal_time += (
                    sample_time_s
                )

                radiation_value = (
                    self.query_radiation_value(
                        x,
                        y
                    )
                )

                dose_rate = (
                    self.radiation_value_to_dose_rate(
                        radiation_value
                    )
                )

                if dose_rate is None:
                    invalid_radiation_samples += 1
                else:
                    radiation_sample_count += 1
                    valid_radiation_time += (
                        sample_time_s
                    )
                    weighted_dose_rate_time += (
                        dose_rate * sample_time_s
                    )
                    accumulated_dose += (
                        dose_rate
                        * sample_time_s
                        / 3600.0
                    )

                    radiation_cell_cost = (
                        self.common_cost_model
                        .clamp01(
                            dose_rate
                            / self.common_cost_model
                            .radiation_reference_usv_h
                        )
                    )

                    radiation_cost += (
                        radiation_cell_cost
                        * sample_time_s
                        / self.common_cost_model
                        .reference_time_s
                    )

                    max_dose_rate = (
                        dose_rate
                        if max_dose_rate is None
                        else max(
                            max_dose_rate,
                            dose_rate
                        )
                    )

                    if (
                        self.radiation_input_mode
                        == 'normalized_occupancy'
                        and radiation_value
                        >= self.radiation_input_max
                    ):
                        saturated_radiation_samples += 1

                terrain_cost += (
                    impedance * sample_length
                )

                weighted_impedance += (
                    impedance * sample_length
                )
                weighted_slope += (
                    slope * sample_length
                )
                weighted_roughness += (
                    roughness * sample_length
                )

                max_impedance = (
                    impedance
                    if max_impedance is None
                    else max(max_impedance, impedance)
                )
                max_slope = (
                    slope
                    if max_slope is None
                    else max(max_slope, slope)
                )
                max_roughness = (
                    roughness
                    if max_roughness is None
                    else max(max_roughness, roughness)
                )

                if result['traversable']:
                    traversable_samples += 1
                else:
                    nontraversable_samples += 1

        if valid_length > 0.0:
            mean_impedance = (
                weighted_impedance / valid_length
            )
            mean_slope = (
                weighted_slope / valid_length
            )
            mean_roughness = (
                weighted_roughness / valid_length
            )
        else:
            mean_impedance = None
            mean_slope = None
            mean_roughness = None

        if valid_radiation_time > 0.0:
            mean_dose_rate = (
                weighted_dose_rate_time
                / valid_radiation_time
            )
        else:
            mean_dose_rate = None

        path_valid = (
            sample_count > 0
            and invalid_samples == 0
        )

        fully_traversable = (
            path_valid
            and nontraversable_samples == 0
        )

        if sample_count == 0:
            status = 'NO_NONZERO_SEGMENTS'
        elif invalid_samples > 0:
            status = 'PATH_CONTAINS_INVALID_OR_OUTSIDE_CELLS'
        elif nontraversable_samples > 0:
            status = 'PATH_CONTAINS_NONTRAVERSABLE_CELLS'
        elif self.radiation_map is None:
            status = 'RADIATION_MAP_NOT_RECEIVED'
        elif invalid_radiation_samples > 0:
            status = 'PATH_CONTAINS_INVALID_RADIATION_CELLS'
        else:
            status = 'OK'

        now = self.get_clock().now().nanoseconds / 1e9

        metrics = {
            'timestamp': round(now, 6),
            'terrain_level': self.terrain_level,
            'planner_name': self.planner_name,
            'path_topic': self.path_topic,
            'frame_id': message.header.frame_id,
            'path_points': len(poses),
            'sample_count': sample_count,
            'invalid_samples': invalid_samples,
            'nontraversable_samples': (
                nontraversable_samples
            ),
            'valid_sample_fraction': round(
                self.safe_ratio(
                    valid_samples,
                    sample_count
                ),
                6
            ),
            'traversable_sample_fraction': round(
                self.safe_ratio(
                    traversable_samples,
                    sample_count
                ),
                6
            ),
            'path_length_m': round(
                total_length,
                6
            ),
            'radiation_sample_count': (
                radiation_sample_count
            ),
            'invalid_radiation_samples': (
                invalid_radiation_samples
            ),
            'saturated_radiation_samples': (
                saturated_radiation_samples
            ),
            'valid_radiation_sample_fraction': round(
                self.safe_ratio(
                    radiation_sample_count,
                    sample_count
                ),
                6
            ),
            'estimated_traversal_time_s': round(
                estimated_traversal_time,
                6
            ),
            'mean_dose_rate_usv_h': (
                self.optional_round(
                    mean_dose_rate
                )
            ),
            'max_dose_rate_usv_h': (
                self.optional_round(
                    max_dose_rate
                )
            ),
            'accumulated_dose_usv': round(
                accumulated_dose,
                9
            ),
            'radiation_cost_integral': round(
                radiation_cost,
                9
            ),
            'terrain_cost_integral': round(
                terrain_cost,
                6
            ),
            'mean_impedance': self.optional_round(
                mean_impedance
            ),
            'max_impedance': self.optional_round(
                max_impedance
            ),
            'mean_slope_deg': self.optional_round(
                mean_slope
            ),
            'max_slope_deg': self.optional_round(
                max_slope
            ),
            'mean_roughness_m': self.optional_round(
                mean_roughness
            ),
            'max_roughness_m': self.optional_round(
                max_roughness
            ),
            'path_valid': path_valid,
            'fully_traversable': fully_traversable,
            'status': status,
        }

        self.publish_metrics(metrics)

        self.get_logger().info(
            f'Planner={self.planner_name}, '
            f'length={total_length:.3f} m, '
            f'terrain_cost={terrain_cost:.3f}, '
            f'dose={accumulated_dose:.6f} uSv, '
            f'radiation_cost={radiation_cost:.6f}, '
            f'invalid={invalid_samples}, '
            f'nontraversable={nontraversable_samples}, '
            f'status={status}'
        )

    def empty_metrics(self, message, status):
        now = self.get_clock().now().nanoseconds / 1e9

        return {
            'timestamp': round(now, 6),
            'terrain_level': self.terrain_level,
            'planner_name': self.planner_name,
            'path_topic': self.path_topic,
            'frame_id': message.header.frame_id,
            'path_points': len(message.poses),
            'sample_count': 0,
            'invalid_samples': 0,
            'nontraversable_samples': 0,
            'valid_sample_fraction': 0.0,
            'traversable_sample_fraction': 0.0,
            'path_length_m': 0.0,
            'radiation_sample_count': 0,
            'invalid_radiation_samples': 0,
            'saturated_radiation_samples': 0,
            'valid_radiation_sample_fraction': 0.0,
            'estimated_traversal_time_s': 0.0,
            'mean_dose_rate_usv_h': None,
            'max_dose_rate_usv_h': None,
            'accumulated_dose_usv': 0.0,
            'radiation_cost_integral': 0.0,
            'terrain_cost_integral': 0.0,
            'mean_impedance': None,
            'max_impedance': None,
            'mean_slope_deg': None,
            'max_slope_deg': None,
            'mean_roughness_m': None,
            'max_roughness_m': None,
            'path_valid': False,
            'fully_traversable': False,
            'status': status,
        }

    def publish_metrics(self, metrics):
        message = String()
        message.data = json.dumps(
            metrics,
            ensure_ascii=False,
            sort_keys=True
        )

        self.metrics_publisher.publish(message)
        self.append_csv(metrics)

    def append_csv(self, metrics):
        if self.csv_path is None:
            return

        write_header = (
            not self.csv_path.exists()
            or self.csv_path.stat().st_size == 0
        )

        with self.csv_path.open(
            'a',
            newline='',
            encoding='utf-8'
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=self.CSV_FIELDS
            )

            if write_header:
                writer.writeheader()

            writer.writerow(metrics)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = TerrainPathEvaluator()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
