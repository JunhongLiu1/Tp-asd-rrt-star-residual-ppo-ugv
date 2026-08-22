#!/usr/bin/env python3

import copy
import csv
import hashlib
import json
import math
import os
import time
from pathlib import Path as FilePath

import numpy as np

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


class HuskyVehicleAwareTerrainNode(Node):
    """Build central/full-chassis terrain-risk maps and inspect planned paths."""

    def __init__(self):
        super().__init__('husky_vehicle_aware_terrain_node')

        self.declare_parameter(
            'data_directory',
            '/home/i/terrain_radiation_ws/src/radiation_mapping/dem/processed',
        )
        self.declare_parameter('terrain_level', 'hard')
        self.declare_parameter('terrain_map_topic', '/terrain_impedance_map')

        # Geometry extracted from the active Husky Xacro.
        self.declare_parameter('wheelbase_m', 0.5120)
        self.declare_parameter('track_width_m', 0.5708)
        self.declare_parameter('wheel_length_m', 0.1143)
        self.declare_parameter('ground_clearance_m', 0.13228)
        self.declare_parameter('central_length_m', 0.5120)
        self.declare_parameter('central_width_m', 0.4565)
        self.declare_parameter('full_chassis_length_m', 0.9874)
        self.declare_parameter('full_chassis_width_m', 0.5709)
        self.declare_parameter('sample_spacing_m', 0.050)

        self.declare_parameter('clearance_warning_m', 0.050)
        self.declare_parameter('clearance_block_m', 0.000)
        self.declare_parameter('vehicle_risk_weight', 0.75)
        self.declare_parameter(
            'evaluation_yaws_deg',
            [0.0, 45.0, 90.0, 135.0],
        )

        self.declare_parameter(
            'path_topics',
            [
                '/asd_time_aware_rrt_star_path',
                '/rrt_star_baseline_path',
                '/rviz_asd_time_aware_rrt_star_path',
            ],
        )
        self.declare_parameter(
            'path_output_directory',
            '/home/i/terrain_radiation_ws/module36_vehicle_path_risk',
        )
        self.declare_parameter('path_sample_spacing_m', 0.100)
        self.declare_parameter('publish_rate_hz', 0.2)

        self.data_directory = str(self.get_parameter('data_directory').value)
        self.terrain_level = str(self.get_parameter('terrain_level').value)
        self.terrain_map_topic = str(
            self.get_parameter('terrain_map_topic').value
        )

        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.wheel_length = float(self.get_parameter('wheel_length_m').value)
        self.ground_clearance = float(
            self.get_parameter('ground_clearance_m').value
        )
        self.central_length = float(
            self.get_parameter('central_length_m').value
        )
        self.central_width = float(self.get_parameter('central_width_m').value)
        self.full_length = float(
            self.get_parameter('full_chassis_length_m').value
        )
        self.full_width = float(
            self.get_parameter('full_chassis_width_m').value
        )
        self.sample_spacing = float(
            self.get_parameter('sample_spacing_m').value
        )

        self.clearance_warning = float(
            self.get_parameter('clearance_warning_m').value
        )
        self.clearance_block = float(
            self.get_parameter('clearance_block_m').value
        )
        self.vehicle_risk_weight = float(
            self.get_parameter('vehicle_risk_weight').value
        )
        self.evaluation_yaws_deg = [
            float(value)
            for value in self.get_parameter('evaluation_yaws_deg').value
        ]

        self.path_topics = [
            str(value) for value in self.get_parameter('path_topics').value
        ]
        self.path_output_directory = os.path.expanduser(
            str(self.get_parameter('path_output_directory').value)
        )
        self.path_sample_spacing = float(
            self.get_parameter('path_sample_spacing_m').value
        )
        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )

        self._validate_parameters()
        self._load_npz()

        static_qos = QoSProfile(depth=1)
        static_qos.reliability = QoSReliabilityPolicy.RELIABLE
        static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.central_risk_pub = self.create_publisher(
            OccupancyGrid,
            '/husky_central_high_centering_risk_map',
            static_qos,
        )
        self.full_risk_pub = self.create_publisher(
            OccupancyGrid,
            '/husky_full_chassis_clearance_risk_map',
            static_qos,
        )
        self.combined_risk_pub = self.create_publisher(
            OccupancyGrid,
            '/husky_vehicle_collision_risk_map',
            static_qos,
        )
        self.vehicle_map_pub = self.create_publisher(
            OccupancyGrid,
            '/husky_vehicle_impedance_map',
            static_qos,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/husky_vehicle_terrain_points',
            static_qos,
        )

        self.path_summary_pub = self.create_publisher(
            String,
            '/husky_vehicle_path_risk_summary',
            10,
        )
        self.path_marker_pub = self.create_publisher(
            MarkerArray,
            '/husky_vehicle_path_risk_markers',
            10,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.terrain_map_topic,
            self.map_callback,
            static_qos,
        )

        self.path_subscriptions = []
        for topic in self.path_topics:
            subscription = self.create_subscription(
                Path,
                topic,
                lambda msg, path_topic=topic: self.path_callback(
                    msg,
                    path_topic,
                ),
                static_qos,
            )
            self.path_subscriptions.append(subscription)

        self.prepared = False
        self.pending_paths = {}
        self.last_path_signatures = {}
        self.publish_timer = None

        self.central_map_msg = None
        self.full_map_msg = None
        self.combined_map_msg = None
        self.vehicle_map_msg = None
        self.cloud_msg = None

        self.get_logger().info('Husky vehicle-aware terrain node started.')
        self.get_logger().info(
            f'Wheels: wheelbase={self.wheelbase:.4f} m, '
            f'track={self.track_width:.4f} m, '
            f'clearance={self.ground_clearance:.5f} m'
        )
        self.get_logger().info(
            f'Central region={self.central_length:.4f} x '
            f'{self.central_width:.4f} m; '
            f'full chassis={self.full_length:.4f} x '
            f'{self.full_width:.4f} m'
        )
        self.get_logger().info(
            f'Path topics: {self.path_topics}'
        )
        self.get_logger().info(
            f'Waiting for {self.terrain_map_topic} ...'
        )

    def _validate_parameters(self):
        positive = {
            'wheelbase_m': self.wheelbase,
            'track_width_m': self.track_width,
            'wheel_length_m': self.wheel_length,
            'ground_clearance_m': self.ground_clearance,
            'central_length_m': self.central_length,
            'central_width_m': self.central_width,
            'full_chassis_length_m': self.full_length,
            'full_chassis_width_m': self.full_width,
            'sample_spacing_m': self.sample_spacing,
            'path_sample_spacing_m': self.path_sample_spacing,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero.')

        if self.clearance_warning <= self.clearance_block:
            raise ValueError(
                'clearance_warning_m must be greater than clearance_block_m.'
            )
        if not 0.0 <= self.vehicle_risk_weight <= 1.0:
            raise ValueError('vehicle_risk_weight must be in [0, 1].')
        if not self.evaluation_yaws_deg:
            raise ValueError('evaluation_yaws_deg must not be empty.')

    def _load_npz(self):
        filename = f'terrain_layers_{self.terrain_level}.npz'
        self.npz_path = os.path.join(self.data_directory, filename)
        self.get_logger().info(f'Loading terrain layers: {self.npz_path}')

        if not os.path.isfile(self.npz_path):
            raise FileNotFoundError(
                f'Terrain NPZ file not found: {self.npz_path}'
            )

        data = np.load(self.npz_path)
        required = [
            'elevation_m',
            'terrain_impedance',
            'validity_mask',
            'traversability_mask',
        ]
        for key in required:
            if key not in data:
                raise KeyError(f'Missing NPZ layer: {key}')

        self.raw_elevation = np.asarray(data['elevation_m'], dtype=np.float32)
        self.raw_impedance = np.asarray(
            data['terrain_impedance'],
            dtype=np.float32,
        )
        self.raw_validity = np.asarray(
            data['validity_mask'],
            dtype=np.float32,
        )
        self.raw_traversability = np.asarray(
            data['traversability_mask'],
            dtype=np.float32,
        )

        self.get_logger().info(
            f'NPZ loaded: shape={self.raw_elevation.shape}, '
            f'elevation={np.nanmin(self.raw_elevation):.3f}..'
            f'{np.nanmax(self.raw_elevation):.3f} m'
        )

    @staticmethod
    def _npz_planning_cost(impedance, validity, traversability):
        cost = np.rint(100.0 * np.clip(impedance, 0.0, 1.0)).astype(
            np.float32
        )
        blocked = (
            (validity < 0.5)
            | (traversability < 0.5)
            | ~np.isfinite(impedance)
        )
        cost[blocked] = 100.0
        return cost

    def map_callback(self, msg):
        if self.prepared:
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            self.get_logger().error('Received an empty terrain map.')
            return
        if self.raw_elevation.shape != (height, width):
            self.get_logger().error(
                f'NPZ/map shape mismatch: NPZ={self.raw_elevation.shape}, '
                f'map={(height, width)}'
            )
            return

        map_cost_raw = np.asarray(msg.data, dtype=np.int16).reshape(
            height,
            width,
        )
        map_known = map_cost_raw >= 0
        map_cost = np.where(
            map_known,
            np.clip(map_cost_raw, 0, 100),
            100,
        ).astype(np.float32)

        direct_cost = self._npz_planning_cost(
            self.raw_impedance,
            self.raw_validity,
            self.raw_traversability,
        )
        flipped_cost = np.flipud(direct_cost)

        direct_mae = float(
            np.mean(np.abs(direct_cost[map_known] - map_cost[map_known]))
        )
        flipped_mae = float(
            np.mean(np.abs(flipped_cost[map_known] - map_cost[map_known]))
        )
        use_flip = flipped_mae < direct_mae

        self.get_logger().info(
            'NPZ/map orientation check: '
            f'direct MAE={direct_mae:.3f}, '
            f'flipped-Y MAE={flipped_mae:.3f}'
        )
        self.get_logger().info(
            'Selected orientation: '
            + ('FLIPPED Y' if use_flip else 'DIRECT')
        )

        if use_flip:
            self.elevation = np.flipud(self.raw_elevation).copy()
            self.validity = np.flipud(self.raw_validity).copy()
            self.traversability = np.flipud(
                self.raw_traversability
            ).copy()
        else:
            self.elevation = self.raw_elevation.copy()
            self.validity = self.raw_validity.copy()
            self.traversability = self.raw_traversability.copy()

        self.width = width
        self.height = height
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.frame_id = msg.header.frame_id or 'map'
        self.base_cost = map_cost
        self.map_known = map_known

        self.usable_grid = (
            map_known
            & np.isfinite(self.elevation)
            & (self.validity >= 0.5)
            & (self.traversability >= 0.5)
            & (self.base_cost < 100.0)
        )

        columns = np.arange(self.width, dtype=np.float32)
        rows = np.arange(self.height, dtype=np.float32)
        grid_x, grid_y = np.meshgrid(
            self.origin_x + (columns + 0.5) * self.resolution,
            self.origin_y + (rows + 0.5) * self.resolution,
        )
        self.centre_x = grid_x.reshape(-1).astype(np.float32)
        self.centre_y = grid_y.reshape(-1).astype(np.float32)
        self.centre_z = self.elevation.reshape(-1).astype(np.float32)
        self.centre_usable = self.usable_grid.reshape(-1)

        start = time.perf_counter()
        central = self._calculate_region_map(
            self.central_length,
            self.central_width,
            'central',
        )
        full = self._calculate_region_map(
            self.full_length,
            self.full_width,
            'full chassis',
        )
        combined = self._combine_region_maps(central, full)
        elapsed = time.perf_counter() - start

        self._prepare_messages(msg, central, full, combined)
        self.prepared = True
        self.publish_all()

        if self.publish_rate_hz > 0.0:
            self.publish_timer = self.create_timer(
                1.0 / self.publish_rate_hz,
                self.publish_all,
            )

        self.get_logger().info(
            f'Central + full-chassis calculation finished in '
            f'{elapsed:.2f} seconds.'
        )
        self.get_logger().info(
            f'Map geometry: {self.width} x {self.height}, '
            f'resolution={self.resolution:.4f} m, '
            f'origin=({self.origin_x:.3f}, {self.origin_y:.3f})'
        )
        self._log_region_statistics('Central', central)
        self._log_region_statistics('Full chassis', full)

        combined_warning = int(
            np.count_nonzero(
                combined['assessable']
                & (combined['minimum_clearance'] > self.clearance_block)
                & (combined['minimum_clearance'] < self.clearance_warning)
            )
        )
        combined_contact = int(
            np.count_nonzero(
                combined['assessable']
                & (combined['minimum_clearance'] <= self.clearance_block)
            )
        )
        self.get_logger().info(
            f'Combined valid warning cells: {combined_warning}; '
            f'predicted contact cells: {combined_contact}'
        )

        for topic, pending_msg in list(self.pending_paths.items()):
            self._evaluate_path(pending_msg, topic)
        self.pending_paths.clear()

    def _sample_grid_bilinear(self, grid, x_world, y_world):
        x_world = np.asarray(x_world, dtype=np.float64)
        y_world = np.asarray(y_world, dtype=np.float64)

        gx = (x_world - self.origin_x) / self.resolution - 0.5
        gy = (y_world - self.origin_y) / self.resolution - 0.5

        inside = (
            (gx >= 0.0)
            & (gx <= self.width - 1)
            & (gy >= 0.0)
            & (gy <= self.height - 1)
        )

        x0 = np.clip(np.floor(gx).astype(np.int32), 0, self.width - 1)
        y0 = np.clip(np.floor(gy).astype(np.int32), 0, self.height - 1)
        x1 = np.minimum(x0 + 1, self.width - 1)
        y1 = np.minimum(y0 + 1, self.height - 1)

        tx = np.clip(gx - x0, 0.0, 1.0)
        ty = np.clip(gy - y0, 0.0, 1.0)

        z00 = grid[y0, x0]
        z10 = grid[y0, x1]
        z01 = grid[y1, x0]
        z11 = grid[y1, x1]

        value = (
            (1.0 - tx) * (1.0 - ty) * z00
            + tx * (1.0 - ty) * z10
            + (1.0 - tx) * ty * z01
            + tx * ty * z11
        )

        nearest_x = np.clip(
            np.floor(gx + 0.5).astype(np.int32),
            0,
            self.width - 1,
        )
        nearest_y = np.clip(
            np.floor(gy + 0.5).astype(np.int32),
            0,
            self.height - 1,
        )

        valid = (
            inside
            & self.usable_grid[nearest_y, nearest_x]
            & np.isfinite(value)
        )
        value = np.where(valid, value, np.nan).astype(np.float32)
        return value, valid

    @staticmethod
    def _sample_offsets(length, spacing):
        count = max(3, int(math.ceil(length / spacing)) + 1)
        return np.linspace(
            -0.5 * length,
            0.5 * length,
            count,
            dtype=np.float32,
        )

    def _evaluate_yaw(self, yaw_deg, region_length, region_width):
        yaw = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        half_wheelbase = 0.5 * self.wheelbase
        half_track = 0.5 * self.track_width
        wheel_u = np.asarray(
            [half_wheelbase, half_wheelbase, -half_wheelbase, -half_wheelbase],
            dtype=np.float32,
        )
        wheel_v = np.asarray(
            [half_track, -half_track, half_track, -half_track],
            dtype=np.float32,
        )

        count = self.centre_x.size
        wheel_z = np.zeros((4, count), dtype=np.float32)
        wheel_valid = np.ones((4, count), dtype=bool)

        for index in range(4):
            dx = cos_yaw * wheel_u[index] - sin_yaw * wheel_v[index]
            dy = sin_yaw * wheel_u[index] + cos_yaw * wheel_v[index]
            sampled_z, sampled_valid = self._sample_grid_bilinear(
                self.elevation,
                self.centre_x + dx,
                self.centre_y + dy,
            )
            wheel_z[index] = np.nan_to_num(sampled_z, nan=0.0)
            wheel_valid[index] = sampled_valid

        support_valid = self.centre_usable & np.all(wheel_valid, axis=0)

        design = np.column_stack(
            (wheel_u, wheel_v, np.ones(4, dtype=np.float32))
        ).astype(np.float64)
        coefficients = np.linalg.pinv(design) @ wheel_z.astype(np.float64)
        plane_a = coefficients[0]
        plane_b = coefficients[1]
        plane_c = coefficients[2]

        minimum_clearance = np.full(count, np.inf, dtype=np.float64)
        region_valid = np.ones(count, dtype=bool)

        for local_u in self._sample_offsets(region_length, self.sample_spacing):
            for local_v in self._sample_offsets(
                region_width,
                self.sample_spacing,
            ):
                dx = cos_yaw * local_u - sin_yaw * local_v
                dy = sin_yaw * local_u + cos_yaw * local_v
                terrain_z, sample_valid = self._sample_grid_bilinear(
                    self.elevation,
                    self.centre_x + dx,
                    self.centre_y + dy,
                )
                underside_z = (
                    plane_a * local_u
                    + plane_b * local_v
                    + plane_c
                    + self.ground_clearance
                )
                clearance = underside_z - terrain_z.astype(np.float64)
                minimum_clearance = np.minimum(
                    minimum_clearance,
                    np.where(sample_valid, clearance, np.inf),
                )
                region_valid &= sample_valid

        assessable = (
            support_valid
            & region_valid
            & np.isfinite(minimum_clearance)
        )
        minimum_clearance = np.where(
            assessable,
            minimum_clearance,
            np.nan,
        )
        risk = self._clearance_to_risk(minimum_clearance, assessable)
        traversable = assessable & (
            minimum_clearance > self.clearance_block
        )

        return {
            'risk': risk,
            'minimum_clearance': minimum_clearance.astype(np.float32),
            'assessable': assessable,
            'traversable': traversable,
            'pitch_deg': np.degrees(np.arctan(plane_a)).astype(np.float32),
            'roll_deg': np.degrees(np.arctan(plane_b)).astype(np.float32),
        }

    def _clearance_to_risk(self, clearance, assessable):
        denominator = max(
            self.clearance_warning - self.clearance_block,
            1e-6,
        )
        risk = np.clip(
            (self.clearance_warning - clearance) / denominator,
            0.0,
            1.0,
        )
        return np.where(assessable, risk, np.nan).astype(np.float32)

    def _calculate_region_map(self, length, width, label):
        count = self.centre_x.size
        worst_risk = np.full(count, -np.inf, dtype=np.float32)
        worst_clearance = np.full(count, np.inf, dtype=np.float32)
        worst_yaw = np.full(count, -1.0, dtype=np.float32)
        worst_pitch = np.zeros(count, dtype=np.float32)
        worst_roll = np.zeros(count, dtype=np.float32)
        all_yaws_assessable = np.ones(count, dtype=bool)
        all_yaws_traversable = np.ones(count, dtype=bool)

        for yaw_deg in self.evaluation_yaws_deg:
            result = self._evaluate_yaw(yaw_deg, length, width)
            all_yaws_assessable &= result['assessable']
            all_yaws_traversable &= result['traversable']

            update = result['assessable'] & (
                (result['risk'] > worst_risk)
                | (
                    np.isclose(result['risk'], worst_risk, atol=1e-6)
                    & (result['minimum_clearance'] < worst_clearance)
                )
            )
            worst_risk[update] = result['risk'][update]
            worst_clearance[update] = result['minimum_clearance'][update]
            worst_yaw[update] = yaw_deg
            worst_pitch[update] = result['pitch_deg'][update]
            worst_roll[update] = result['roll_deg'][update]

            valid_clearance = result['minimum_clearance'][
                result['assessable']
            ]
            minimum_text = (
                f'{np.min(valid_clearance):.3f} m'
                if valid_clearance.size
                else 'none'
            )
            warning_count = int(
                np.count_nonzero(
                    result['assessable']
                    & (result['minimum_clearance'] > self.clearance_block)
                    & (result['minimum_clearance'] < self.clearance_warning)
                )
            )
            self.get_logger().info(
                f'{label}, yaw {yaw_deg:6.1f} deg: '
                f'min clearance={minimum_text}, '
                f'valid warning cells={warning_count}'
            )

        assessable = all_yaws_assessable & self.centre_usable
        traversable = assessable & all_yaws_traversable
        worst_risk[~assessable] = np.nan
        worst_clearance[~assessable] = np.nan
        worst_yaw[~assessable] = -1.0

        return {
            'risk': worst_risk,
            'minimum_clearance': worst_clearance,
            'worst_yaw_deg': worst_yaw,
            'pitch_deg': worst_pitch,
            'roll_deg': worst_roll,
            'assessable': assessable,
            'traversable': traversable,
        }

    def _combine_region_maps(self, central, full):
        assessable = central['assessable'] & full['assessable']
        combined_risk = np.full(self.centre_x.size, np.nan, dtype=np.float32)
        combined_clearance = np.full(
            self.centre_x.size,
            np.nan,
            dtype=np.float32,
        )
        combined_risk[assessable] = np.maximum(
            central['risk'][assessable],
            full['risk'][assessable],
        )
        combined_clearance[assessable] = np.minimum(
            central['minimum_clearance'][assessable],
            full['minimum_clearance'][assessable],
        )
        traversable = (
            assessable
            & central['traversable']
            & full['traversable']
        )

        base_flat = self.base_cost.reshape(-1).astype(np.float32)
        vehicle_cost = np.full(self.centre_x.size, -1.0, dtype=np.float32)
        original_blocked = self.map_known.reshape(-1) & ~self.centre_usable
        vehicle_cost[original_blocked] = 100.0
        vehicle_cost[assessable] = np.clip(
            base_flat[assessable]
            + 100.0
            * self.vehicle_risk_weight
            * combined_risk[assessable],
            0.0,
            100.0,
        )
        vehicle_cost[assessable & ~traversable] = 100.0

        return {
            'risk': combined_risk,
            'minimum_clearance': combined_clearance,
            'assessable': assessable,
            'traversable': traversable,
            'vehicle_cost': vehicle_cost,
        }

    def _prepare_messages(self, source_map, central, full, combined):
        self.central_map_msg = self._make_risk_map(source_map, central['risk'])
        self.full_map_msg = self._make_risk_map(source_map, full['risk'])
        self.combined_map_msg = self._make_risk_map(
            source_map,
            combined['risk'],
        )

        vehicle_values = np.rint(combined['vehicle_cost']).astype(np.int16)
        self.vehicle_map_msg = OccupancyGrid()
        self.vehicle_map_msg.header.frame_id = self.frame_id
        self.vehicle_map_msg.info = copy.deepcopy(source_map.info)
        self.vehicle_map_msg.data = vehicle_values.tolist()

        field_names = [
            'x',
            'y',
            'z',
            'base_cost',
            'central_risk',
            'central_min_clearance_m',
            'central_worst_yaw_deg',
            'full_risk',
            'full_min_clearance_m',
            'full_worst_yaw_deg',
            'combined_risk',
            'vehicle_cost',
            'assessable',
            'traversable',
        ]
        dtype = np.dtype([(name, '<f4') for name in field_names], align=False)
        array = np.empty(self.centre_x.size, dtype=dtype)
        array['x'] = self.centre_x
        array['y'] = self.centre_y
        array['z'] = self.centre_z
        array['base_cost'] = self.base_cost.reshape(-1)
        array['central_risk'] = np.nan_to_num(central['risk'], nan=-1.0)
        array['central_min_clearance_m'] = np.nan_to_num(
            central['minimum_clearance'],
            nan=-1.0,
        )
        array['central_worst_yaw_deg'] = central['worst_yaw_deg']
        array['full_risk'] = np.nan_to_num(full['risk'], nan=-1.0)
        array['full_min_clearance_m'] = np.nan_to_num(
            full['minimum_clearance'],
            nan=-1.0,
        )
        array['full_worst_yaw_deg'] = full['worst_yaw_deg']
        array['combined_risk'] = np.nan_to_num(combined['risk'], nan=-1.0)
        array['vehicle_cost'] = combined['vehicle_cost']
        array['assessable'] = combined['assessable'].astype(np.float32)
        array['traversable'] = combined['traversable'].astype(np.float32)

        self.cloud_msg = PointCloud2()
        self.cloud_msg.header.frame_id = self.frame_id
        self.cloud_msg.height = 1
        self.cloud_msg.width = int(array.size)
        self.cloud_msg.fields = [
            PointField(
                name=name,
                offset=int(dtype.fields[name][1]),
                datatype=PointField.FLOAT32,
                count=1,
            )
            for name in field_names
        ]
        self.cloud_msg.is_bigendian = False
        self.cloud_msg.point_step = int(dtype.itemsize)
        self.cloud_msg.row_step = self.cloud_msg.point_step * self.cloud_msg.width
        self.cloud_msg.data = array.tobytes()
        self.cloud_msg.is_dense = True

        self.get_logger().info(
            f'Combined vehicle point cloud prepared: {array.size} points, '
            f'point_step={self.cloud_msg.point_step} bytes'
        )

    def _make_risk_map(self, source_map, risk):
        values = np.full(risk.size, -1, dtype=np.int16)
        valid = np.isfinite(risk)
        values[valid] = np.rint(
            100.0 * np.clip(risk[valid], 0.0, 1.0)
        ).astype(np.int16)

        message = OccupancyGrid()
        message.header.frame_id = self.frame_id
        message.info = copy.deepcopy(source_map.info)
        message.data = values.tolist()
        return message

    def _log_region_statistics(self, label, result):
        assessable = result['assessable']
        clearance = result['minimum_clearance']
        contact = assessable & (clearance <= self.clearance_block)
        warning = (
            assessable
            & (clearance > self.clearance_block)
            & (clearance < self.clearance_warning)
        )
        safe = assessable & (clearance >= self.clearance_warning)
        minimum = (
            float(np.nanmin(clearance[assessable]))
            if np.any(assessable)
            else float('nan')
        )
        self.get_logger().info(
            f'{label}: assessable={np.count_nonzero(assessable)}, '
            f'contact={np.count_nonzero(contact)}, '
            f'warning={np.count_nonzero(warning)}, '
            f'safe={np.count_nonzero(safe)}, '
            f'minimum={minimum:.4f} m'
        )

    def publish_all(self):
        if not self.prepared:
            return
        stamp = self.get_clock().now().to_msg()
        for message in [
            self.central_map_msg,
            self.full_map_msg,
            self.combined_map_msg,
            self.vehicle_map_msg,
            self.cloud_msg,
        ]:
            message.header.stamp = stamp

        self.central_risk_pub.publish(self.central_map_msg)
        self.full_risk_pub.publish(self.full_map_msg)
        self.combined_risk_pub.publish(self.combined_map_msg)
        self.vehicle_map_pub.publish(self.vehicle_map_msg)
        self.cloud_pub.publish(self.cloud_msg)

    def path_callback(self, msg, topic):
        if not self.prepared:
            self.pending_paths[topic] = msg
            return
        self._evaluate_path(msg, topic)

    def _path_signature(self, msg):
        coordinates = np.asarray(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in msg.poses
            ],
            dtype='<f4',
        )
        return hashlib.sha1(coordinates.tobytes()).hexdigest()

    def _resample_path(self, points):
        if len(points) <= 1:
            return [(points[0][0], points[0][1], 0.0)] if points else []

        samples = []
        previous_yaw = 0.0
        for index in range(len(points) - 1):
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
            dx = x1 - x0
            dy = y1 - y0
            segment_length = math.hypot(dx, dy)
            if segment_length <= 1e-9:
                continue
            previous_yaw = math.atan2(dy, dx)
            steps = max(1, int(math.ceil(segment_length / self.path_sample_spacing)))
            for step in range(steps):
                ratio = step / steps
                samples.append(
                    (x0 + ratio * dx, y0 + ratio * dy, previous_yaw)
                )
        samples.append((points[-1][0], points[-1][1], previous_yaw))
        return samples

    def _evaluate_pose_region(self, x, y, yaw, length, width):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        half_wheelbase = 0.5 * self.wheelbase
        half_track = 0.5 * self.track_width
        wheel_u = np.asarray(
            [half_wheelbase, half_wheelbase, -half_wheelbase, -half_wheelbase],
            dtype=np.float64,
        )
        wheel_v = np.asarray(
            [half_track, -half_track, half_track, -half_track],
            dtype=np.float64,
        )

        wheel_z = np.zeros(4, dtype=np.float64)
        for index in range(4):
            dx = cos_yaw * wheel_u[index] - sin_yaw * wheel_v[index]
            dy = sin_yaw * wheel_u[index] + cos_yaw * wheel_v[index]
            sampled, valid = self._sample_grid_bilinear(
                self.elevation,
                np.asarray([x + dx]),
                np.asarray([y + dy]),
            )
            if not bool(valid[0]):
                return {'assessable': False}
            wheel_z[index] = float(sampled[0])

        design = np.column_stack((wheel_u, wheel_v, np.ones(4)))
        plane_a, plane_b, plane_c = np.linalg.pinv(design) @ wheel_z

        minimum_clearance = float('inf')
        for local_u in self._sample_offsets(length, self.sample_spacing):
            for local_v in self._sample_offsets(width, self.sample_spacing):
                dx = cos_yaw * local_u - sin_yaw * local_v
                dy = sin_yaw * local_u + cos_yaw * local_v
                terrain_z, valid = self._sample_grid_bilinear(
                    self.elevation,
                    np.asarray([x + dx]),
                    np.asarray([y + dy]),
                )
                if not bool(valid[0]):
                    return {'assessable': False}
                underside_z = (
                    plane_a * local_u
                    + plane_b * local_v
                    + plane_c
                    + self.ground_clearance
                )
                minimum_clearance = min(
                    minimum_clearance,
                    underside_z - float(terrain_z[0]),
                )

        risk = float(
            np.clip(
                (
                    self.clearance_warning
                    - minimum_clearance
                )
                / max(
                    self.clearance_warning - self.clearance_block,
                    1e-6,
                ),
                0.0,
                1.0,
            )
        )
        return {
            'assessable': True,
            'minimum_clearance': minimum_clearance,
            'risk': risk,
            'traversable': minimum_clearance > self.clearance_block,
            'pitch_deg': math.degrees(math.atan(plane_a)),
            'roll_deg': math.degrees(math.atan(plane_b)),
        }

    def _base_cost_at(self, x, y):
        column = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if not (0 <= column < self.width and 0 <= row < self.height):
            return None
        return float(self.base_cost[row, column])

    def _evaluate_path(self, msg, topic):
        if not msg.poses:
            self.get_logger().warning(f'Empty path received on {topic}.')
            return
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().error(
                f'Path {topic} is in frame {msg.header.frame_id}, '
                f'but terrain map is in {self.frame_id}. No TF conversion '
                f'is performed by this node.'
            )
            return

        signature = self._path_signature(msg)
        if self.last_path_signatures.get(topic) == signature:
            return
        self.last_path_signatures[topic] = signature

        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        samples = self._resample_path(points)
        rows = []

        for index, (x, y, yaw) in enumerate(samples):
            central = self._evaluate_pose_region(
                x,
                y,
                yaw,
                self.central_length,
                self.central_width,
            )
            full = self._evaluate_pose_region(
                x,
                y,
                yaw,
                self.full_length,
                self.full_width,
            )
            assessable = central.get('assessable', False) and full.get(
                'assessable',
                False,
            )
            base_cost = self._base_cost_at(x, y)

            if assessable:
                combined_risk = max(central['risk'], full['risk'])
                vehicle_cost = min(
                    100.0,
                    (base_cost if base_cost is not None else 100.0)
                    + 100.0 * self.vehicle_risk_weight * combined_risk,
                )
                traversable = central['traversable'] and full['traversable']
            else:
                combined_risk = float('nan')
                vehicle_cost = float('nan')
                traversable = False

            rows.append(
                {
                    'index': index,
                    'x': x,
                    'y': y,
                    'yaw_deg': math.degrees(yaw),
                    'base_cost': base_cost,
                    'central_clearance_m': central.get(
                        'minimum_clearance',
                        float('nan'),
                    ),
                    'central_risk': central.get('risk', float('nan')),
                    'full_clearance_m': full.get(
                        'minimum_clearance',
                        float('nan'),
                    ),
                    'full_risk': full.get('risk', float('nan')),
                    'combined_risk': combined_risk,
                    'vehicle_cost': vehicle_cost,
                    'assessable': assessable,
                    'traversable': traversable,
                }
            )

        summary = self._summarise_path(topic, rows)
        summary['source_pose_count'] = len(points)
        summary['sample_count'] = len(samples)
        csv_path = self._write_path_csv(topic, rows)
        summary['csv_path'] = csv_path

        message = String()
        message.data = json.dumps(summary, ensure_ascii=False, sort_keys=True)
        self.path_summary_pub.publish(message)
        self._publish_path_markers(topic, rows)

        self.get_logger().info(
            f'Path risk [{topic}]: source_poses={summary["source_pose_count"]}, '
            f'samples={summary["sample_count"]}, '
            f'length={summary["path_length_m"]:.3f} m, '
            f'central_min={summary["central_min_clearance_m"]:.4f} m, '
            f'full_min={summary["full_min_clearance_m"]:.4f} m, '
            f'max_combined_risk={summary["max_combined_risk"]:.3f}, '
            f'full_warning_poses={summary["full_warning_pose_count"]}, '
            f'contact_poses={summary["contact_pose_count"]}, '
            f'unassessable={summary["unassessable_pose_count"]}'
        )
        self.get_logger().info(
            f'Path risk CSV written: {csv_path}'
        )

    def _summarise_path(self, topic, rows):
        pose_count = len(rows)
        length = 0.0
        integrated_risk = 0.0
        for index in range(pose_count - 1):
            dx = rows[index + 1]['x'] - rows[index]['x']
            dy = rows[index + 1]['y'] - rows[index]['y']
            segment = math.hypot(dx, dy)
            length += segment
            r0 = rows[index]['combined_risk']
            r1 = rows[index + 1]['combined_risk']
            if math.isfinite(r0) and math.isfinite(r1):
                integrated_risk += 0.5 * (r0 + r1) * segment

        assessable_rows = [row for row in rows if row['assessable']]
        if assessable_rows:
            central_min = min(
                row['central_clearance_m'] for row in assessable_rows
            )
            full_min = min(row['full_clearance_m'] for row in assessable_rows)
            max_central_risk = max(
                row['central_risk'] for row in assessable_rows
            )
            max_full_risk = max(row['full_risk'] for row in assessable_rows)
            max_combined_risk = max(
                row['combined_risk'] for row in assessable_rows
            )
            worst_row = max(
                assessable_rows,
                key=lambda row: row['combined_risk'],
            )
        else:
            central_min = float('nan')
            full_min = float('nan')
            max_central_risk = float('nan')
            max_full_risk = float('nan')
            max_combined_risk = float('nan')
            worst_row = None

        central_warning = sum(
            row['assessable']
            and self.clearance_block < row['central_clearance_m'] < self.clearance_warning
            for row in rows
        )
        full_warning = sum(
            row['assessable']
            and self.clearance_block < row['full_clearance_m'] < self.clearance_warning
            for row in rows
        )
        contact_count = sum(
            row['assessable']
            and (
                row['central_clearance_m'] <= self.clearance_block
                or row['full_clearance_m'] <= self.clearance_block
            )
            for row in rows
        )
        unassessable = sum(not row['assessable'] for row in rows)

        summary = {
            'path_topic': topic,
            'pose_count': pose_count,
            'path_length_m': length,
            'central_min_clearance_m': central_min,
            'full_min_clearance_m': full_min,
            'max_central_risk': max_central_risk,
            'max_full_risk': max_full_risk,
            'max_combined_risk': max_combined_risk,
            'integrated_combined_risk_m': integrated_risk,
            'central_warning_pose_count': int(central_warning),
            'full_warning_pose_count': int(full_warning),
            'contact_pose_count': int(contact_count),
            'unassessable_pose_count': int(unassessable),
        }
        if worst_row is not None:
            summary.update(
                {
                    'worst_pose_index': worst_row['index'],
                    'worst_pose_x': worst_row['x'],
                    'worst_pose_y': worst_row['y'],
                    'worst_pose_yaw_deg': worst_row['yaw_deg'],
                    'worst_pose_central_clearance_m': worst_row[
                        'central_clearance_m'
                    ],
                    'worst_pose_full_clearance_m': worst_row[
                        'full_clearance_m'
                    ],
                    'worst_pose_combined_risk': worst_row[
                        'combined_risk'
                    ],
                }
            )
        return summary

    def _write_path_csv(self, topic, rows):
        output_dir = FilePath(self.path_output_directory)
        output_dir.mkdir(parents=True, exist_ok=True)
        safe_topic = topic.strip('/').replace('/', '_') or 'path'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        output_path = output_dir / f'{safe_topic}_{timestamp}.csv'

        fieldnames = list(rows[0].keys())
        with output_path.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return str(output_path)

    def _publish_path_markers(self, topic, rows):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = topic.strip('/').replace('/', '_') or 'path'
        marker.id = int(hashlib.sha1(topic.encode()).hexdigest()[:8], 16) % 2000000000
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.pose.orientation.w = 1.0

        for row in rows:
            point = Point()
            point.x = row['x']
            point.y = row['y']
            point.z = 0.08
            marker.points.append(point)

            color = ColorRGBA()
            color.a = 1.0
            if not row['assessable']:
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
            else:
                risk = float(np.clip(row['combined_risk'], 0.0, 1.0))
                color.r = risk
                color.g = 1.0 - risk
                color.b = 0.0
            marker.colors.append(color)

        marker_array.markers.append(marker)
        self.path_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HuskyVehicleAwareTerrainNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'ERROR: {error}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
