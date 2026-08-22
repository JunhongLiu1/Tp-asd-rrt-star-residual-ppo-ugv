#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField


class Terrain3DImpedanceMapNode(Node):

    REQUIRED_KEYS = (
        'elevation_m',
        'slope_deg',
        'roughness_m',
        'terrain_impedance',
        'validity_mask',
        'traversability_mask',
    )

    def __init__(self):
        super().__init__('terrain_3d_impedance_map_node')

        self.declare_parameter('terrain_level', 'hard')

        self.declare_parameter(
            'data_directory',
            '/home/i/terrain_radiation_ws/src/'
            'radiation_mapping/dem/processed'
        )

        self.declare_parameter(
            'metadata_topic',
            '/terrain_impedance_map'
        )

        self.declare_parameter(
            'output_topic',
            '/terrain_3d_impedance_points'
        )

        self.declare_parameter(
            'publish_rate_hz',
            0.5
        )

        # Radius used only for the diagnostic local-height-range layer.
        # This is NOT yet the final Husky clearance model.
        self.declare_parameter(
            'height_window_radius_m',
            0.30
        )

        self.declare_parameter(
            'z_offset',
            0.0
        )

        self.terrain_level = str(
            self.get_parameter('terrain_level').value
        ).strip().lower()

        self.data_directory = Path(
            str(self.get_parameter('data_directory').value)
        ).expanduser()

        self.metadata_topic = str(
            self.get_parameter('metadata_topic').value
        )

        self.output_topic = str(
            self.get_parameter('output_topic').value
        )

        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )

        self.height_window_radius_m = float(
            self.get_parameter(
                'height_window_radius_m'
            ).value
        )

        self.z_offset = float(
            self.get_parameter('z_offset').value
        )

        if self.terrain_level not in (
            'easy',
            'medium',
            'hard',
        ):
            raise ValueError(
                'terrain_level must be easy, medium, or hard'
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError(
                'publish_rate_hz must be > 0'
            )

        self.layers = self.load_layers()

        self.cloud_data = None
        self.cloud_fields = None
        self.cloud_width = 0
        self.cloud_point_step = 0
        self.cloud_frame_id = 'map'

        qos = QoSProfile(depth=1)
        qos.reliability = (
            QoSReliabilityPolicy.RELIABLE
        )
        qos.durability = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.metadata_topic,
            self.map_callback,
            qos,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_cloud,
        )

        self.get_logger().info(
            'Terrain 3D impedance map node started.'
        )

        self.get_logger().info(
            f'Terrain level: {self.terrain_level}'
        )

        self.get_logger().info(
            f'Waiting for map metadata on '
            f'{self.metadata_topic} ...'
        )

    def load_layers(self):
        path = (
            self.data_directory /
            f'terrain_layers_{self.terrain_level}.npz'
        )

        if not path.exists():
            raise FileNotFoundError(
                f'Terrain NPZ not found: {path}'
            )

        self.get_logger().info(
            f'Loading terrain layers: {path}'
        )

        with np.load(
            str(path),
            allow_pickle=False
        ) as data:

            missing = [
                key
                for key in self.REQUIRED_KEYS
                if key not in data.files
            ]

            if missing:
                raise KeyError(
                    f'Missing NPZ layers: {missing}'
                )

            layers = {
                key: np.array(
                    data[key],
                    copy=True,
                )
                for key in self.REQUIRED_KEYS
            }

        reference_shape = layers[
            'elevation_m'
        ].shape

        for key, array in layers.items():
            if array.shape != reference_shape:
                raise ValueError(
                    f'Layer {key} shape '
                    f'{array.shape} != '
                    f'{reference_shape}'
                )

        elevation = layers['elevation_m']

        self.get_logger().info(
            'NPZ loaded: '
            f'shape={reference_shape}, '
            f'elevation='
            f'{float(np.min(elevation)):.3f}..'
            f'{float(np.max(elevation)):.3f} m'
        )

        return layers

    @staticmethod
    def build_planning_cost(
        impedance,
        validity,
        traversability,
    ):
        cost = np.clip(
            impedance.astype(np.float32) * 100.0,
            0.0,
            100.0,
        )

        blocked = (
            (validity == 0) |
            (traversability == 0)
        )

        cost[blocked] = 100.0

        return cost

    @staticmethod
    def orientation_error(
        candidate,
        occupancy,
    ):
        occupancy = occupancy.astype(
            np.float32
        )

        valid = occupancy >= 0

        if not np.any(valid):
            return float('inf')

        difference = np.abs(
            candidate[valid] -
            occupancy[valid]
        )

        return float(
            np.mean(difference)
        )

    def determine_orientation(
        self,
        occupancy_array,
    ):
        cost = self.build_planning_cost(
            self.layers['terrain_impedance'],
            self.layers['validity_mask'],
            self.layers['traversability_mask'],
        )

        direct_error = self.orientation_error(
            cost,
            occupancy_array,
        )

        flipped_error = self.orientation_error(
            np.flipud(cost),
            occupancy_array,
        )

        self.get_logger().info(
            'NPZ/map orientation check: '
            f'direct MAE={direct_error:.3f}, '
            f'flipped-Y MAE={flipped_error:.3f}'
        )

        use_flip = (
            flipped_error + 1e-6 <
            direct_error
        )

        if use_flip:
            self.get_logger().info(
                'Selected orientation: FLIPPED Y'
            )
        else:
            self.get_logger().info(
                'Selected orientation: DIRECT'
            )

        return use_flip

    @staticmethod
    def orient_layer(array, flip_y):
        if flip_y:
            return np.flipud(array).copy()

        return np.array(
            array,
            copy=True,
        )

    @staticmethod
    def compute_local_height_range(
        elevation,
        resolution,
        radius_m,
    ):
        radius_cells = max(
            1,
            int(
                math.ceil(
                    radius_m /
                    resolution
                )
            )
        )

        height, width = elevation.shape

        padded = np.pad(
            elevation.astype(np.float32),
            radius_cells,
            mode='edge',
        )

        local_min = np.full(
            (height, width),
            np.inf,
            dtype=np.float32,
        )

        local_max = np.full(
            (height, width),
            -np.inf,
            dtype=np.float32,
        )

        window_size = (
            2 * radius_cells + 1
        )

        for row_offset in range(
            window_size
        ):
            for col_offset in range(
                window_size
            ):
                view = padded[
                    row_offset:
                    row_offset + height,
                    col_offset:
                    col_offset + width,
                ]

                np.minimum(
                    local_min,
                    view,
                    out=local_min,
                )

                np.maximum(
                    local_max,
                    view,
                    out=local_max,
                )

        return (
            local_max - local_min
        ).astype(np.float32)

    def map_callback(self, msg):
        height = int(msg.info.height)
        width = int(msg.info.width)
        resolution = float(
            msg.info.resolution
        )

        expected_shape = (
            height,
            width,
        )

        npz_shape = self.layers[
            'elevation_m'
        ].shape

        if npz_shape != expected_shape:
            self.get_logger().error(
                'NPZ / OccupancyGrid shape '
                'mismatch: '
                f'NPZ={npz_shape}, '
                f'map={expected_shape}'
            )
            return

        # Do not rebuild the same cloud every
        # time the 2D map republishes.
        if self.cloud_data is not None:
            return

        occupancy = np.asarray(
            msg.data,
            dtype=np.float32,
        ).reshape(
            height,
            width,
        )

        flip_y = self.determine_orientation(
            occupancy
        )

        elevation = self.orient_layer(
            self.layers['elevation_m'],
            flip_y,
        ).astype(np.float32)

        slope = self.orient_layer(
            self.layers['slope_deg'],
            flip_y,
        ).astype(np.float32)

        roughness = self.orient_layer(
            self.layers['roughness_m'],
            flip_y,
        ).astype(np.float32)

        impedance = self.orient_layer(
            self.layers['terrain_impedance'],
            flip_y,
        ).astype(np.float32)

        validity = self.orient_layer(
            self.layers['validity_mask'],
            flip_y,
        ).astype(np.float32)

        traversability = self.orient_layer(
            self.layers[
                'traversability_mask'
            ],
            flip_y,
        ).astype(np.float32)

        planning_cost = (
            self.build_planning_cost(
                impedance,
                validity,
                traversability,
            )
        )

        height_range = (
            self.compute_local_height_range(
                elevation,
                resolution,
                self.height_window_radius_m,
            )
        )

        origin_x = float(
            msg.info.origin.position.x
        )

        origin_y = float(
            msg.info.origin.position.y
        )

        x_coordinates = (
            origin_x +
            (
                np.arange(
                    width,
                    dtype=np.float32
                ) + 0.5
            ) * resolution
        )

        y_coordinates = (
            origin_y +
            (
                np.arange(
                    height,
                    dtype=np.float32
                ) + 0.5
            ) * resolution
        )

        xx, yy = np.meshgrid(
            x_coordinates,
            y_coordinates,
        )

        zz = (
            elevation +
            np.float32(self.z_offset)
        )

        # All fields are float32 so the binary
        # PointCloud2 layout stays simple and
        # ROS2 Foxy-compatible.
        points = np.stack(
            (
                xx,
                yy,
                zz,
                planning_cost,
                impedance,
                slope,
                roughness,
                height_range,
                traversability,
                validity,
            ),
            axis=-1,
        ).astype(
            np.float32,
            copy=False,
        )

        # Flatten H x W into an unorganised
        # PointCloud2 of N points.
        points = np.ascontiguousarray(
            points.reshape(-1, 10)
        )

        field_names = (
            'x',
            'y',
            'z',
            'planning_cost',
            'terrain_impedance',
            'slope_deg',
            'roughness_m',
            'height_range_m',
            'traversable',
            'valid',
        )

        fields = []

        for index, name in enumerate(
            field_names
        ):
            fields.append(
                PointField(
                    name=name,
                    offset=index * 4,
                    datatype=PointField.FLOAT32,
                    count=1,
                )
            )

        self.cloud_data = points.tobytes()
        self.cloud_fields = fields
        self.cloud_width = points.shape[0]
        self.cloud_point_step = (
            points.shape[1] * 4
        )

        if msg.header.frame_id:
            self.cloud_frame_id = (
                msg.header.frame_id
            )
        else:
            self.cloud_frame_id = 'map'

        blocked_count = int(
            np.count_nonzero(
                planning_cost >= 100.0
            )
        )

        self.get_logger().info(
            '3D terrain cloud prepared: '
            f'{self.cloud_width} points'
        )

        self.get_logger().info(
            'Map geometry: '
            f'{width} x {height}, '
            f'resolution={resolution:.4f} m, '
            f'origin=({origin_x:.3f}, '
            f'{origin_y:.3f}), '
            f'frame={self.cloud_frame_id}'
        )

        self.get_logger().info(
            'Elevation: '
            f'{float(np.min(elevation)):.3f}..'
            f'{float(np.max(elevation)):.3f} m'
        )

        self.get_logger().info(
            'Local height range: '
            f'{float(np.min(height_range)):.3f}..'
            f'{float(np.max(height_range)):.3f} m'
        )

        self.get_logger().info(
            f'Blocked cells: '
            f'{blocked_count}/'
            f'{self.cloud_width}'
        )

        # Publish immediately instead of
        # waiting for the first timer tick.
        self.publish_cloud()

    def publish_cloud(self):
        if self.cloud_data is None:
            return

        msg = PointCloud2()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            self.cloud_frame_id
        )

        msg.height = 1
        msg.width = self.cloud_width

        msg.fields = self.cloud_fields

        msg.is_bigendian = False
        msg.point_step = (
            self.cloud_point_step
        )

        msg.row_step = (
            msg.point_step *
            msg.width
        )

        msg.is_dense = True
        msg.data = self.cloud_data

        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = Terrain3DImpedanceMapNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f'Fatal error: {exc}'
            )
        else:
            print(
                f'Fatal error: {exc}'
            )

        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
