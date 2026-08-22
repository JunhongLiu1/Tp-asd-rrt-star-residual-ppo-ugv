#!/usr/bin/env python3

import json
from pathlib import Path

import numpy as np
import rclpy

from ament_index_python.packages import (
    get_package_share_directory
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class TerrainLayerPublisher(Node):
    """Publish DEM-derived terrain layers as OccupancyGrid maps."""

    VALID_LEVELS = (
        'easy',
        'medium',
        'hard',
    )

    def __init__(self):
        super().__init__('terrain_layer_publisher')

        self.declare_parameter(
            'terrain_level',
            'easy'
        )

        self.declare_parameter(
            'data_directory',
            ''
        )

        self.declare_parameter(
            'frame_id',
            'map'
        )

        self.declare_parameter(
            'publish_rate_hz',
            1.0
        )

        self.terrain_level = str(
            self.get_parameter(
                'terrain_level'
            ).value
        ).strip().lower()

        self.frame_id = str(
            self.get_parameter(
                'frame_id'
            ).value
        ).strip()

        data_directory = str(
            self.get_parameter(
                'data_directory'
            ).value
        ).strip()

        publish_rate_hz = float(
            self.get_parameter(
                'publish_rate_hz'
            ).value
        )

        if self.terrain_level not in self.VALID_LEVELS:
            raise RuntimeError(
                'terrain_level must be easy, medium, or hard'
            )

        if not self.frame_id:
            raise RuntimeError(
                'frame_id cannot be empty'
            )

        if publish_rate_hz <= 0.0:
            raise RuntimeError(
                'publish_rate_hz must be positive'
            )

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
                package_share
                / 'dem'
                / 'processed'
            )

        prefix = (
            self.data_directory
            / f'terrain_layers_{self.terrain_level}'
        )

        self.npz_path = Path(
            str(prefix) + '.npz'
        )

        self.metadata_path = Path(
            str(prefix) + '_metadata.json'
        )

        self._load_terrain_layers()

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.slope_publisher = self.create_publisher(
            OccupancyGrid,
            '/slope_map',
            map_qos
        )

        self.roughness_publisher = self.create_publisher(
            OccupancyGrid,
            '/roughness_map',
            map_qos
        )

        self.impedance_publisher = self.create_publisher(
            OccupancyGrid,
            '/terrain_impedance_map',
            map_qos
        )

        self.validity_publisher = self.create_publisher(
            OccupancyGrid,
            '/dem_validity_mask',
            map_qos
        )

        self.traversability_publisher = self.create_publisher(
            OccupancyGrid,
            '/terrain_traversability_mask',
            map_qos
        )

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_all
        )

        self.publish_all()

        self.get_logger().info(
            'Terrain layer publisher started'
        )

        self.get_logger().info(
            f'Terrain level: {self.terrain_level}'
        )

        self.get_logger().info(
            f'NPZ file: {self.npz_path}'
        )

        self.get_logger().info(
            f'Metadata file: {self.metadata_path}'
        )

        self.get_logger().info(
            f'Grid: {self.width} x {self.height}'
        )

        self.get_logger().info(
            f'Resolution: {self.resolution:.3f} m/cell'
        )

        self.get_logger().info(
            f'Origin: ({self.origin_x:.3f}, '
            f'{self.origin_y:.3f})'
        )

        self.get_logger().info(
            f'Frame: {self.frame_id}'
        )

    def _load_terrain_layers(self):
        if not self.npz_path.is_file():
            raise FileNotFoundError(
                f'Terrain NPZ file not found: '
                f'{self.npz_path}'
            )

        if not self.metadata_path.is_file():
            raise FileNotFoundError(
                f'Terrain metadata file not found: '
                f'{self.metadata_path}'
            )

        with self.metadata_path.open(
            'r',
            encoding='utf-8'
        ) as file:
            metadata = json.load(file)

        if 'grid' not in metadata:
            raise KeyError(
                'Terrain metadata does not contain grid'
            )

        if 'cost_model' not in metadata:
            raise KeyError(
                'Terrain metadata does not contain cost_model'
            )

        grid = metadata['grid']
        cost_model = metadata['cost_model']

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

        self.slope_limit_deg = float(
            cost_model['slope_cost_limit_deg']
        )

        self.roughness_limit_m = float(
            cost_model['roughness_cost_limit_m']
        )

        with np.load(
            self.npz_path
        ) as data:
            required_arrays = (
                'slope_deg',
                'roughness_m',
                'terrain_impedance',
                'validity_mask',
                'traversability_mask',
            )

            for array_name in required_arrays:
                if array_name not in data.files:
                    raise KeyError(
                        'Missing terrain array in NPZ: '
                        f'{array_name}'
                    )

            self.slope_deg = np.asarray(
                data['slope_deg'],
                dtype=np.float64
            )

            self.roughness_m = np.asarray(
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

        arrays = {
            'slope_deg': self.slope_deg,
            'roughness_m': self.roughness_m,
            'terrain_impedance': self.impedance,
            'validity_mask': self.validity,
            'traversability_mask': self.traversability,
        }

        for name, array in arrays.items():
            if array.shape != expected_shape:
                raise RuntimeError(
                    f'{name} has shape {array.shape}, '
                    f'expected {expected_shape}'
                )

        slope_values = self._encode_cost(
            array=self.slope_deg,
            minimum=0.0,
            maximum=self.slope_limit_deg,
            valid_mask=self.validity
        )

        roughness_values = self._encode_cost(
            array=self.roughness_m,
            minimum=0.0,
            maximum=self.roughness_limit_m,
            valid_mask=self.validity
        )

        impedance_values = self._encode_cost(
            array=self.impedance,
            minimum=0.0,
            maximum=1.0,
            valid_mask=self.validity
        )

        validity_values = np.where(
            self.validity,
            100,
            0
        ).astype(np.int8)

        traversability_values = np.full(
            expected_shape,
            -1,
            dtype=np.int8
        )

        traversability_values[
            self.validity
            & self.traversability
        ] = 100

        traversability_values[
            self.validity
            & ~self.traversability
        ] = 0

        self.slope_message = self._make_grid(
            slope_values
        )

        self.roughness_message = self._make_grid(
            roughness_values
        )

        self.impedance_message = self._make_grid(
            impedance_values
        )

        self.validity_message = self._make_grid(
            validity_values
        )

        self.traversability_message = self._make_grid(
            traversability_values
        )

    @staticmethod
    def _encode_cost(
        array,
        minimum,
        maximum,
        valid_mask
    ):
        output = np.full(
            array.shape,
            -1,
            dtype=np.int8
        )

        if maximum <= minimum:
            output[valid_mask] = 0
            return output

        normalised = (
            array - minimum
        ) / (
            maximum - minimum
        )

        normalised = np.clip(
            normalised,
            0.0,
            1.0
        )

        encoded = np.rint(
            normalised * 100.0
        ).astype(np.int16)

        output[valid_mask] = encoded[
            valid_mask
        ].astype(np.int8)

        return output

    def _make_grid(self, array):
        message = OccupancyGrid()

        message.header.frame_id = self.frame_id

        message.info.resolution = float(
            self.resolution
        )

        message.info.width = int(
            self.width
        )

        message.info.height = int(
            self.height
        )

        message.info.origin.position.x = float(
            self.origin_x
        )

        message.info.origin.position.y = float(
            self.origin_y
        )

        message.info.origin.position.z = 0.0

        message.info.origin.orientation.x = 0.0
        message.info.origin.orientation.y = 0.0
        message.info.origin.orientation.z = 0.0
        message.info.origin.orientation.w = 1.0

        # NPZ/image row zero is at the top of the image,
        # corresponding to positive world Y.
        #
        # OccupancyGrid row zero starts at negative world Y.
        # Therefore the array must be vertically flipped.
        ros_array = np.flipud(
            array
        )

        message.data = (
            ros_array
            .reshape(-1)
            .astype(np.int8)
            .tolist()
        )

        return message

    def publish_all(self):
        current_time = (
            self.get_clock()
            .now()
            .to_msg()
        )

        messages = (
            self.slope_message,
            self.roughness_message,
            self.impedance_message,
            self.validity_message,
            self.traversability_message,
        )

        for message in messages:
            message.header.stamp = current_time
            message.info.map_load_time = current_time

        self.slope_publisher.publish(
            self.slope_message
        )

        self.roughness_publisher.publish(
            self.roughness_message
        )

        self.impedance_publisher.publish(
            self.impedance_message
        )

        self.validity_publisher.publish(
            self.validity_message
        )

        self.traversability_publisher.publish(
            self.traversability_message
        )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = TerrainLayerPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                str(error)
            )
        else:
            print(
                f'ERROR: {error}'
            )

        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
