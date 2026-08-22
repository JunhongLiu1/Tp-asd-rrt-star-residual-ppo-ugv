#!/usr/bin/env python3

import json
import math
from pathlib import Path

import numpy as np
import rclpy

from ament_index_python.packages import (
    get_package_share_directory
)
from radiation_interfaces.srv import TerrainQuery
from rclpy.node import Node


class TerrainQueryServer(Node):
    """Query DEM-derived terrain data using map/world coordinates."""

    VALID_LEVELS = (
        'easy',
        'medium',
        'hard',
    )

    def __init__(self):
        super().__init__('terrain_query_server')

        self.declare_parameter(
            'terrain_level',
            'easy'
        )

        self.declare_parameter(
            'data_directory',
            ''
        )

        self.declare_parameter(
            'service_name',
            '/query_terrain'
        )

        self.terrain_level = str(
            self.get_parameter(
                'terrain_level'
            ).value
        ).strip().lower()

        data_directory = str(
            self.get_parameter(
                'data_directory'
            ).value
        ).strip()

        service_name = str(
            self.get_parameter(
                'service_name'
            ).value
        ).strip()

        if self.terrain_level not in self.VALID_LEVELS:
            raise RuntimeError(
                'terrain_level must be easy, '
                'medium, or hard'
            )

        if not service_name:
            raise RuntimeError(
                'service_name cannot be empty'
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

        self._load_data()

        self.service = self.create_service(
            TerrainQuery,
            service_name,
            self.query_callback
        )

        self.get_logger().info(
            'Terrain query server started'
        )

        self.get_logger().info(
            f'Terrain level: {self.terrain_level}'
        )

        self.get_logger().info(
            f'Service: {service_name}'
        )

        self.get_logger().info(
            f'Grid: {self.width} x {self.height}'
        )

        self.get_logger().info(
            f'Resolution: {self.resolution:.3f} m/cell'
        )

        self.get_logger().info(
            'Map bounds: '
            f'x=[{self.minimum_x:.3f}, '
            f'{self.maximum_x:.3f}), '
            f'y=[{self.minimum_y:.3f}, '
            f'{self.maximum_y:.3f})'
        )

    def _load_data(self):
        if not self.npz_path.is_file():
            raise FileNotFoundError(
                f'NPZ file not found: {self.npz_path}'
            )

        if not self.metadata_path.is_file():
            raise FileNotFoundError(
                'Metadata file not found: '
                f'{self.metadata_path}'
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

        self.minimum_x = self.origin_x
        self.minimum_y = self.origin_y

        self.maximum_x = (
            self.origin_x
            + self.width * self.resolution
        )

        self.maximum_y = (
            self.origin_y
            + self.height * self.resolution
        )

        with np.load(self.npz_path) as data:
            required = (
                'elevation_m',
                'slope_deg',
                'roughness_m',
                'terrain_impedance',
                'validity_mask',
                'traversability_mask',
            )

            for name in required:
                if name not in data.files:
                    raise KeyError(
                        f'Missing NPZ array: {name}'
                    )

            self.elevation = np.asarray(
                data['elevation_m'],
                dtype=np.float64
            )

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

        arrays = {
            'elevation_m': self.elevation,
            'slope_deg': self.slope,
            'roughness_m': self.roughness,
            'terrain_impedance': self.impedance,
            'validity_mask': self.validity,
            'traversability_mask': (
                self.traversability
            ),
        }

        for name, array in arrays.items():
            if array.shape != expected_shape:
                raise RuntimeError(
                    f'{name} has shape {array.shape}; '
                    f'expected {expected_shape}'
                )

    def query_callback(
        self,
        request,
        response
    ):
        x = float(request.x)
        y = float(request.y)

        if not (
            math.isfinite(x)
            and math.isfinite(y)
        ):
            return self._outside_response(
                response,
                'Coordinates must be finite.'
            )

        inside = (
            self.minimum_x <= x < self.maximum_x
            and self.minimum_y <= y < self.maximum_y
        )

        if not inside:
            return self._outside_response(
                response,
                (
                    'Point is outside terrain bounds: '
                    f'x=[{self.minimum_x:.3f}, '
                    f'{self.maximum_x:.3f}), '
                    f'y=[{self.minimum_y:.3f}, '
                    f'{self.maximum_y:.3f})'
                )
            )

        grid_column = int(
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

        # NPZ/image row 0 is at positive world Y.
        # ROS OccupancyGrid row 0 is at negative world Y.
        source_row = (
            self.height
            - 1
            - grid_row
        )

        valid = bool(
            self.validity[
                source_row,
                grid_column
            ]
        )

        traversable = bool(
            valid
            and self.traversability[
                source_row,
                grid_column
            ]
        )

        response.inside_map = True
        response.valid = valid
        response.traversable = traversable

        response.grid_row = grid_row
        response.grid_column = grid_column
        response.source_array_row = source_row

        if valid:
            response.elevation_m = float(
                self.elevation[
                    source_row,
                    grid_column
                ]
            )

            response.slope_deg = float(
                self.slope[
                    source_row,
                    grid_column
                ]
            )

            response.roughness_m = float(
                self.roughness[
                    source_row,
                    grid_column
                ]
            )

            response.terrain_impedance = float(
                np.clip(
                    self.impedance[
                        source_row,
                        grid_column
                    ],
                    0.0,
                    1.0
                )
            )

            response.status = (
                'OK'
                if traversable
                else 'Valid cell but marked non-traversable.'
            )

        else:
            response.elevation_m = float('nan')
            response.slope_deg = float('nan')
            response.roughness_m = float('nan')
            response.terrain_impedance = float('nan')
            response.status = (
                'Cell is inside the map but invalid.'
            )

        return response

    @staticmethod
    def _outside_response(
        response,
        status
    ):
        response.inside_map = False
        response.valid = False
        response.traversable = False

        response.grid_row = -1
        response.grid_column = -1
        response.source_array_row = -1

        response.elevation_m = float('nan')
        response.slope_deg = float('nan')
        response.roughness_m = float('nan')
        response.terrain_impedance = float('nan')

        response.status = status

        return response


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = TerrainQueryServer()
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
