#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys

import numpy as np
from osgeo import gdal


def expand_path(path):
    return os.path.abspath(os.path.expanduser(path))


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Find a flat and smooth TurtleBot3 spawn point '
            'from a generated DEM terrain metadata file.'
        )
    )

    parser.add_argument(
        '--metadata',
        required=True,
        help='Terrain metadata JSON file'
    )

    parser.add_argument(
        '--margin',
        type=float,
        default=0.20,
        help='Fraction of map excluded near each boundary'
    )

    parser.add_argument(
        '--window',
        type=int,
        default=15,
        help='Local evaluation window size in pixels'
    )

    parser.add_argument(
        '--robot-clearance',
        type=float,
        default=0.28,
        help='Spawn height above terrain in metres'
    )

    parser.add_argument(
        '--top',
        type=int,
        default=5,
        help='Number of candidate points to print'
    )

    args = parser.parse_args()

    metadata_path = expand_path(args.metadata)

    if not os.path.isfile(metadata_path):
        print(
            f'ERROR: Metadata file not found: '
            f'{metadata_path}'
        )
        sys.exit(1)

    with open(
        metadata_path,
        'r',
        encoding='utf-8'
    ) as file:
        metadata = json.load(file)

    heightmap_path = expand_path(
        metadata['output_heightmap']
    )

    if not os.path.isfile(heightmap_path):
        print(
            f'ERROR: Heightmap not found: '
            f'{heightmap_path}'
        )
        sys.exit(1)

    target_x = float(
        metadata['gazebo']['target_x_m']
    )

    target_y = float(
        metadata['gazebo']['target_y_m']
    )

    target_z = float(
        metadata['gazebo']['final_relief_m']
    )

    dataset = gdal.Open(
        heightmap_path,
        gdal.GA_ReadOnly
    )

    if dataset is None:
        print(
            f'ERROR: GDAL could not open: '
            f'{heightmap_path}'
        )
        sys.exit(1)

    height_data = (
        dataset
        .GetRasterBand(1)
        .ReadAsArray()
        .astype(np.float64)
    )

    dataset = None

    minimum = float(np.min(height_data))
    maximum = float(np.max(height_data))

    if maximum <= minimum:
        print('ERROR: Heightmap has no elevation range.')
        sys.exit(1)

    normalised = (
        height_data - minimum
    ) / (
        maximum - minimum
    )

    terrain_height = normalised * target_z

    rows, columns = terrain_height.shape

    pixel_x = target_x / max(1, columns - 1)
    pixel_y = target_y / max(1, rows - 1)

    gradient_y, gradient_x = np.gradient(
        terrain_height,
        pixel_y,
        pixel_x
    )

    slope_deg = np.degrees(
        np.arctan(
            np.sqrt(
                gradient_x ** 2
                + gradient_y ** 2
            )
        )
    )

    half_window = max(
        2,
        args.window // 2
    )

    margin_rows = max(
        half_window + 1,
        int(rows * args.margin)
    )

    margin_columns = max(
        half_window + 1,
        int(columns * args.margin)
    )

    candidates = []

    step = max(
        2,
        args.window // 3
    )

    for row in range(
        margin_rows,
        rows - margin_rows,
        step
    ):
        for column in range(
            margin_columns,
            columns - margin_columns,
            step
        ):
            row_start = row - half_window
            row_end = row + half_window + 1

            column_start = column - half_window
            column_end = column + half_window + 1

            local_height = terrain_height[
                row_start:row_end,
                column_start:column_end
            ]

            local_slope = slope_deg[
                row_start:row_end,
                column_start:column_end
            ]

            mean_slope = float(
                np.mean(local_slope)
            )

            slope_95 = float(
                np.percentile(
                    local_slope,
                    95
                )
            )

            roughness = float(
                np.std(local_height)
            )

            # Lower score means safer and flatter.
            score = (
                0.55 * mean_slope
                + 0.35 * slope_95
                + 25.0 * roughness
            )

            # Heightmap image row zero is at the top.
            world_x = (
                -target_x / 2.0
                + column * pixel_x
            )

            world_y = (
                target_y / 2.0
                - row * pixel_y
            )

            surface_z = float(
                terrain_height[row, column]
            )

            spawn_z = (
                surface_z
                + args.robot_clearance
            )

            candidates.append({
                'score': score,
                'row': row,
                'column': column,
                'x': world_x,
                'y': world_y,
                'surface_z': surface_z,
                'spawn_z': spawn_z,
                'mean_slope': mean_slope,
                'slope_95': slope_95,
                'roughness': roughness,
            })

    candidates.sort(
        key=lambda item: item['score']
    )

    selected = candidates[
        :max(1, args.top)
    ]

    print()
    print('Safe spawn candidates')
    print('---------------------')
    print(f'Heightmap: {heightmap_path}')
    print(
        f'Terrain dimensions: '
        f'{target_x:.3f} × '
        f'{target_y:.3f} × '
        f'{target_z:.3f} m'
    )
    print()

    for index, candidate in enumerate(
        selected,
        start=1
    ):
        print(
            f'Candidate {index}:'
        )

        print(
            f"  start_x = "
            f"{candidate['x']:.3f}"
        )

        print(
            f"  start_y = "
            f"{candidate['y']:.3f}"
        )

        print(
            f"  start_z = "
            f"{candidate['spawn_z']:.3f}"
        )

        print(
            f"  surface_z = "
            f"{candidate['surface_z']:.3f}"
        )

        print(
            f"  mean slope = "
            f"{candidate['mean_slope']:.3f} deg"
        )

        print(
            f"  local 95% slope = "
            f"{candidate['slope_95']:.3f} deg"
        )

        print(
            f"  roughness = "
            f"{candidate['roughness']:.5f} m"
        )

        print()

    best = selected[0]

    print('Recommended launch arguments:')
    print(
        f"start_x:={best['x']:.3f} "
        f"start_y:={best['y']:.3f} "
        f"start_z:={best['spawn_z']:.3f}"
    )


if __name__ == '__main__':
    main()
