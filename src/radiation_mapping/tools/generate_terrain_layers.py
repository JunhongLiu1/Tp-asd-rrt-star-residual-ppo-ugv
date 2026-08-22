#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import uniform_filter
from scipy.ndimage import zoom


def expand_path(path):
    return Path(
        os.path.abspath(
            os.path.expanduser(path)
        )
    )


def read_required_float(container, key, description):
    if key not in container:
        raise KeyError(
            f'Missing metadata value for {description}: {key}'
        )

    return float(container[key])


def resize_array(array, output_rows, output_columns):
    input_rows, input_columns = array.shape

    scale_y = output_rows / float(input_rows)
    scale_x = output_columns / float(input_columns)

    resized = zoom(
        array,
        zoom=(scale_y, scale_x),
        order=1,
        mode='nearest',
        prefilter=False
    )

    # scipy.ndimage.zoom can occasionally differ by one cell
    # because of floating-point rounding.
    resized = resized[
        :output_rows,
        :output_columns
    ]

    if resized.shape != (
        output_rows,
        output_columns
    ):
        pad_rows = output_rows - resized.shape[0]
        pad_columns = (
            output_columns - resized.shape[1]
        )

        resized = np.pad(
            resized,
            (
                (0, max(0, pad_rows)),
                (0, max(0, pad_columns))
            ),
            mode='edge'
        )

        resized = resized[
            :output_rows,
            :output_columns
        ]

    return resized


def save_grayscale(path, array, minimum, maximum):
    if maximum <= minimum:
        scaled = np.zeros(
            array.shape,
            dtype=np.uint8
        )
    else:
        scaled = (
            (array - minimum)
            / (maximum - minimum)
        )

        scaled = np.clip(
            scaled,
            0.0,
            1.0
        )

        scaled = np.rint(
            scaled * 255.0
        ).astype(np.uint8)

    Image.fromarray(
        scaled,
        mode='L'
    ).save(path)


def calculate_statistics(array, valid_mask):
    values = array[valid_mask]

    if values.size == 0:
        return {
            'minimum': None,
            'mean': None,
            'median': None,
            'percentile_90': None,
            'percentile_95': None,
            'maximum': None,
        }

    return {
        'minimum': float(np.min(values)),
        'mean': float(np.mean(values)),
        'median': float(np.median(values)),
        'percentile_90': float(
            np.percentile(values, 90)
        ),
        'percentile_95': float(
            np.percentile(values, 95)
        ),
        'maximum': float(np.max(values)),
    }


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Generate metric elevation, slope, roughness, '
            'terrain impedance and traversability layers '
            'from a DEM-inspired Gazebo heightmap.'
        )
    )

    parser.add_argument(
        '--metadata',
        required=True,
        help='Input terrain metadata JSON'
    )

    parser.add_argument(
        '--heightmap',
        default='',
        help=(
            'Optional heightmap override. When omitted, '
            'output_heightmap from metadata is used.'
        )
    )

    parser.add_argument(
        '--output-prefix',
        required=True,
        help=(
            'Output prefix, for example '
            '/path/terrain_layers_easy'
        )
    )

    parser.add_argument(
        '--resolution',
        type=float,
        default=0.10,
        help='Metric map resolution in metres per cell'
    )

    parser.add_argument(
        '--roughness-window-m',
        type=float,
        default=0.50,
        help=(
            'Width of the local roughness window '
            'in metres'
        )
    )

    parser.add_argument(
        '--slope-cost-limit-deg',
        type=float,
        default=20.0,
        help=(
            'Slope corresponding to normalised '
            'slope cost 1.0'
        )
    )

    parser.add_argument(
        '--roughness-cost-limit-m',
        type=float,
        default=0.06,
        help=(
            'Roughness corresponding to normalised '
            'roughness cost 1.0'
        )
    )

    parser.add_argument(
        '--max-traversable-slope-deg',
        type=float,
        default=22.0,
        help='Slope threshold for traversability mask'
    )

    parser.add_argument(
        '--max-traversable-roughness-m',
        type=float,
        default=0.10,
        help='Roughness threshold for traversability mask'
    )

    parser.add_argument(
        '--slope-weight',
        type=float,
        default=0.75,
        help='Slope contribution to impedance'
    )

    parser.add_argument(
        '--roughness-weight',
        type=float,
        default=0.25,
        help='Roughness contribution to impedance'
    )

    args = parser.parse_args()

    if args.resolution <= 0.0:
        raise ValueError(
            '--resolution must be positive'
        )

    if args.roughness_window_m <= 0.0:
        raise ValueError(
            '--roughness-window-m must be positive'
        )

    weight_sum = (
        args.slope_weight
        + args.roughness_weight
    )

    if weight_sum <= 0.0:
        raise ValueError(
            'Terrain cost weights must sum '
            'to a positive value'
        )

    slope_weight = (
        args.slope_weight / weight_sum
    )

    roughness_weight = (
        args.roughness_weight / weight_sum
    )

    metadata_path = expand_path(
        args.metadata
    )

    output_prefix = expand_path(
        args.output_prefix
    )

    output_prefix.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    if not metadata_path.is_file():
        raise FileNotFoundError(
            f'Metadata not found: {metadata_path}'
        )

    with metadata_path.open(
        'r',
        encoding='utf-8'
    ) as file:
        source_metadata = json.load(file)

    gazebo_metadata = source_metadata.get(
        'gazebo',
        {}
    )

    target_x = read_required_float(
        gazebo_metadata,
        'target_x_m',
        'terrain X size'
    )

    target_y = read_required_float(
        gazebo_metadata,
        'target_y_m',
        'terrain Y size'
    )

    final_relief = read_required_float(
        gazebo_metadata,
        'final_relief_m',
        'terrain relief'
    )

    if args.heightmap:
        heightmap_path = expand_path(
            args.heightmap
        )
    else:
        metadata_heightmap = source_metadata.get(
            'output_heightmap',
            ''
        )

        if not metadata_heightmap:
            raise KeyError(
                'Metadata does not contain output_heightmap. '
                'Use --heightmap explicitly.'
            )

        heightmap_path = expand_path(
            metadata_heightmap
        )

    if not heightmap_path.is_file():
        raise FileNotFoundError(
            f'Heightmap not found: {heightmap_path}'
        )

    image = Image.open(
        heightmap_path
    ).convert('L')

    source_pixels = np.asarray(
        image,
        dtype=np.float64
    )

    source_min = float(
        np.min(source_pixels)
    )

    source_max = float(
        np.max(source_pixels)
    )

    if source_max <= source_min:
        raise RuntimeError(
            'Heightmap contains no elevation range'
        )

    normalised_height = (
        source_pixels - source_min
    ) / (
        source_max - source_min
    )

    source_elevation = (
        normalised_height * final_relief
    )

    output_columns = max(
        2,
        int(round(
            target_x / args.resolution
        ))
    )

    output_rows = max(
        2,
        int(round(
            target_y / args.resolution
        ))
    )

    actual_size_x = (
        output_columns * args.resolution
    )

    actual_size_y = (
        output_rows * args.resolution
    )

    origin_x = (
        -actual_size_x / 2.0
    )

    origin_y = (
        -actual_size_y / 2.0
    )

    elevation = resize_array(
        source_elevation,
        output_rows,
        output_columns
    )

    valid_mask = np.isfinite(
        elevation
    )

    elevation = np.nan_to_num(
        elevation,
        nan=0.0,
        posinf=final_relief,
        neginf=0.0
    )

    # Array row zero corresponds to the top of the image,
    # which represents positive world Y. Slope magnitude is
    # unaffected by the sign of the Y derivative.
    gradient_y, gradient_x = np.gradient(
        elevation,
        args.resolution,
        args.resolution
    )

    slope_rise = np.sqrt(
        gradient_x ** 2
        + gradient_y ** 2
    )

    slope_deg = np.degrees(
        np.arctan(
            slope_rise
        )
    )

    window_cells = max(
        3,
        int(round(
            args.roughness_window_m
            / args.resolution
        ))
    )

    if window_cells % 2 == 0:
        window_cells += 1

    local_mean = uniform_filter(
        elevation,
        size=window_cells,
        mode='nearest'
    )

    local_mean_squared = uniform_filter(
        elevation ** 2,
        size=window_cells,
        mode='nearest'
    )

    local_variance = np.maximum(
        local_mean_squared
        - local_mean ** 2,
        0.0
    )

    roughness_m = np.sqrt(
        local_variance
    )

    slope_cost = np.clip(
        slope_deg
        / args.slope_cost_limit_deg,
        0.0,
        1.0
    )

    roughness_cost = np.clip(
        roughness_m
        / args.roughness_cost_limit_m,
        0.0,
        1.0
    )

    impedance = (
        slope_weight * slope_cost
        + roughness_weight * roughness_cost
    )

    impedance = np.clip(
        impedance,
        0.0,
        1.0
    )

    impedance[~valid_mask] = 1.0

    traversability_mask = (
        valid_mask
        & (
            slope_deg
            <= args.max_traversable_slope_deg
        )
        & (
            roughness_m
            <= args.max_traversable_roughness_m
        )
    )

    npz_path = Path(
        str(output_prefix) + '.npz'
    )

    json_path = Path(
        str(output_prefix) + '_metadata.json'
    )

    elevation_png = Path(
        str(output_prefix) + '_elevation.png'
    )

    slope_png = Path(
        str(output_prefix) + '_slope.png'
    )

    roughness_png = Path(
        str(output_prefix) + '_roughness.png'
    )

    impedance_png = Path(
        str(output_prefix) + '_impedance.png'
    )

    validity_png = Path(
        str(output_prefix) + '_validity.png'
    )

    traversability_png = Path(
        str(output_prefix) + '_traversability.png'
    )

    np.savez_compressed(
        npz_path,
        elevation_m=elevation.astype(
            np.float32
        ),
        slope_deg=slope_deg.astype(
            np.float32
        ),
        roughness_m=roughness_m.astype(
            np.float32
        ),
        slope_cost=slope_cost.astype(
            np.float32
        ),
        roughness_cost=roughness_cost.astype(
            np.float32
        ),
        terrain_impedance=impedance.astype(
            np.float32
        ),
        validity_mask=valid_mask.astype(
            np.uint8
        ),
        traversability_mask=(
            traversability_mask.astype(
                np.uint8
            )
        )
    )

    save_grayscale(
        elevation_png,
        elevation,
        0.0,
        final_relief
    )

    save_grayscale(
        slope_png,
        slope_deg,
        0.0,
        args.slope_cost_limit_deg
    )

    save_grayscale(
        roughness_png,
        roughness_m,
        0.0,
        args.roughness_cost_limit_m
    )

    save_grayscale(
        impedance_png,
        impedance,
        0.0,
        1.0
    )

    Image.fromarray(
        valid_mask.astype(
            np.uint8
        ) * 255,
        mode='L'
    ).save(validity_png)

    Image.fromarray(
        traversability_mask.astype(
            np.uint8
        ) * 255,
        mode='L'
    ).save(traversability_png)

    valid_cells = int(
        np.count_nonzero(valid_mask)
    )

    traversable_cells = int(
        np.count_nonzero(
            traversability_mask
        )
    )

    traversable_fraction = (
        traversable_cells
        / valid_cells
        if valid_cells > 0
        else 0.0
    )

    output_metadata = {
        'source_metadata': str(
            metadata_path
        ),
        'source_heightmap': str(
            heightmap_path
        ),
        'output_npz': str(
            npz_path
        ),
        'coordinate_convention': {
            'array_row_zero': (
                'positive world Y / top of map'
            ),
            'array_column_zero': (
                'negative world X / left of map'
            ),
            'ros_occupancy_grid_note': (
                'Flip arrays vertically before flattening '
                'for OccupancyGrid publication.'
            ),
        },
        'grid': {
            'resolution_m': (
                args.resolution
            ),
            'width_cells': (
                output_columns
            ),
            'height_cells': (
                output_rows
            ),
            'size_x_m': (
                actual_size_x
            ),
            'size_y_m': (
                actual_size_y
            ),
            'origin_x_m': (
                origin_x
            ),
            'origin_y_m': (
                origin_y
            ),
        },
        'terrain': {
            'source_size_x_m': (
                target_x
            ),
            'source_size_y_m': (
                target_y
            ),
            'relief_m': (
                final_relief
            ),
        },
        'cost_model': {
            'slope_weight': (
                slope_weight
            ),
            'roughness_weight': (
                roughness_weight
            ),
            'slope_cost_limit_deg': (
                args.slope_cost_limit_deg
            ),
            'roughness_cost_limit_m': (
                args.roughness_cost_limit_m
            ),
            'roughness_window_m': (
                args.roughness_window_m
            ),
            'roughness_window_cells': (
                window_cells
            ),
            'maximum_traversable_slope_deg': (
                args.max_traversable_slope_deg
            ),
            'maximum_traversable_roughness_m': (
                args.max_traversable_roughness_m
            ),
        },
        'statistics': {
            'elevation_m': (
                calculate_statistics(
                    elevation,
                    valid_mask
                )
            ),
            'slope_deg': (
                calculate_statistics(
                    slope_deg,
                    valid_mask
                )
            ),
            'roughness_m': (
                calculate_statistics(
                    roughness_m,
                    valid_mask
                )
            ),
            'terrain_impedance': (
                calculate_statistics(
                    impedance,
                    valid_mask
                )
            ),
            'valid_cells': (
                valid_cells
            ),
            'traversable_cells': (
                traversable_cells
            ),
            'traversable_fraction': (
                float(
                    traversable_fraction
                )
            ),
        },
        'preview_images': {
            'elevation': str(
                elevation_png
            ),
            'slope': str(
                slope_png
            ),
            'roughness': str(
                roughness_png
            ),
            'impedance': str(
                impedance_png
            ),
            'validity': str(
                validity_png
            ),
            'traversability': str(
                traversability_png
            ),
        },
    }

    with json_path.open(
        'w',
        encoding='utf-8'
    ) as file:
        json.dump(
            output_metadata,
            file,
            indent=2
        )

    slope_stats = output_metadata[
        'statistics'
    ]['slope_deg']

    roughness_stats = output_metadata[
        'statistics'
    ]['roughness_m']

    impedance_stats = output_metadata[
        'statistics'
    ]['terrain_impedance']

    print()
    print('Terrain layer generation complete')
    print('---------------------------------')
    print(f'Metadata: {metadata_path}')
    print(f'Heightmap: {heightmap_path}')
    print(
        f'Grid: {output_columns} × '
        f'{output_rows} cells'
    )
    print(
        f'Resolution: '
        f'{args.resolution:.3f} m/cell'
    )
    print(
        f'Physical size: '
        f'{actual_size_x:.3f} × '
        f'{actual_size_y:.3f} m'
    )
    print(
        f'Slope mean: '
        f'{slope_stats["mean"]:.3f} deg'
    )
    print(
        f'Slope 95%: '
        f'{slope_stats["percentile_95"]:.3f} deg'
    )
    print(
        f'Slope maximum: '
        f'{slope_stats["maximum"]:.3f} deg'
    )
    print(
        f'Roughness mean: '
        f'{roughness_stats["mean"]:.5f} m'
    )
    print(
        f'Roughness 95%: '
        f'{roughness_stats["percentile_95"]:.5f} m'
    )
    print(
        f'Impedance mean: '
        f'{impedance_stats["mean"]:.4f}'
    )
    print(
        f'Impedance 95%: '
        f'{impedance_stats["percentile_95"]:.4f}'
    )
    print(
        f'Traversable fraction: '
        f'{traversable_fraction * 100.0:.2f}%'
    )
    print(f'NPZ: {npz_path}')
    print(f'Output metadata: {json_path}')
    print()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr
        )
        sys.exit(1)
