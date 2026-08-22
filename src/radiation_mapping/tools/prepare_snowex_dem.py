#!/usr/bin/env python3

import argparse
import os
import sys

import numpy as np
from osgeo import gdal


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Prepare a SnowEx DEM for Gazebo by centre-cropping, '
            'filling invalid cells, and shifting minimum elevation to zero.'
        )
    )

    parser.add_argument(
        'input_dem',
        help='Input SnowEx GeoTIFF DEM'
    )

    parser.add_argument(
        'output_dem',
        help='Output zero-based GeoTIFF'
    )

    parser.add_argument(
        '--size',
        type=int,
        default=257,
        help='Output crop size. Default: 257'
    )

    args = parser.parse_args()

    gdal.UseExceptions()

    input_path = os.path.abspath(
        os.path.expanduser(args.input_dem)
    )

    output_path = os.path.abspath(
        os.path.expanduser(args.output_dem)
    )

    if not os.path.isfile(input_path):
        print(f'ERROR: Input DEM does not exist: {input_path}')
        sys.exit(1)

    if args.size < 3:
        print('ERROR: Crop size must be at least 3.')
        sys.exit(1)

    source = gdal.Open(input_path, gdal.GA_ReadOnly)

    if source is None:
        print(f'ERROR: GDAL could not open: {input_path}')
        sys.exit(1)

    source_width = source.RasterXSize
    source_height = source.RasterYSize

    if source_width < args.size or source_height < args.size:
        print(
            f'ERROR: Source size is {source_width} x {source_height}, '
            f'but requested crop is {args.size} x {args.size}.'
        )
        sys.exit(1)

    x_offset = (source_width - args.size) // 2
    y_offset = (source_height - args.size) // 2

    source_band = source.GetRasterBand(1)

    elevation = source_band.ReadAsArray(
        x_offset,
        y_offset,
        args.size,
        args.size
    ).astype(np.float32)

    nodata = source_band.GetNoDataValue()

    valid_mask = np.isfinite(elevation)

    if nodata is not None:
        valid_mask &= elevation != nodata

    valid_mask &= elevation > -32000.0

    if not np.any(valid_mask):
        print('ERROR: Selected crop contains no valid elevation values.')
        sys.exit(1)

    valid_values = elevation[valid_mask]

    original_min = float(np.min(valid_values))
    original_max = float(np.max(valid_values))
    median_value = float(np.median(valid_values))

    # Replace invalid cells with the median valid elevation.
    elevation[~valid_mask] = median_value

    # Shift the minimum elevation to zero.
    elevation -= original_min

    processed_min = float(np.min(elevation))
    processed_max = float(np.max(elevation))

    os.makedirs(
        os.path.dirname(output_path),
        exist_ok=True
    )

    driver = gdal.GetDriverByName('GTiff')

    output = driver.Create(
        output_path,
        args.size,
        args.size,
        1,
        gdal.GDT_Float32,
        options=[
            'COMPRESS=LZW',
            'TILED=YES'
        ]
    )

    if output is None:
        print(f'ERROR: Could not create output: {output_path}')
        sys.exit(1)

    source_transform = source.GetGeoTransform()

    if source_transform:
        output_origin_x = (
            source_transform[0]
            + x_offset * source_transform[1]
            + y_offset * source_transform[2]
        )

        output_origin_y = (
            source_transform[3]
            + x_offset * source_transform[4]
            + y_offset * source_transform[5]
        )

        output_transform = (
            output_origin_x,
            source_transform[1],
            source_transform[2],
            output_origin_y,
            source_transform[4],
            source_transform[5]
        )

        output.SetGeoTransform(output_transform)

    projection = source.GetProjection()

    if projection:
        output.SetProjection(projection)

    output_band = output.GetRasterBand(1)
    output_band.WriteArray(elevation)
    output_band.SetDescription('Zero-based SnowEx elevation')
    output_band.FlushCache()

    output.FlushCache()

    output_band = None
    output = None
    source_band = None
    source = None

    print('SnowEx DEM preprocessing completed.')
    print(f'Input: {input_path}')
    print(f'Original size: {source_width} x {source_height}')
    print(f'Crop offset: x={x_offset}, y={y_offset}')
    print(f'Output size: {args.size} x {args.size}')
    print(
        f'Original elevation range: '
        f'{original_min:.3f} m to {original_max:.3f} m'
    )
    print(
        f'Processed elevation range: '
        f'{processed_min:.3f} m to {processed_max:.3f} m'
    )
    print(f'Output: {output_path}')


if __name__ == '__main__':
    main()
