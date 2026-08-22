#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys

import numpy as np
from osgeo import gdal

from scipy.ndimage import (
    distance_transform_edt,
    gaussian_filter,
    median_filter,
    zoom
)


def expand_path(path):
    """Expand ~ and return an absolute path."""
    return os.path.abspath(
        os.path.expanduser(path)
    )


def file_uri(path):
    """Convert an absolute filesystem path to a file URI."""
    return 'file://' + expand_path(path)


def read_valid_dem(input_dem):
    """
    Read the source DEM and extract its valid-data envelope.

    This removes completely empty outside rows and columns, while
    retaining internal NoData cells inside the LiDAR coverage area.
    """

    gdal.UseExceptions()

    dataset = gdal.Open(
        input_dem,
        gdal.GA_ReadOnly
    )

    if dataset is None:
        raise RuntimeError(
            f'GDAL could not open DEM: {input_dem}'
        )

    band = dataset.GetRasterBand(1)

    elevation = band.ReadAsArray().astype(
        np.float64
    )

    nodata = band.GetNoDataValue()

    valid = np.isfinite(elevation)

    if nodata is not None:
        valid &= elevation != nodata

    # Safety check for common DEM void values.
    valid &= elevation > -32000.0

    if not np.any(valid):
        raise RuntimeError(
            'The input DEM contains no valid elevation values.'
        )

    valid_rows, valid_columns = np.where(valid)

    row_min = int(np.min(valid_rows))
    row_max = int(np.max(valid_rows))

    column_min = int(np.min(valid_columns))
    column_max = int(np.max(valid_columns))

    elevation = elevation[
        row_min:row_max + 1,
        column_min:column_max + 1
    ]

    valid = valid[
        row_min:row_max + 1,
        column_min:column_max + 1
    ]

    transform = dataset.GetGeoTransform()

    pixel_x = math.hypot(
        float(transform[1]),
        float(transform[4])
    )

    pixel_y = math.hypot(
        float(transform[2]),
        float(transform[5])
    )

    if pixel_x <= 0.0:
        pixel_x = 1.0

    if pixel_y <= 0.0:
        pixel_y = pixel_x

    valid_percentage = (
        100.0
        * np.count_nonzero(valid)
        / valid.size
    )

    information = {
        'original_width_pixels':
            int(dataset.RasterXSize),

        'original_height_pixels':
            int(dataset.RasterYSize),

        'valid_width_pixels':
            int(elevation.shape[1]),

        'valid_height_pixels':
            int(elevation.shape[0]),

        'pixel_x_m':
            float(pixel_x),

        'pixel_y_m':
            float(pixel_y),

        'valid_percentage':
            float(valid_percentage),

        'valid_envelope': {
            'row_min': row_min,
            'row_max': row_max,
            'column_min': column_min,
            'column_max': column_max
        }
    }

    band = None
    dataset = None

    return elevation, valid, information


def fill_internal_invalid_values(elevation, valid):
    """
    Fill NoData cells using the nearest valid elevation.

    Using one common median value for all NoData cells can create
    artificial flight-line bands and sharp rectangular regions.
    Nearest-valid filling produces a more continuous initial surface.
    """

    if elevation.ndim != 2:
        raise ValueError(
            'Elevation array must be two-dimensional.'
        )

    if elevation.shape != valid.shape:
        raise ValueError(
            'Elevation and validity mask shapes do not match.'
        )

    if not np.any(valid):
        raise RuntimeError(
            'No valid DEM elevation is available.'
        )

    invalid = ~valid

    if not np.any(invalid):
        return elevation.astype(np.float64)

    # Valid cells are zero in the invalid mask.
    # The distance transform returns the indices of the nearest zero,
    # therefore the nearest valid elevation.
    nearest_indices = distance_transform_edt(
        invalid,
        return_distances=False,
        return_indices=True
    )

    filled = elevation[
        tuple(nearest_indices)
    ].astype(np.float64)

    # Preserve measured DEM cells exactly.
    filled[valid] = elevation[valid]

    return filled


def resample_array(array, output_size):
    """
    Resample a two-dimensional array to output_size × output_size.

    scipy.ndimage.zoom is used instead of GDAL Warp because this
    in-memory terrain array has no geographic coordinate transform.
    """

    if array.ndim != 2:
        raise ValueError(
            'Input terrain array must be two-dimensional.'
        )

    if output_size < 3:
        raise ValueError(
            'Output size must be at least 3.'
        )

    input_height, input_width = array.shape

    zoom_y = float(output_size) / float(input_height)
    zoom_x = float(output_size) / float(input_width)

    result = zoom(
        array.astype(np.float64),
        zoom=(zoom_y, zoom_x),
        order=1,
        mode='nearest',
        prefilter=False
    )

    # scipy rounding can occasionally produce a one-pixel difference.
    result = result[
        :output_size,
        :output_size
    ]

    pad_y = output_size - result.shape[0]
    pad_x = output_size - result.shape[1]

    if pad_y > 0 or pad_x > 0:
        result = np.pad(
            result,
            (
                (0, max(0, pad_y)),
                (0, max(0, pad_x))
            ),
            mode='edge'
        )

    if result.shape != (
        output_size,
        output_size
    ):
        raise RuntimeError(
            'Unexpected resampled terrain size: '
            f'{result.shape}'
        )

    return result


def mean_smooth(array, iterations):
    """
    Remove narrow LiDAR artefacts while retaining medium-scale
    valleys, ridges, and slope transitions.

    A small median filter removes isolated spikes. A light Gaussian
    filter removes scan artefacts without excessively blurring the
    principal terrain morphology.
    """

    if array.ndim != 2:
        raise ValueError(
            'Terrain array must be two-dimensional.'
        )

    result = array.astype(np.float64)

    result = median_filter(
        result,
        size=3,
        mode='nearest'
    )

    sigma = max(
        0.8,
        float(iterations) * 0.6
    )

    result = gaussian_filter(
        result,
        sigma=sigma,
        mode='nearest'
    )

    return result


def normalise_elevation(
    elevation,
    lower_percentile,
    upper_percentile
):
    """
    Remove extreme DEM spikes and normalise relative elevation
    into the interval 0 to 1.
    """

    if not 0.0 <= lower_percentile < upper_percentile <= 100.0:
        raise ValueError(
            'Percentiles must satisfy '
            '0 <= lower < upper <= 100.'
        )

    lower = float(
        np.percentile(
            elevation,
            lower_percentile
        )
    )

    upper = float(
        np.percentile(
            elevation,
            upper_percentile
        )
    )

    if upper <= lower:
        raise RuntimeError(
            'DEM elevation range is too small after processing.'
        )

    clipped = np.clip(
        elevation,
        lower,
        upper
    )

    normalised = (
        clipped - lower
    ) / (
        upper - lower
    )

    return normalised, lower, upper


def calculate_world_dimensions(
    source_information,
    target_longest_side
):
    """
    Preserve the X:Y aspect ratio of the valid DEM envelope while
    assigning a robot-scale horizontal size.
    """

    source_width = (
        source_information['valid_width_pixels']
        * source_information['pixel_x_m']
    )

    source_height = (
        source_information['valid_height_pixels']
        * source_information['pixel_y_m']
    )

    longest = max(
        source_width,
        source_height
    )

    if longest <= 0.0:
        raise RuntimeError(
            'Invalid physical DEM dimensions.'
        )

    target_x = (
        target_longest_side
        * source_width
        / longest
    )

    target_y = (
        target_longest_side
        * source_height
        / longest
    )

    return float(target_x), float(target_y)


def calculate_slope_statistics(
    normalised_height,
    target_x,
    target_y,
    target_relief
):
    """
    Calculate slope statistics after mapping normalised elevation
    into the Gazebo world dimensions.
    """

    height_m = (
        normalised_height
        * target_relief
    )

    denominator_x = max(
        1,
        normalised_height.shape[1] - 1
    )

    denominator_y = max(
        1,
        normalised_height.shape[0] - 1
    )

    pixel_x = target_x / denominator_x
    pixel_y = target_y / denominator_y

    gradient_y, gradient_x = np.gradient(
        height_m,
        pixel_y,
        pixel_x
    )

    gradient = np.sqrt(
        gradient_x ** 2
        + gradient_y ** 2
    )

    slopes = np.degrees(
        np.arctan(gradient)
    )

    return {
        'slope_mean_deg':
            float(np.mean(slopes)),

        'slope_50_deg':
            float(np.percentile(slopes, 50)),

        'slope_90_deg':
            float(np.percentile(slopes, 90)),

        'slope_95_deg':
            float(np.percentile(slopes, 95)),

        'slope_99_deg':
            float(np.percentile(slopes, 99)),

        'slope_max_deg':
            float(np.max(slopes))
    }


def limit_relief_for_slope(
    normalised_height,
    target_x,
    target_y,
    requested_relief,
    maximum_percentile_slope
):
    """
    Reduce the vertical relief when the 95th-percentile slope exceeds
    the specified robot-scale benchmark limit.
    """

    initial_statistics = calculate_slope_statistics(
        normalised_height,
        target_x,
        target_y,
        requested_relief
    )

    current_slope = initial_statistics[
        'slope_95_deg'
    ]

    if current_slope <= maximum_percentile_slope:
        return (
            requested_relief,
            initial_statistics,
            1.0
        )

    current_gradient = math.tan(
        math.radians(current_slope)
    )

    allowed_gradient = math.tan(
        math.radians(
            maximum_percentile_slope
        )
    )

    if current_gradient <= 1e-12:
        vertical_factor = 1.0
    else:
        vertical_factor = (
            allowed_gradient
            / current_gradient
        )

    vertical_factor = min(
        1.0,
        max(0.01, vertical_factor)
    )

    final_relief = (
        requested_relief
        * vertical_factor
    )

    final_statistics = calculate_slope_statistics(
        normalised_height,
        target_x,
        target_y,
        final_relief
    )

    return (
        float(final_relief),
        final_statistics,
        float(vertical_factor)
    )


def save_heightmap_png(
    normalised_height,
    output_png
):
    """
    Save a Gazebo-compatible 8-bit single-channel grayscale PNG.

    Gazebo Classic may produce Image coordinates-out-of-range errors
    with some 16-bit PNG heightmaps, therefore a true 8-bit Byte image
    is used here.
    """

    if normalised_height.ndim != 2:
        raise ValueError(
            'Heightmap must be a two-dimensional array.'
        )

    output_directory = os.path.dirname(
        output_png
    )

    os.makedirs(
        output_directory,
        exist_ok=True
    )

    clipped_height = np.clip(
        normalised_height,
        0.0,
        1.0
    )

    height_uint8 = np.round(
        clipped_height * 255.0
    ).astype(np.uint8)

    height_pixels, width_pixels = (
        height_uint8.shape
    )

    memory_driver = gdal.GetDriverByName(
        'MEM'
    )

    memory_dataset = memory_driver.Create(
        '',
        width_pixels,
        height_pixels,
        1,
        gdal.GDT_Byte
    )

    if memory_dataset is None:
        raise RuntimeError(
            'Could not create temporary heightmap dataset.'
        )

    memory_band = memory_dataset.GetRasterBand(1)

    memory_band.WriteArray(
        height_uint8
    )

    memory_band.SetColorInterpretation(
        gdal.GCI_GrayIndex
    )

    memory_band.FlushCache()

    png_driver = gdal.GetDriverByName(
        'PNG'
    )

    output_dataset = png_driver.CreateCopy(
        output_png,
        memory_dataset,
        strict=1
    )

    if output_dataset is None:
        memory_band = None
        memory_dataset = None

        raise RuntimeError(
            f'Could not save heightmap: {output_png}'
        )

    output_dataset.FlushCache()

    output_dataset = None
    memory_band = None
    memory_dataset = None

    print(
        'Saved Gazebo-compatible heightmap: '
        f'{width_pixels} × {height_pixels}, '
        '8-bit grayscale'
    )


def save_uniform_surface_texture(
    output_png
):
    """
    Generate a small uniform grey texture.

    This is not a geographic texture or hillshade. It only gives the
    Gazebo heightmap a neutral surface colour.
    """

    output_directory = os.path.dirname(
        output_png
    )

    os.makedirs(
        output_directory,
        exist_ok=True
    )

    width = 16
    height = 16

    red = np.full(
        (height, width),
        155,
        dtype=np.uint8
    )

    green = np.full(
        (height, width),
        158,
        dtype=np.uint8
    )

    blue = np.full(
        (height, width),
        150,
        dtype=np.uint8
    )

    memory_driver = gdal.GetDriverByName(
        'MEM'
    )

    memory_dataset = memory_driver.Create(
        '',
        width,
        height,
        3,
        gdal.GDT_Byte
    )

    if memory_dataset is None:
        raise RuntimeError(
            'Could not create uniform surface texture.'
        )

    memory_dataset.GetRasterBand(1).WriteArray(red)
    memory_dataset.GetRasterBand(2).WriteArray(green)
    memory_dataset.GetRasterBand(3).WriteArray(blue)

    memory_dataset.GetRasterBand(
        1
    ).SetColorInterpretation(
        gdal.GCI_RedBand
    )

    memory_dataset.GetRasterBand(
        2
    ).SetColorInterpretation(
        gdal.GCI_GreenBand
    )

    memory_dataset.GetRasterBand(
        3
    ).SetColorInterpretation(
        gdal.GCI_BlueBand
    )

    png_driver = gdal.GetDriverByName(
        'PNG'
    )

    output_dataset = png_driver.CreateCopy(
        output_png,
        memory_dataset,
        strict=1
    )

    if output_dataset is None:
        memory_dataset = None

        raise RuntimeError(
            'Could not save uniform surface texture.'
        )

    output_dataset.FlushCache()

    output_dataset = None
    memory_dataset = None


def create_world(
    output_world,
    heightmap_png,
    surface_texture_png,
    target_x,
    target_y,
    target_z,
    sampling
):
    """
    Create a Gazebo world using the generated heightmap.

    The visual uses one neutral grey texture. Strong shadows are
    disabled so that the terrain geometry can be inspected clearly.
    """

    heightmap_uri = file_uri(
        heightmap_png
    )

    surface_texture_uri = file_uri(
        surface_texture_png
    )

    longest_side = max(
        target_x,
        target_y
    )

    camera_distance = (
        longest_side * 1.15
    )

    camera_height = max(
        longest_side * 0.75,
        target_z * 4.0,
        12.0
    )

    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="dem_inspired_benchmark_world">

    <gravity>0 0 -9.81</gravity>

    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.75 0.75 0.75 1</ambient>
      <background>0.82 0.82 0.84 1</background>
      <shadows>false</shadows>
    </scene>

    <light name="terrain_light" type="directional">
      <cast_shadows>false</cast_shadows>

      <pose>0 0 30 0 0 0</pose>

      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>

      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>

      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>
          {-camera_distance:.6f}
          {-camera_distance:.6f}
          {camera_height:.6f}
          0 0.48 0.78
        </pose>

        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <model name="dem_inspired_terrain">
      <static>true</static>

      <link name="terrain_link">

        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_uri}</uri>

              <size>
                {target_x:.6f}
                {target_y:.6f}
                {target_z:.6f}
              </size>

              <pos>0 0 0</pos>
              <sampling>{sampling}</sampling>
            </heightmap>
          </geometry>

          <surface>
            <friction>
              <ode>
                <mu>0.9</mu>
                <mu2>0.9</mu2>
              </ode>
            </friction>

            <contact>
              <ode>
                <kp>1000000</kp>
                <kd>1</kd>
              </ode>
            </contact>
          </surface>
        </collision>

        <visual name="terrain_visual">

          <cast_shadows>false</cast_shadows>

          <geometry>
            <heightmap>
              <uri>{heightmap_uri}</uri>

              <size>
                {target_x:.6f}
                {target_y:.6f}
                {target_z:.6f}
              </size>

              <pos>0 0 0</pos>
              <sampling>{sampling}</sampling>

              <!--
                Uniform neutral surface only.
                This is not a geographic or hillshade texture.
              -->
              <texture>
                <diffuse>{surface_texture_uri}</diffuse>

                <normal>
                  file://media/materials/textures/flat_normal.png
                </normal>

                <size>{longest_side:.6f}</size>
              </texture>
            </heightmap>
          </geometry>

        </visual>

      </link>
    </model>

  </world>

</sdf>
"""

    os.makedirs(
        os.path.dirname(output_world),
        exist_ok=True
    )

    with open(
        output_world,
        'w',
        encoding='utf-8'
    ) as file:
        file.write(world_content)


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Generate a robot-scale DEM-inspired Gazebo terrain. '
            'The source DEM provides terrain morphology, while '
            'horizontal size, relief, and slope are normalised.'
        )
    )

    parser.add_argument(
        '--input-dem',
        required=True,
        help='Input SnowEx GeoTIFF DEM'
    )

    parser.add_argument(
        '--output-heightmap',
        required=True,
        help='Output Gazebo 8-bit grayscale heightmap PNG'
    )

    parser.add_argument(
        '--output-world',
        required=True,
        help='Output Gazebo world file'
    )

    parser.add_argument(
        '--output-metadata',
        required=True,
        help='Output JSON processing metadata'
    )

    parser.add_argument(
        '--samples',
        type=int,
        default=513,
        help=(
            'Heightmap size. Use 129, 257, 513, or 1025. '
            'Default: 513'
        )
    )

    parser.add_argument(
        '--target-longest-side',
        type=float,
        default=30.0,
        help='Longest Gazebo terrain side in metres'
    )

    parser.add_argument(
        '--target-relief',
        type=float,
        default=2.2,
        help='Requested maximum terrain relief in metres'
    )

    parser.add_argument(
        '--max-slope-deg',
        type=float,
        default=12.0,
        help='Maximum allowed 95th-percentile slope'
    )

    parser.add_argument(
        '--smooth-iterations',
        type=int,
        default=2,
        help='Medium-scale Gaussian smoothing parameter'
    )

    parser.add_argument(
        '--lower-percentile',
        type=float,
        default=3.0,
        help='Lower elevation clipping percentile'
    )

    parser.add_argument(
        '--upper-percentile',
        type=float,
        default=97.0,
        help='Upper elevation clipping percentile'
    )

    parser.add_argument(
        '--sampling',
        type=int,
        default=1,
        help='Gazebo heightmap sampling value'
    )

    args = parser.parse_args()

    input_dem = expand_path(
        args.input_dem
    )

    output_heightmap = expand_path(
        args.output_heightmap
    )

    output_world = expand_path(
        args.output_world
    )

    output_metadata = expand_path(
        args.output_metadata
    )

    if not os.path.isfile(input_dem):
        print(
            f'ERROR: Input DEM not found: {input_dem}'
        )
        sys.exit(1)

    if args.samples not in [
        129,
        257,
        513,
        1025
    ]:
        print(
            'ERROR: --samples must be one of '
            '129, 257, 513, or 1025.'
        )
        sys.exit(1)

    if args.target_longest_side <= 0.0:
        print(
            'ERROR: --target-longest-side must be positive.'
        )
        sys.exit(1)

    if args.target_relief <= 0.0:
        print(
            'ERROR: --target-relief must be positive.'
        )
        sys.exit(1)

    if args.max_slope_deg <= 0.0:
        print(
            'ERROR: --max-slope-deg must be positive.'
        )
        sys.exit(1)

    try:
        elevation, valid, source_info = (
            read_valid_dem(input_dem)
        )

        filled = fill_internal_invalid_values(
            elevation,
            valid
        )

        # Reconstruct a medium-resolution terrain surface.
        # 129 × 129 preserves broad valleys and ridges while removing
        # narrow LiDAR scanning artefacts.
        if args.samples >= 257:
            coarse_samples = 129
        else:
            coarse_samples = 65

        coarse_terrain = resample_array(
            filled,
            coarse_samples
        )

        coarse_smoothed = mean_smooth(
            coarse_terrain,
            args.smooth_iterations
        )

        # Restore to a Gazebo-compatible 2^n + 1 resolution.
        reconstructed = resample_array(
            coarse_smoothed,
            args.samples
        )

        normalised, lower, upper = (
            normalise_elevation(
                reconstructed,
                args.lower_percentile,
                args.upper_percentile
            )
        )

        target_x, target_y = (
            calculate_world_dimensions(
                source_info,
                args.target_longest_side
            )
        )

        (
            final_relief,
            slope_statistics,
            vertical_factor
        ) = limit_relief_for_slope(
            normalised,
            target_x,
            target_y,
            args.target_relief,
            args.max_slope_deg
        )

        save_heightmap_png(
            normalised,
            output_heightmap
        )

        surface_texture = os.path.join(
            os.path.dirname(output_heightmap),
            'dem_uniform_gray_surface.png'
        )

        save_uniform_surface_texture(
            surface_texture
        )

        create_world(
            output_world,
            output_heightmap,
            surface_texture,
            target_x,
            target_y,
            final_relief,
            args.sampling
        )

    except Exception as error:
        print(f'ERROR: {error}')
        sys.exit(1)

    metadata = {
        'terrain_type':
            'DEM-inspired normalised benchmark terrain',

        'direct_geographic_scaling':
            False,

        'terrain_texture_used':
            False,

        'uniform_surface_material_used':
            True,

        'relative_morphology_preserved':
            True,

        'source_dem':
            input_dem,

        'source_information':
            source_info,

        'processing': {
            'output_samples':
                args.samples,

            'coarse_reconstruction_samples':
                coarse_samples,

            'smooth_iterations':
                args.smooth_iterations,

            'gaussian_sigma':
                max(
                    0.8,
                    float(
                        args.smooth_iterations
                    ) * 0.6
                ),

            'lower_percentile':
                args.lower_percentile,

            'upper_percentile':
                args.upper_percentile,

            'normalisation_min_m':
                lower,

            'normalisation_max_m':
                upper,

            'nodata_fill_method':
                'nearest_valid_elevation',

            'median_filter_size':
                3,

            'heightmap_bit_depth':
                8
        },

        'gazebo': {
            'target_x_m':
                target_x,

            'target_y_m':
                target_y,

            'requested_relief_m':
                args.target_relief,

            'final_relief_m':
                final_relief,

            'maximum_95_percentile_slope_deg':
                args.max_slope_deg,

            'vertical_factor':
                vertical_factor,

            'heightmap_sampling':
                args.sampling,

            'slope_statistics':
                slope_statistics
        },

        'output_heightmap':
            output_heightmap,

        'surface_texture':
            surface_texture,

        'output_world':
            output_world
    }

    os.makedirs(
        os.path.dirname(output_metadata),
        exist_ok=True
    )

    with open(
        output_metadata,
        'w',
        encoding='utf-8'
    ) as file:
        json.dump(
            metadata,
            file,
            indent=2
        )

    print()
    print(
        'DEM-inspired terrain generated successfully.'
    )

    print(
        'Direct geographic scaling: False'
    )

    print(
        'Relative terrain morphology preserved: True'
    )

    print(
        'Geographic or hillshade texture used: False'
    )

    print(
        f'Coarse reconstruction: '
        f'{coarse_samples} × {coarse_samples}'
    )

    print(
        f'Final heightmap: '
        f'{args.samples} × {args.samples}'
    )

    print(
        f'Gazebo dimensions: '
        f'{target_x:.3f} m × '
        f'{target_y:.3f} m × '
        f'{final_relief:.3f} m'
    )

    print(
        f"Mean slope: "
        f"{slope_statistics['slope_mean_deg']:.3f} deg"
    )

    print(
        f"95% slope: "
        f"{slope_statistics['slope_95_deg']:.3f} deg"
    )

    print(
        f"Maximum slope: "
        f"{slope_statistics['slope_max_deg']:.3f} deg"
    )

    print(
        f'Heightmap: {output_heightmap}'
    )

    print(
        f'Uniform surface: {surface_texture}'
    )

    print(
        f'World: {output_world}'
    )

    print(
        f'Metadata: {output_metadata}'
    )


if __name__ == '__main__':
    main()
