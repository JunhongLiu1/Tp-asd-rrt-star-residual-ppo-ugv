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


def to_file_uri(path):
    return 'file://' + expand_path(path)


def get_source_dimensions(dataset):
    """
    Calculate the complete physical width and height of the input DEM.

    SnowEx DEM uses UTM coordinates, therefore the raster coordinate
    units are metres.
    """
    transform = dataset.GetGeoTransform()

    # Physical distance represented by one raster column.
    pixel_x_m = math.hypot(
        float(transform[1]),
        float(transform[4])
    )

    # Physical distance represented by one raster row.
    pixel_y_m = math.hypot(
        float(transform[2]),
        float(transform[5])
    )

    raster_width = dataset.RasterXSize
    raster_height = dataset.RasterYSize

    source_width_m = raster_width * pixel_x_m
    source_height_m = raster_height * pixel_y_m

    return {
        'raster_width': raster_width,
        'raster_height': raster_height,
        'pixel_x_m': pixel_x_m,
        'pixel_y_m': pixel_y_m,
        'source_width_m': source_width_m,
        'source_height_m': source_height_m
    }


def create_full_resampled_dem(
    source_dataset,
    output_tif,
    samples
):
    """
    Resample the complete DEM without cropping.

    Valid LiDAR elevation is retained.
    Small internal holes are interpolated.
    Large external NoData regions are set to the minimum elevation
    instead of being extrapolated across the complete raster.
    """

    source_band = source_dataset.GetRasterBand(1)
    source_nodata = source_band.GetNoDataValue()

    warp_arguments = {
        'format': 'MEM',
        'width': samples,
        'height': samples,

        # Bilinear interpolation produces fewer edge artefacts
        # than cubic interpolation around an irregular LiDAR mask.
        'resampleAlg': 'bilinear',

        'dstNodata': -9999.0,
        'multithread': True
    }

    if source_nodata is not None:
        warp_arguments['srcNodata'] = source_nodata

    resampled_dataset = gdal.Warp(
        '',
        source_dataset,
        **warp_arguments
    )

    if resampled_dataset is None:
        raise RuntimeError(
            'GDAL could not resample the complete DEM.'
        )

    resampled_band = resampled_dataset.GetRasterBand(1)
    resampled_band.SetNoDataValue(-9999.0)

    # Only fill very small internal gaps.
    # Do not use a large value such as 100 because it extrapolates
    # terrain into the external NoData area.
    try:
        gdal.FillNodata(
            targetBand=resampled_band,
            maskBand=None,
            maxSearchDist=3,
            smoothingIterations=0
        )
    except Exception as error:
        print(
            'Warning: small-hole interpolation was skipped:',
            error
        )

    elevation = resampled_band.ReadAsArray().astype(
        np.float32
    )

    valid = np.isfinite(elevation)
    valid &= elevation != -9999.0
    valid &= elevation > -32000.0

    if not np.any(valid):
        raise RuntimeError(
            'The resampled DEM contains no valid elevation cells.'
        )

    valid_values = elevation[valid]

    elevation_min = float(np.min(valid_values))
    elevation_max = float(np.max(valid_values))
    elevation_mean = float(np.mean(valid_values))

    # Important:
    # Do not fill external NoData with the median elevation.
    # Set the external area to the minimum terrain elevation.
    elevation[~valid] = elevation_min

    # Move minimum terrain elevation to z = 0.
    elevation -= elevation_min

    processed_min = float(np.min(elevation))
    processed_max = float(np.max(elevation))
    relief = processed_max - processed_min

    output_directory = os.path.dirname(output_tif)
    os.makedirs(output_directory, exist_ok=True)

    driver = gdal.GetDriverByName('GTiff')

    output_dataset = driver.Create(
        output_tif,
        samples,
        samples,
        1,
        gdal.GDT_Float32,
        options=[
            'COMPRESS=LZW',
            'TILED=YES'
        ]
    )

    if output_dataset is None:
        raise RuntimeError(
            f'Could not create output DEM: {output_tif}'
        )

    output_dataset.SetGeoTransform(
        resampled_dataset.GetGeoTransform()
    )

    projection = resampled_dataset.GetProjection()

    if projection:
        output_dataset.SetProjection(projection)

    output_band = output_dataset.GetRasterBand(1)
    output_band.WriteArray(elevation)

    # All external pixels now contain valid flat elevation zero.
    output_band.DeleteNoDataValue()

    output_band.SetDescription(
        'Complete cleaned zero-based SnowEx elevation'
    )

    output_band.FlushCache()
    output_dataset.FlushCache()

    valid_percentage = (
        100.0 * np.count_nonzero(valid) / valid.size
    )

    output_band = None
    output_dataset = None
    resampled_band = None
    resampled_dataset = None

    return {
        'original_min_m': elevation_min,
        'original_max_m': elevation_max,
        'original_mean_m': elevation_mean,
        'processed_min_m': processed_min,
        'processed_max_m': processed_max,
        'elevation_relief_m': relief,
        'valid_data_percentage': valid_percentage,
        'external_nodata_handling':
            'set_to_minimum_elevation',
        'small_hole_fill_distance_pixels': 3,
        'resampling_method': 'bilinear'
    }
def generate_hillshade(
    input_tif,
    output_png
):
    """
    Generate a preview from exactly the same complete DEM that is used
    by Gazebo.
    """
    preview_directory = os.path.dirname(output_png)
    os.makedirs(preview_directory, exist_ok=True)

    temporary_tif = os.path.splitext(
        output_png
    )[0] + '_temporary.tif'

    hillshade_options = gdal.DEMProcessingOptions(
        computeEdges=True,
        azimuth=315.0,
        altitude=45.0
    )

    result = gdal.DEMProcessing(
        temporary_tif,
        input_tif,
        'hillshade',
        options=hillshade_options
    )

    if result is None:
        raise RuntimeError(
            'Could not generate complete DEM hillshade.'
        )

    result = None

    png_options = gdal.TranslateOptions(
        format='PNG'
    )

    result = gdal.Translate(
        output_png,
        temporary_tif,
        options=png_options
    )

    if result is None:
        raise RuntimeError(
            'Could not convert hillshade to PNG.'
        )

    result = None

    if os.path.exists(temporary_tif):
        os.remove(temporary_tif)


def calculate_scaled_dimensions(
    source_information,
    elevation_information,
    target_longest_side
):
    """
    Apply one common scale factor to X, Y and Z.

    This preserves:
    - terrain aspect ratio;
    - relative elevation;
    - original slope angles;
    - valley and ridge geometry.
    """
    source_width = source_information[
        'source_width_m'
    ]

    source_height = source_information[
        'source_height_m'
    ]

    source_longest_side = max(
        source_width,
        source_height
    )

    uniform_scale = (
        target_longest_side /
        source_longest_side
    )

    target_x = source_width * uniform_scale
    target_y = source_height * uniform_scale

    target_z = (
        elevation_information['elevation_relief_m']
        * uniform_scale
    )

    if target_z <= 0.0:
        target_z = 0.05

    return {
        'uniform_scale': uniform_scale,
        'target_x_m': target_x,
        'target_y_m': target_y,
        'target_z_m': target_z
    }


def generate_world(
    output_world,
    dem_file,
    hillshade_file,
    dimensions,
    sampling
):
    target_x = dimensions['target_x_m']
    target_y = dimensions['target_y_m']
    target_z = dimensions['target_z_m']

    longest_side = max(target_x, target_y)

    camera_distance = longest_side * 1.45

    camera_height = max(
        longest_side * 1.05,
        target_z * 3.0,
        10.0
    )

    dem_uri = to_file_uri(dem_file)
    hillshade_uri = to_file_uri(hillshade_file)

    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="module33_full_snowex_scaled_world">

    <include>
      <uri>model://sun</uri>
    </include>

    <gravity>0 0 -9.81</gravity>

    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>
          {-camera_distance:.6f}
          {-camera_distance:.6f}
          {camera_height:.6f}
          0 0.52 0.78
        </pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <model name="complete_scaled_snowex_terrain">
      <static>true</static>

      <link name="terrain_link">

        <collision name="terrain_collision">
          <geometry>
            <heightmap>

              <uri>{dem_uri}</uri>

              <!--
                X, Y and Z were generated using one common
                scale factor. The full source DEM is retained.
              -->
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
          <geometry>
            <heightmap>

              <uri>{dem_uri}</uri>

              <size>
                {target_x:.6f}
                {target_y:.6f}
                {target_z:.6f}
              </size>

              <pos>0 0 0</pos>
              <sampling>{sampling}</sampling>

              <!--
                The hillshade was generated from the same complete
                processed DEM and is used only as a visual texture.
              -->
              <texture>
                <diffuse>{hillshade_uri}</diffuse>

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

    world_directory = os.path.dirname(output_world)
    os.makedirs(world_directory, exist_ok=True)

    with open(
        output_world,
        'w',
        encoding='utf-8'
    ) as world_file:
        world_file.write(world_content)


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Use the complete SnowEx DEM, resample its complete '
            'extent, and uniformly scale X, Y and Z for Gazebo.'
        )
    )

    parser.add_argument(
        '--input-dem',
        required=True,
        help='Complete original SnowEx GeoTIFF'
    )

    parser.add_argument(
        '--output-dem',
        required=True,
        help='Complete zero-based resampled GeoTIFF'
    )

    parser.add_argument(
        '--output-hillshade',
        required=True,
        help='Hillshade PNG generated from the complete output DEM'
    )

    parser.add_argument(
        '--output-world',
        required=True,
        help='Generated Gazebo world file'
    )

    parser.add_argument(
        '--output-metadata',
        required=True,
        help='JSON file containing complete scaling information'
    )

    parser.add_argument(
        '--samples',
        type=int,
        default=513,
        help=(
            'Output raster resolution. The complete extent is '
            'preserved. Default: 513'
        )
    )

    parser.add_argument(
        '--target-longest-side',
        type=float,
        default=20.0,
        help=(
            'Longest horizontal Gazebo terrain dimension in metres. '
            'Default: 20.0'
        )
    )

    parser.add_argument(
        '--sampling',
        type=int,
        default=1,
        help='Gazebo heightmap sampling. Default: 1'
    )

    args = parser.parse_args()

    gdal.UseExceptions()

    input_dem = expand_path(args.input_dem)
    output_dem = expand_path(args.output_dem)
    output_hillshade = expand_path(
        args.output_hillshade
    )
    output_world = expand_path(args.output_world)
    output_metadata = expand_path(
        args.output_metadata
    )

    if not os.path.isfile(input_dem):
        print(
            f'ERROR: Input DEM does not exist: {input_dem}'
        )
        sys.exit(1)

    if args.samples < 3:
        print('ERROR: --samples must be at least 3.')
        sys.exit(1)

    if args.target_longest_side <= 0.0:
        print(
            'ERROR: --target-longest-side must be positive.'
        )
        sys.exit(1)

    source_dataset = gdal.Open(
        input_dem,
        gdal.GA_ReadOnly
    )

    if source_dataset is None:
        print(
            f'ERROR: GDAL could not open: {input_dem}'
        )
        sys.exit(1)

    try:
        source_information = get_source_dimensions(
            source_dataset
        )

        elevation_information = create_full_resampled_dem(
            source_dataset,
            output_dem,
            args.samples
        )

        source_dataset = None

        generate_hillshade(
            output_dem,
            output_hillshade
        )

        dimensions = calculate_scaled_dimensions(
            source_information,
            elevation_information,
            args.target_longest_side
        )

        generate_world(
            output_world,
            output_dem,
            output_hillshade,
            dimensions,
            args.sampling
        )

    except Exception as error:
        source_dataset = None
        print(f'ERROR: {error}')
        sys.exit(1)

    metadata = {
        'processing': {
            'cropping_used': False,
            'complete_source_extent_retained': True,
            'resampled_raster_size': [
                args.samples,
                args.samples
            ]
        },
        'input_dem': input_dem,
        'output_dem': output_dem,
        'output_hillshade': output_hillshade,
        'output_world': output_world,
        'source': source_information,
        'elevation': elevation_information,
        'gazebo': dimensions,
        'heightmap_sampling': args.sampling
    }

    os.makedirs(
        os.path.dirname(output_metadata),
        exist_ok=True
    )

    with open(
        output_metadata,
        'w',
        encoding='utf-8'
    ) as metadata_file:
        json.dump(
            metadata,
            metadata_file,
            indent=2
        )

    print()
    print('Complete SnowEx DEM processing finished.')
    print('Cropping used: False')
    print('Complete source extent retained: True')
    print()
    print(
        'Original raster: '
        f"{source_information['raster_width']} × "
        f"{source_information['raster_height']}"
    )
    print(
        'Original physical dimensions: '
        f"{source_information['source_width_m']:.3f} m × "
        f"{source_information['source_height_m']:.3f} m"
    )
    print(
        'Original elevation relief: '
        f"{elevation_information['elevation_relief_m']:.3f} m"
    )
    print(
        'Uniform scale factor: '
        f"{dimensions['uniform_scale']:.8f}"
    )
    print(
        'Gazebo terrain dimensions: '
        f"{dimensions['target_x_m']:.3f} m × "
        f"{dimensions['target_y_m']:.3f} m × "
        f"{dimensions['target_z_m']:.3f} m"
    )
    print()
    print(f'Processed DEM: {output_dem}')
    print(f'Hillshade: {output_hillshade}')
    print(f'Gazebo world: {output_world}')
    print(f'Metadata: {output_metadata}')


if __name__ == '__main__':
    main()
