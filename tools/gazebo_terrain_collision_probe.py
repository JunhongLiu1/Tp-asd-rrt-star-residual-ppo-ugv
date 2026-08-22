#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

import rclpy
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


def expand(path):
    return Path(os.path.expanduser(path)).resolve()


def local_name(tag):
    return tag.rsplit('}', 1)[-1]


def first_child_by_local_name(parent, name):
    for child in list(parent):
        if local_name(child.tag) == name:
            return child
    return None


def parse_numbers(text, expected=None):
    values = [float(v) for v in (text or '').split()]
    if expected is not None and len(values) != expected:
        raise RuntimeError(f'Expected {expected} numbers, got {values}')
    return values


def parse_world_heightmap(world_path):
    root = ET.parse(world_path).getroot()

    selected = None
    for element in root.iter():
        if local_name(element.tag) != 'collision':
            continue
        geometry = first_child_by_local_name(element, 'geometry')
        if geometry is None:
            continue
        heightmap = first_child_by_local_name(geometry, 'heightmap')
        if heightmap is not None:
            selected = heightmap
            break

    if selected is None:
        raise RuntimeError('No collision heightmap found in world file')

    uri_node = first_child_by_local_name(selected, 'uri')
    size_node = first_child_by_local_name(selected, 'size')
    pos_node = first_child_by_local_name(selected, 'pos')

    if uri_node is None or size_node is None:
        raise RuntimeError('Collision heightmap is missing uri or size')

    uri = (uri_node.text or '').strip()
    if uri.startswith('file://'):
        image_path = Path(uri[7:]).expanduser().resolve()
    else:
        image_path = Path(uri).expanduser().resolve()

    size = parse_numbers(size_node.text, 3)
    pos = parse_numbers(pos_node.text if pos_node is not None else '0 0 0', 3)

    return {
        'image_path': image_path,
        'size_x': size[0],
        'size_y': size[1],
        'size_z': size[2],
        'pos_x': pos[0],
        'pos_y': pos[1],
        'pos_z': pos[2],
    }


def bilinear(array, row, column):
    rows, columns = array.shape
    if not (0.0 <= row <= rows - 1 and 0.0 <= column <= columns - 1):
        return float('nan')

    r0 = int(math.floor(row))
    c0 = int(math.floor(column))
    r1 = min(r0 + 1, rows - 1)
    c1 = min(c0 + 1, columns - 1)
    dr = row - r0
    dc = column - c0

    return float(
        array[r0, c0] * (1.0 - dr) * (1.0 - dc)
        + array[r1, c0] * dr * (1.0 - dc)
        + array[r0, c1] * (1.0 - dr) * dc
        + array[r1, c1] * dr * dc
    )


def sample_png(height_pixels, config, x, y, flip_x=False, flip_y=False):
    x_local = x - config['pos_x']
    y_local = y - config['pos_y']

    tx = (x_local + config['size_x'] / 2.0) / config['size_x']
    ty = (config['size_y'] / 2.0 - y_local) / config['size_y']

    if flip_x:
        tx = 1.0 - tx
    if flip_y:
        ty = 1.0 - ty

    column = tx * (height_pixels.shape[1] - 1)
    row = ty * (height_pixels.shape[0] - 1)
    normalised = bilinear(height_pixels, row, column) / 255.0

    if not math.isfinite(normalised):
        return float('nan')

    return config['pos_z'] + normalised * config['size_z']


def sample_npz(elevation, metadata, x, y):
    grid = metadata['grid']
    resolution = float(grid['resolution_m'])
    origin_x = float(grid['origin_x_m'])
    origin_y = float(grid['origin_y_m'])
    width = int(grid['width_cells'])
    height = int(grid['height_cells'])

    ros_column = (x - origin_x) / resolution
    ros_row = (y - origin_y) / resolution
    source_row = (height - 1) - ros_row

    return bilinear(elevation, source_row, ros_column)


def fit_metrics(measured, predicted):
    measured = np.asarray(measured, dtype=np.float64)
    predicted = np.asarray(predicted, dtype=np.float64)
    valid = np.isfinite(measured) & np.isfinite(predicted)
    measured = measured[valid]
    predicted = predicted[valid]

    if measured.size < 3:
        return None

    residual = measured - predicted
    raw_rmse = float(np.sqrt(np.mean(residual ** 2)))
    raw_mae = float(np.mean(np.abs(residual)))
    bias = float(np.mean(residual))

    matrix = np.column_stack([predicted, np.ones(predicted.size)])
    scale, offset = np.linalg.lstsq(matrix, measured, rcond=None)[0]
    fitted = scale * predicted + offset
    affine_rmse = float(np.sqrt(np.mean((measured - fitted) ** 2)))
    correlation = float(np.corrcoef(measured, predicted)[0, 1])

    return {
        'sample_count': int(measured.size),
        'raw_rmse_m': raw_rmse,
        'raw_mae_m': raw_mae,
        'mean_bias_measured_minus_predicted_m': bias,
        'correlation': correlation,
        'affine_scale': float(scale),
        'affine_offset_m': float(offset),
        'affine_rmse_m': affine_rmse,
    }


def make_probe_sdf(name, radius):
    mass = 0.02
    inertia = 0.4 * mass * radius * radius
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <allow_auto_disable>false</allow_auto_disable>
    <link name="anchor">
      <kinematic>true</kinematic>
      <gravity>false</gravity>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1</ixx><iyy>1</iyy><izz>1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <link name="probe_link">
      <gravity>true</gravity>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="probe_collision">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          <contact><ode><kp>10000000</kp><kd>1000</kd><max_vel>0.05</max_vel><min_depth>0</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="probe_visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>1 0 1 1</ambient><diffuse>1 0 1 1</diffuse><emissive>0.3 0 0.3 1</emissive>
        </material>
      </visual>
    </link>
    <joint name="vertical_joint" type="prismatic">
      <parent>anchor</parent>
      <child>probe_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-4.0</lower><upper>0.0</upper></limit>
        <dynamics><damping>0.10</damping><friction>0.0</friction></dynamics>
      </axis>
    </joint>
  </model>
</sdf>'''


class ProbeNode(Node):
    def __init__(self):
        super().__init__('gazebo_terrain_collision_probe')
        self.spawn_name = self.find_service('spawn_entity', 'gazebo_msgs/srv/SpawnEntity')
        self.delete_name = self.find_service('delete_entity', 'gazebo_msgs/srv/DeleteEntity')
        self.get_name = self.find_service('get_entity_state', 'gazebo_msgs/srv/GetEntityState')

        self.spawn_client = self.create_client(SpawnEntity, self.spawn_name)
        self.delete_client = self.create_client(DeleteEntity, self.delete_name)
        self.get_client = self.create_client(GetEntityState, self.get_name)

        for client, name in (
            (self.spawn_client, self.spawn_name),
            (self.delete_client, self.delete_name),
            (self.get_client, self.get_name),
        ):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'Gazebo service unavailable: {name}')

    def find_service(self, suffix, required_type):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            for name, types in self.get_service_names_and_types():
                if name.rstrip('/').endswith('/' + suffix) or name == '/' + suffix:
                    if required_type in types:
                        return name
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f'Cannot find Gazebo service ending with /{suffix}')

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Gazebo service call timed out or failed')
        return future.result()

    def spawn_probe(self, name, x, y, start_z, radius):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = make_probe_sdf(name, radius)
        request.robot_namespace = ''
        request.reference_frame = 'world'
        request.initial_pose = Pose()
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(start_z)
        request.initial_pose.orientation.w = 1.0
        response = self.call(self.spawn_client, request)
        if not response.success:
            raise RuntimeError(f'Spawn failed for {name}: {response.status_message}')

    def delete_probe(self, name):
        request = DeleteEntity.Request()
        request.name = name
        try:
            self.call(self.delete_client, request, timeout=5.0)
        except Exception:
            pass

    def get_probe_pose(self, name):
        request = GetEntityState.Request()
        request.name = f'{name}::probe_link'
        request.reference_frame = 'world'
        response = self.call(self.get_client, request)
        if not response.success:
            request.name = name
            response = self.call(self.get_client, request)
        if not response.success:
            raise RuntimeError(f'Cannot read state for {name}')
        return response.state.pose, response.state.twist


def classify(metrics_by_variant, npz_metrics):
    ranked = sorted(
        ((name, values) for name, values in metrics_by_variant.items() if values),
        key=lambda item: item[1]['raw_rmse_m']
    )
    if not ranked:
        return 'INCONCLUSIVE_NO_VALID_PROBES', []

    best_name, best = ranked[0]
    reasons = [
        f'Best PNG mapping: {best_name}',
        f"raw RMSE={best['raw_rmse_m'] * 1000.0:.2f} mm",
        f"bias={best['mean_bias_measured_minus_predicted_m'] * 1000.0:.2f} mm",
        f"scale={best['affine_scale']:.6f}",
        f"offset={best['affine_offset_m'] * 1000.0:.2f} mm",
    ]

    identity = metrics_by_variant.get('identity')

    if (
        best_name == 'identity'
        and best['raw_rmse_m'] <= 0.015
        and abs(best['mean_bias_measured_minus_predicted_m']) <= 0.015
        and npz_metrics
        and npz_metrics['raw_rmse_m'] <= 0.020
    ):
        return 'GAZEBO_COLLISION_MATCHES_PNG_AND_NPZ', reasons

    if best_name != 'identity' and best['raw_rmse_m'] <= 0.020:
        return f'HEIGHTMAP_ORIENTATION_MISMATCH_{best_name.upper()}', reasons

    if (
        best['affine_rmse_m'] <= 0.015
        and (
            abs(best['affine_scale'] - 1.0) > 0.02
            or abs(best['affine_offset_m']) > 0.02
        )
    ):
        return 'HEIGHTMAP_Z_SCALE_OR_OFFSET_MISMATCH', reasons

    return 'GAZEBO_COLLISION_DOES_NOT_MATCH_FILE_PREDICTION', reasons


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='~/terrain_radiation_ws/src/radiation_mapping/worlds/module36_hard_radiation_plugin.world')
    parser.add_argument('--npz', default='~/terrain_radiation_ws/src/radiation_mapping/dem/processed/terrain_layers_hard.npz')
    parser.add_argument('--metadata', default='~/terrain_radiation_ws/src/radiation_mapping/dem/processed/terrain_layers_hard_metadata.json')
    parser.add_argument('--center-x', type=float, default=-1.13)
    parser.add_argument('--center-y', type=float, default=-7.80)
    parser.add_argument('--spacing', type=float, default=0.40)
    parser.add_argument('--grid-radius', type=int, default=2)
    parser.add_argument('--start-z', type=float, default=2.5)
    parser.add_argument('--probe-radius', type=float, default=0.015)
    parser.add_argument('--settle-time', type=float, default=6.0)
    parser.add_argument('--output-root', default='~/terrain_radiation_ws/diagnostics')
    args = parser.parse_args()

    world_path = expand(args.world)
    npz_path = expand(args.npz)
    metadata_path = expand(args.metadata)
    output_root = expand(args.output_root)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    output_dir = output_root / f'gazebo_collision_probe_{stamp}'
    output_dir.mkdir(parents=True, exist_ok=True)

    config = parse_world_heightmap(world_path)
    image = np.asarray(Image.open(config['image_path']).convert('L'), dtype=np.float64)

    with np.load(npz_path) as archive:
        elevation = np.asarray(archive['elevation_m'], dtype=np.float64)

    with metadata_path.open('r', encoding='utf-8') as handle:
        metadata = json.load(handle)

    points = []
    for iy in range(-args.grid_radius, args.grid_radius + 1):
        for ix in range(-args.grid_radius, args.grid_radius + 1):
            points.append({
                'ix': ix,
                'iy': iy,
                'x': args.center_x + ix * args.spacing,
                'y': args.center_y + iy * args.spacing,
            })

    rclpy.init()
    node = None
    spawned = []
    try:
        node = ProbeNode()
        print(f'[probe] Spawn service: {node.spawn_name}')
        print(f'[probe] State service: {node.get_name}')
        print(f'[probe] Spawning {len(points)} vertically constrained probes...')

        for index, point in enumerate(points):
            name = f'terrain_probe_{stamp}_{index:02d}'
            point['name'] = name
            node.spawn_probe(name, point['x'], point['y'], args.start_z, args.probe_radius)
            spawned.append(name)

        print(f'[probe] Waiting {args.settle_time:.1f} s for contact settling...')
        time.sleep(args.settle_time)

        first_z = {}
        for point in points:
            pose, twist = node.get_probe_pose(point['name'])
            first_z[point['name']] = pose.position.z

        time.sleep(1.0)

        for point in points:
            pose, twist = node.get_probe_pose(point['name'])
            point['actual_x'] = float(pose.position.x)
            point['actual_y'] = float(pose.position.y)
            point['probe_center_z'] = float(pose.position.z)
            point['measured_collision_height_m'] = float(pose.position.z - args.probe_radius)
            point['z_change_last_1s_m'] = float(pose.position.z - first_z[point['name']])
            point['linear_speed_m_s'] = float(math.sqrt(
                twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2
            ))
            point['xy_error_m'] = float(math.hypot(
                pose.position.x - point['x'], pose.position.y - point['y']
            ))

            for variant, flip_x, flip_y in (
                ('identity', False, False),
                ('flip_x', True, False),
                ('flip_y', False, True),
                ('flip_xy', True, True),
            ):
                point[f'png_{variant}_m'] = sample_png(
                    image, config, point['x'], point['y'], flip_x=flip_x, flip_y=flip_y
                )

            point['npz_identity_m'] = sample_npz(elevation, metadata, point['x'], point['y'])

    finally:
        if node is not None:
            for name in spawned:
                node.delete_probe(name)
            node.destroy_node()
        rclpy.shutdown()

    measured = [p['measured_collision_height_m'] for p in points]
    metrics_by_variant = {}
    for variant in ('identity', 'flip_x', 'flip_y', 'flip_xy'):
        metrics_by_variant[variant] = fit_metrics(
            measured, [p[f'png_{variant}_m'] for p in points]
        )

    npz_metrics = fit_metrics(measured, [p['npz_identity_m'] for p in points])
    verdict, reasons = classify(metrics_by_variant, npz_metrics)

    result = {
        'verdict': verdict,
        'reasons': reasons,
        'arguments': vars(args),
        'world_heightmap': {k: str(v) if isinstance(v, Path) else v for k, v in config.items()},
        'png_mapping_metrics': metrics_by_variant,
        'npz_identity_metrics': npz_metrics,
        'maximum_xy_constraint_error_m': max(p['xy_error_m'] for p in points),
        'maximum_probe_z_change_last_1s_m': max(abs(p['z_change_last_1s_m']) for p in points),
        'points': points,
    }

    json_path = output_dir / 'result.json'
    csv_path = output_dir / 'samples.csv'
    report_path = output_dir / 'report.txt'

    with json_path.open('w', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2)

    fieldnames = list(points[0].keys())
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(points)

    lines = [
        '=' * 76,
        f'VERDICT: {verdict}',
        '=' * 76,
        *reasons,
        '',
        'PNG mapping metrics:',
    ]
    for name, values in metrics_by_variant.items():
        if values:
            lines.append(
                f"  {name}: RMSE={values['raw_rmse_m']*1000:.2f} mm, "
                f"bias={values['mean_bias_measured_minus_predicted_m']*1000:.2f} mm, "
                f"corr={values['correlation']:.6f}, "
                f"affine scale={values['affine_scale']:.6f}, "
                f"offset={values['affine_offset_m']*1000:.2f} mm"
            )
    if npz_metrics:
        lines.extend([
            '',
            'NPZ identity metrics:',
            f"  RMSE={npz_metrics['raw_rmse_m']*1000:.2f} mm, "
            f"bias={npz_metrics['mean_bias_measured_minus_predicted_m']*1000:.2f} mm, "
            f"corr={npz_metrics['correlation']:.6f}",
        ])
    lines.extend([
        '',
        f"Maximum probe XY error: {result['maximum_xy_constraint_error_m']*1000:.3f} mm",
        f"Maximum final 1 s Z change: {result['maximum_probe_z_change_last_1s_m']*1000:.3f} mm",
        f'Report: {report_path}',
        f'JSON:   {json_path}',
        f'CSV:    {csv_path}',
    ])

    report_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
