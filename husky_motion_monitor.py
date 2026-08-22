#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class HuskyMotionMonitor(Node):

    def __init__(self):
        super().__init__('husky_motion_monitor')

        dem_path = (
            Path.home()
            / 'terrain_radiation_ws/src/radiation_mapping/dem/processed'
            / 'dem_terrain_hard_husky_015_513.png'
        )

        self.dem = np.asarray(
            Image.open(dem_path).convert('L'),
            dtype=np.float64
        )

        self.size_x = 19.047619
        self.size_y = 30.0
        self.size_z = 1.514038

        self.wheelbase = 0.5120
        self.track = 0.5708
        self.wheel_z = 0.03282
        self.wheel_radius = 0.1651

        self.wheels = {
            'FL': ( self.wheelbase / 2,  self.track / 2, self.wheel_z),
            'FR': ( self.wheelbase / 2, -self.track / 2, self.wheel_z),
            'RL': (-self.wheelbase / 2,  self.track / 2, self.wheel_z),
            'RR': (-self.wheelbase / 2, -self.track / 2, self.wheel_z),
        }

        out_dir = Path.home() / 'terrain_radiation_ws/experiment_results'
        out_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = out_dir / 'husky_motion_monitor.csv'
        self.csv_file = self.csv_path.open('w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time_sec',
            'x', 'y', 'z',
            'speed_m_s',
            'roll_deg',
            'pitch_deg',
            'max_penetration_cm',
            'max_clearance_cm'
        ])

        self.last_print_time = -1.0

        self.sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.callback,
            20
        )

        self.get_logger().info(
            f'Monitor started. CSV: {self.csv_path}'
        )

    def dem_height(self, x, y):
        u = (
            (x + self.size_x / 2.0)
            / self.size_x
            * (self.dem.shape[1] - 1)
        )

        v = (
            (y + self.size_y / 2.0)
            / self.size_y
            * (self.dem.shape[0] - 1)
        )

        if (
            u < 0 or u >= self.dem.shape[1] - 1
            or v < 0 or v >= self.dem.shape[0] - 1
        ):
            return None

        # PNG Y direction is opposite to Gazebo world Y.
        row_f = (self.dem.shape[0] - 1) - v
        col_f = u

        r0 = int(math.floor(row_f))
        c0 = int(math.floor(col_f))
        r1 = min(r0 + 1, self.dem.shape[0] - 1)
        c1 = min(c0 + 1, self.dem.shape[1] - 1)

        dr = row_f - r0
        dc = col_f - c0

        gray = (
            self.dem[r0, c0] * (1-dr) * (1-dc)
            + self.dem[r0, c1] * (1-dr) * dc
            + self.dem[r1, c0] * dr * (1-dc)
            + self.dem[r1, c1] * dr * dc
        )

        return gray / 255.0 * self.size_z

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        x, y, z = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Quaternion -> rotation matrix.
        R = np.array([
            [
                1 - 2*(qy*qy + qz*qz),
                2*(qx*qy - qz*qw),
                2*(qx*qz + qy*qw)
            ],
            [
                2*(qx*qy + qz*qw),
                1 - 2*(qx*qx + qz*qz),
                2*(qy*qz - qx*qw)
            ],
            [
                2*(qx*qz - qy*qw),
                2*(qy*qz + qx*qw),
                1 - 2*(qx*qx + qy*qy)
            ]
        ])

        sinr = 2.0 * (qw*qx + qy*qz)
        cosr = 1.0 - 2.0 * (qx*qx + qy*qy)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (qw*qy - qz*qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        contacts = []

        for local in self.wheels.values():
            world = (
                np.array([x, y, z])
                + R @ np.array(local)
            )

            wx, wy, wz = world
            terrain_z = self.dem_height(wx, wy)

            if terrain_z is None:
                continue

            wheel_bottom = wz - self.wheel_radius

            # Positive = wheel enters terrain.
            penetration = terrain_z - wheel_bottom
            contacts.append(penetration)

        if contacts:
            max_pen = max(0.0, max(contacts)) * 100.0
            max_clear = max(0.0, -min(contacts)) * 100.0
        else:
            max_pen = 0.0
            max_clear = 0.0

        v = msg.twist.twist.linear
        speed = math.sqrt(
            v.x*v.x + v.y*v.y + v.z*v.z
        )

        t = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        self.writer.writerow([
            t, x, y, z,
            speed,
            math.degrees(roll),
            math.degrees(pitch),
            max_pen,
            max_clear
        ])
        self.csv_file.flush()

        if self.last_print_time < 0 or t - self.last_print_time >= 1.0:
            self.last_print_time = t

            self.get_logger().info(
                f'pos=({x:.2f},{y:.2f},{z:.2f}) '
                f'v={speed:.2f} m/s '
                f'roll={math.degrees(roll):.1f}° '
                f'pitch={math.degrees(pitch):.1f}° '
                f'penetration={max_pen:.2f} cm '
                f'clearance={max_clear:.2f} cm'
            )


def main():
    rclpy.init()
    node = HuskyMotionMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
