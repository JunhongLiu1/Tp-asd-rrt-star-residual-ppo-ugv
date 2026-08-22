#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Capture(Node):
    def __init__(self, topic: str):
        super().__init__('radiation_map_stats_capture_v4')
        self.message = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid,
            topic,
            self.callback,
            qos,
        )

    def callback(self, message: OccupancyGrid) -> None:
        self.message = message


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    index = fraction * (len(sorted_values) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return float(sorted_values[lower])
    weight = index - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/radiation_map')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = Capture(args.topic)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and node.message is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    message = node.message
    node.destroy_node()
    rclpy.shutdown()

    if message is None:
        print('[WARN] No radiation map received within timeout', file=sys.stderr)
        return 2

    values = [float(value) for value in message.data if value >= 0]
    values.sort()
    count = len(values)
    if count == 0:
        print('[WARN] Radiation map has no valid cells', file=sys.stderr)
        return 3

    def proportion(predicate):
        return sum(1 for value in values if predicate(value)) / count

    payload = {
        'topic': args.topic,
        'width': int(message.info.width),
        'height': int(message.info.height),
        'resolution_m': float(message.info.resolution),
        'valid_cell_count': count,
        'unknown_cell_count': len(message.data) - count,
        'minimum': min(values),
        'maximum': max(values),
        'mean': sum(values) / count,
        'p50': percentile(values, 0.50),
        'p75': percentile(values, 0.75),
        'p90': percentile(values, 0.90),
        'p95': percentile(values, 0.95),
        'p99': percentile(values, 0.99),
        'fraction_ge_50': proportion(lambda value: value >= 50.0),
        'fraction_ge_90': proportion(lambda value: value >= 90.0),
        'fraction_eq_100': proportion(lambda value: value >= 100.0),
    }
    payload['recommended_distribution_check'] = {
        'high_risk_ge_90_below_15_percent': payload['fraction_ge_90'] < 0.15,
        'fully_saturated_below_10_percent': payload['fraction_eq_100'] < 0.10,
    }

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
