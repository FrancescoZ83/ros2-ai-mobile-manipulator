#!/usr/bin/env python3
"""Print a summary of one LaserScan: geometry, header and the closest return per sector."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Sector centres in degrees, in the scan frame (0 = x axis, positive = counter-clockwise)
SECTORS = [('front', 0.0), ('left', 90.0), ('back', 180.0), ('right', -90.0)]


def angle_diff_deg(a, b):
    """Smallest signed difference a - b in degrees, in [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def sector_min(scan, center_deg, half_deg):
    """Closest valid return within +-half_deg of center_deg, as (range, angle_deg) or None."""
    best = None
    for i, r in enumerate(scan.ranges):
        # Gazebo reports rays without a hit as inf: skip them and anything outside the limits
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        if abs(angle_diff_deg(a, center_deg)) <= half_deg:
            if best is None or r < best[0]:
                best = (r, a)
    return best


class Probe(Node):
    def __init__(self, topic):
        super().__init__('scan_probe')
        self.scan = None
        self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scan = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--half', type=float, default=10.0, help='half width of each sector [deg]')
    ap.add_argument('--timeout', type=float, default=10.0, help='wall-clock seconds to wait')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.topic)
    start = time.time()
    while rclpy.ok() and node.scan is None and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = node.scan
    if s is None:
        print(f'no message on {args.topic}')
        node.destroy_node()
        rclpy.shutdown()
        return

    valid = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
    print(f'frame_id      {s.header.frame_id!r}')
    print(f'rays          {len(s.ranges)}')
    print(f'angle         {math.degrees(s.angle_min):+.2f} .. {math.degrees(s.angle_max):+.2f} deg, '
          f'step {math.degrees(s.angle_increment):.4f} deg')
    print(f'range limits  {s.range_min:.2f} .. {s.range_max:.2f} m')
    print(f'valid returns {len(valid)} of {len(s.ranges)}')
    print(f'closest return per sector (+-{args.half:.0f} deg):')
    for name, centre in SECTORS:
        best = sector_min(s, centre, args.half)
        text = 'none' if best is None else f'{best[0]:.3f} m at {best[1]:+.1f} deg'
        print(f'  {name:<6} {text}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
