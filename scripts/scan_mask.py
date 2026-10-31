#!/usr/bin/env python3
"""Find the LiDAR rays that hit the robot itself: sectors with a near return, over many scans."""
import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Collector(Node):
    def __init__(self, topic):
        super().__init__('scan_mask')
        self.scans = []
        self.create_subscription(LaserScan, topic, lambda m: self.scans.append(m), qos_profile_sensor_data)


def ray_state(r, scan):
    """Median-friendly value of one ray: a finite range, or None when there is no usable return."""
    if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
        return r
    return None


def median_ranges(scans):
    """Per-ray median range over all scans (None where most scans have no return)."""
    out = []
    for i in range(len(scans[0].ranges)):
        values = [ray_state(s.ranges[i], s) for s in scans]
        good = [v for v in values if v is not None]
        out.append(statistics.median(good) if len(good) > len(values) / 2 else None)
    return out


def runs(flags):
    """Contiguous runs of True in a circular list, as (first_index, length)."""
    n = len(flags)
    if all(flags):
        return [(0, n)]
    start = next(i for i in range(n) if not flags[i])          # begin right after a False: no run wraps
    result, i = [], 0
    while i < n:
        j = (start + i) % n
        if flags[j]:
            length = 0
            while i + length < n and flags[(start + i + length) % n]:
                length += 1
            result.append((j, length))
            i += length
        else:
            i += 1
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--scans', type=int, default=20, help='number of scans to take the median of')
    ap.add_argument('--near', type=float, default=0.5, help='returns closer than this [m] are the robot')
    ap.add_argument('--timeout', type=float, default=30.0, help='wall-clock seconds to wait')
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.topic)
    start = time.time()
    while rclpy.ok() and len(node.scans) < args.scans and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    scans = node.scans
    node.destroy_node()
    rclpy.shutdown()
    if not scans:
        print(f'no message on {args.topic}')
        return

    s = scans[0]
    med = median_ranges(scans)
    angle = lambda i: math.degrees(s.angle_min + i * s.angle_increment)
    print(f'{len(scans)} scans, {len(med)} rays, step {math.degrees(s.angle_increment):.4f} deg, '
          f'range limits {s.range_min:.2f} .. {s.range_max:.2f} m')

    near = [m is not None and m < args.near for m in med]
    print(f'\nrays with a median return closer than {args.near} m: {sum(near)}')
    for first, length in runs(near):
        ids = [(first + k) % len(med) for k in range(length)]
        rng = [med[k] for k in ids]
        print(f'  {angle(ids[0]):+8.1f} .. {angle(ids[-1]):+8.1f} deg  ({length:3d} rays)  '
              f'range {min(rng):.3f} .. {max(rng):.3f} m')

    # Rays with no return at all: in a closed warehouse they are sensor artefacts, not free space
    blind = [m is None for m in med]
    print(f'\nrays with no valid return: {sum(blind)}')
    for first, length in runs(blind):
        last = (first + length - 1) % len(med)
        print(f'  {angle(first):+8.1f} .. {angle(last):+8.1f} deg  ({length:3d} rays)')


if __name__ == '__main__':
    main()
