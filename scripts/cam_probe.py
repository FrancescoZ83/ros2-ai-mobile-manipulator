#!/usr/bin/env python3
"""Read one color, depth and camera_info message of the RGB-D camera and check them (diagnostic)."""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def rotate(q, v):
    """Rotate the vector v by the unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    c = (y * t[2] - z * t[1], z * t[0] - x * t[2], x * t[1] - y * t[0])
    return (v[0] + w * t[0] + c[0], v[1] + w * t[1] + c[1], v[2] + w * t[2] + c[2])


def image_array(msg, channels):
    """Image message -> numpy array (h, w) or (h, w, channels). Only 8-bit and 32FC1."""
    dtype = np.float32 if msg.encoding == '32FC1' else np.uint8
    row_bytes = msg.width * channels * np.dtype(dtype).itemsize
    if msg.step != row_bytes:
        raise ValueError(f'unexpected row padding: step {msg.step}, expected {row_bytes}')
    a = np.frombuffer(msg.data, dtype=dtype)
    return a.reshape((msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels))


def patch(a, u, v, half):
    """Square window of the image around pixel (u = column, v = row), clipped to the image."""
    h, w = a.shape[:2]
    return a[max(0, v - half):min(h, v + half + 1), max(0, u - half):min(w, u + half + 1)]


def back_project(u, v, z, fx, fy, cx, cy):
    """Pinhole model: pixel (u, v) with depth z -> (x, y, z) in the optical frame."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


class Probe(Node):
    def __init__(self, base):
        super().__init__('cam_probe')
        self.msgs = {}
        self.create_subscription(
            Image, base + '/image', lambda m: self.msgs.setdefault('color', m), qos_profile_sensor_data)
        self.create_subscription(
            Image, base + '/depth_image', lambda m: self.msgs.setdefault('depth', m), qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, base + '/camera_info', lambda m: self.msgs.setdefault('info', m), qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='/camera', help='camera topic prefix')
    ap.add_argument('--u', type=int, default=-1, help='pixel column (default: image centre)')
    ap.add_argument('--v', type=int, default=-1, help='pixel row (default: image centre)')
    ap.add_argument('--half', type=int, default=5, help='half size of the window around the pixel')
    ap.add_argument('--world', default='base_footprint', help='frame used for the camera pose')
    ap.add_argument('--timeout', type=float, default=10.0, help='wall-clock seconds to wait')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.base)
    start = time.time()
    while rclpy.ok() and time.time() - start < args.timeout and len(node.msgs) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [k for k in ('color', 'depth', 'info') if k not in node.msgs]
    if missing:
        print(f'missing messages: {missing}')
        node.destroy_node()
        rclpy.shutdown()
        return

    color, depth, info = node.msgs['color'], node.msgs['depth'], node.msgs['info']
    cimg = image_array(color, 3)
    dimg = image_array(depth, 1)
    u = args.u if args.u >= 0 else depth.width // 2
    v = args.v if args.v >= 0 else depth.height // 2
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]

    # Depth statistics: no-return pixels are inf or nan, so count only finite positive values
    valid = np.isfinite(dimg) & (dimg > 0.0)
    win = patch(dimg, u, v, args.half)
    win = win[np.isfinite(win) & (win > 0.0)]
    z = float(np.median(win)) if win.size else float('nan')
    rgb = patch(cimg, u, v, args.half).reshape(-1, 3).mean(axis=0)

    print(f'color  {color.width}x{color.height} {color.encoding}, frame {color.header.frame_id!r}, '
          f'mean RGB around ({u},{v}): {rgb[0]:.0f} {rgb[1]:.0f} {rgb[2]:.0f}')
    print(f'depth  {depth.width}x{depth.height} {depth.encoding}, frame {depth.header.frame_id!r}, '
          f'valid {100.0 * valid.mean():.1f} %')
    if valid.any():
        print(f'       min {dimg[valid].min():.3f} m, max {dimg[valid].max():.3f} m, '
              f'median around ({u},{v}): {z:.3f} m')
    print(f'info   {info.width}x{info.height}, frame {info.header.frame_id!r}, '
          f'fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f}')
    if math.isfinite(z):
        x, y, zz = back_project(u, v, z, fx, fy, cx, cy)
        print(f'pixel ({u},{v}) back-projected in the optical frame: x {x:+.3f} y {y:+.3f} z {zz:.3f} m')

    # Where the camera is and where it looks, to predict the depth of a flat floor
    try:
        tf = node.tf_buffer.lookup_transform(args.world, color.header.frame_id, Time())
        p, o = tf.transform.translation, tf.transform.rotation
        axis = rotate((o.x, o.y, o.z, o.w), (0.0, 0.0, 1.0))   # optical z = viewing direction
        print(f'camera in {args.world}: x {p.x:.4f} y {p.y:.4f} z {p.z:.4f}, '
              f'viewing direction ({axis[0]:+.3f} {axis[1]:+.3f} {axis[2]:+.3f})')
        if axis[2] < -0.1:
            # Plane z = 0 seen along the optical axis (exact for the centre pixel)
            print(f'expected depth of the floor at the image centre: {p.z / -axis[2]:.3f} m')
    except Exception as exc:   # tf2 raises several exception types
        print(f'no transform {args.world} -> {color.header.frame_id!r}: {exc}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
