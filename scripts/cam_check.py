#!/usr/bin/env python3
"""Check camera model + TF end to end: find the orange box in the image and compare with its known pose."""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.time import Time

# Helpers shared with cam_probe.py (same folder)
from cam_probe import Probe, back_project, image_array, rotate


def transform_point(tf, p):
    """Apply a geometry_msgs/TransformStamped to the point p = (x, y, z)."""
    t, o = tf.transform.translation, tf.transform.rotation
    r = rotate((o.x, o.y, o.z, o.w), p)
    return (r[0] + t.x, r[1] + t.y, r[2] + t.z)


def project(p, fx, fy, cx, cy):
    """Point in the optical frame (z > 0) -> pixel (u = column, v = row)."""
    return (fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy)


def locate_box(cimg, dimg):
    """Pixel centroid and mean depth of the top face of the orange box, or None.

    The top face is the nearest orange surface, so only orange pixels within 1 cm
    of the smallest orange depth are kept (the visible side faces are farther away).
    """
    c = cimg.astype(np.int16)
    orange = (c[..., 0] - c[..., 2] > 80) & (c[..., 0] > c[..., 1])
    ok = orange & np.isfinite(dimg) & (dimg > 0.0)
    if ok.sum() < 20:
        return None
    top = ok & (dimg < float(dimg[ok].min()) + 0.01)
    vs, us = np.nonzero(top)
    return float(us.mean()), float(vs.mean()), float(dimg[top].mean()), int(top.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--box', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'),
                    help='known position of the centre of the box top face in the world frame')
    ap.add_argument('--world', default='base_footprint', help='frame of --box (robot at the origin)')
    ap.add_argument('--base', default='/camera', help='camera topic prefix')
    ap.add_argument('--timeout', type=float, default=10.0, help='wall-clock seconds to wait')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.base)
    start = time.time()
    frame = None
    while rclpy.ok() and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.msgs) == 3:
            frame = node.msgs['color'].header.frame_id
            if node.tf_buffer.can_transform(args.world, frame, Time()):
                break
    if frame is None or not node.tf_buffer.can_transform(args.world, frame, Time()):
        print('missing images or transform')
        node.destroy_node()
        rclpy.shutdown()
        return

    color, depth, info = node.msgs['color'], node.msgs['depth'], node.msgs['info']
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    found = locate_box(image_array(color, 3), image_array(depth, 1))
    if found is None:
        print('orange box not found in the image')
        node.destroy_node()
        rclpy.shutdown()
        return
    u, v, z, n = found

    # Measured: pixel + depth -> point in the optical frame -> world frame
    tf_world_cam = node.tf_buffer.lookup_transform(args.world, frame, Time())
    measured = transform_point(tf_world_cam, back_project(u, v, z, fx, fy, cx, cy))
    # Expected: known world point -> optical frame -> pixel
    tf_cam_world = node.tf_buffer.lookup_transform(frame, args.world, Time())
    p_cam = transform_point(tf_cam_world, tuple(args.box))
    ue, ve = project(p_cam, fx, fy, cx, cy)

    print(f'top face pixels: {n}')
    print(f'pixel    measured ({u:6.1f}, {v:6.1f})   expected ({ue:6.1f}, {ve:6.1f})   '
          f'difference ({u - ue:+.1f}, {v - ve:+.1f}) px')
    print(f'depth    measured {z:.3f} m             expected {p_cam[2]:.3f} m')
    print('world    axis   measured  expected   error[mm]')
    for name, m, e in zip('xyz', measured, args.box):
        print(f'         {name}      {m:8.4f}  {e:8.4f}   {1000 * (m - e):+7.1f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
