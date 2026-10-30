#!/usr/bin/env python3
"""Print where the gripper TCP is and how its axes point, from TF (diagnostic)."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rotate(q, v):
    """Rotate the vector v by the unit quaternion q = (x, y, z, w)."""
    u, w = q[:3], q[3]
    t = cross(u, v)
    t = (2 * t[0], 2 * t[1], 2 * t[2])   # t = 2 (u x v)
    c = cross(u, t)                      # u x t
    return (v[0] + w * t[0] + c[0], v[1] + w * t[1] + c[1], v[2] + w * t[2] + c[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='base_link', help='reference frame (must be level)')
    ap.add_argument('--tip', default='gripper_tcp', help='frame to describe')
    args = ap.parse_args()

    rclpy.init()
    node = Node('tcp_pose')
    buf = Buffer()
    TransformListener(buf, node)

    tf = None
    start = time.time()
    while rclpy.ok() and time.time() - start < 5.0 and tf is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform(args.frame, args.tip, Time()):
            tf = buf.lookup_transform(args.frame, args.tip, Time())
    if tf is None:
        print(f'no transform {args.frame} -> {args.tip}')
    else:
        p = tf.transform.translation
        r = tf.transform.rotation
        q = (r.x, r.y, r.z, r.w)
        print(f'{args.tip} in {args.frame}: x={p.x:+.4f} y={p.y:+.4f} z={p.z:+.4f}')
        for name, axis in (('x', (1, 0, 0)), ('y', (0, 1, 0)), ('z', (0, 0, 1))):
            a = rotate(q, axis)
            print(f'  tool {name} axis = ({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})   '
                  f'vertical component {a[2]:+.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
