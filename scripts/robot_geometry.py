#!/usr/bin/env python3
"""List every link with its place in base_footprint and its collision shapes, then the XY footprint."""
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

PLATFORM = ('chassis', 'wheel', 'caster')      # link name parts of the mobile platform


def floats(text):
    """'1 2 3' -> [1.0, 2.0, 3.0]; a missing attribute means zeros."""
    return [float(v) for v in (text or '0 0 0').split()]


def rot_rpy(r, p, y):
    """URDF rotation matrix Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_quat(q):
    """Rotation matrix of a quaternion message."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def describe(collision):
    """One-line description of a <collision> element of the URDF."""
    geom = collision.find('geometry')[0]
    origin = collision.find('origin')
    xyz = floats(origin.get('xyz') if origin is not None else None)
    rpy = floats(origin.get('rpy') if origin is not None else None)
    if geom.tag == 'box':
        what = 'box ' + 'x'.join(f'{v:.3f}' for v in floats(geom.get('size')))
    elif geom.tag == 'cylinder':
        what = f"cylinder r={float(geom.get('radius')):.3f} l={float(geom.get('length')):.3f}"
    elif geom.tag == 'sphere':
        what = f"sphere r={float(geom.get('radius')):.3f}"
    else:
        what = f"{geom.tag} {geom.get('filename').split('/')[-1]}"
    return f"{what}  xyz=({xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f}) rpy=({rpy[0]:+.2f} {rpy[1]:+.2f} {rpy[2]:+.2f})"


def stl_vertices(path):
    """All triangle vertices of a binary or ASCII STL file."""
    data = open(path, 'rb').read()
    n = int.from_bytes(data[80:84], 'little')
    if len(data) == 84 + 50 * n:                                   # binary STL
        tri = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                            count=n, offset=84)
        return tri['v'].reshape(-1, 3).astype(float)
    lines = data.decode(errors='ignore').splitlines()              # ASCII STL
    return np.array([[float(t) for t in ln.split()[1:4]] for ln in lines if ln.strip().startswith('vertex')])


def local_points(geom):
    """Points on the outline of a collision geometry, in the frame of the collision element."""
    angles = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
    if geom.tag == 'box':
        h = np.array(floats(geom.get('size'))) / 2.0
        return np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    if geom.tag == 'cylinder':
        r, h = float(geom.get('radius')), float(geom.get('length')) / 2.0
        return np.array([[r * math.cos(a), r * math.sin(a), z] for a in angles for z in (-h, h)])
    if geom.tag == 'sphere':
        r = float(geom.get('radius'))
        return np.array([p for a in angles for p in ([r * math.cos(a), r * math.sin(a), 0.0],
                                                     [r * math.cos(a), 0.0, r * math.sin(a)],
                                                     [0.0, r * math.cos(a), r * math.sin(a)])])
    package, rest = geom.get('filename').replace('package://', '').split('/', 1)
    path = os.path.join(get_package_share_directory(package), rest)
    if not path.lower().endswith('.stl'):
        raise ValueError(f'only STL meshes are read, not {path}')
    return stl_vertices(path) * np.array(floats(geom.get('scale') or '1 1 1'))


def hull(points):
    """Convex hull of 2D points, counter-clockwise (Andrew's monotone chain)."""
    pts = sorted(set(map(tuple, np.round(points, 4))))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def report(title, points):
    """Print the extent and the convex hull (x, y) of a set of 3D points."""
    if not len(points):
        print(f'  {title}: no points')
        return
    p = np.array(points)
    print(f'  {title}: x {p[:, 0].min():+.3f} .. {p[:, 0].max():+.3f}   y {p[:, 1].min():+.3f} .. '
          f'{p[:, 1].max():+.3f}   z up to {p[:, 2].max():.3f}')
    print('    polygon:', [[round(float(x), 3), round(float(y), 3)] for x, y in hull(p[:, :2])])


def main():
    share = get_package_share_directory('amr_manipulator_description')
    urdf = subprocess.run(['xacro', f'{share}/urdf/amr_manipulator.urdf.xacro', 'use_sim:=true'],
                          capture_output=True, text=True, check=True).stdout
    links = ET.fromstring(urdf).findall('link')

    rclpy.init()
    node = Node('robot_geometry', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    end = time.monotonic() + 3.0      # wall clock: the simulation clock is still 0 before the first message
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"{'link':26s} {'position in base_footprint [m]':32s} collision shapes")
    platform, tools = [], []
    for link in links:
        name = link.get('name')
        try:
            tf = buf.lookup_transform('base_footprint', name, rclpy.time.Time()).transform
            pos = f'{tf.translation.x:+.3f} {tf.translation.y:+.3f} {tf.translation.z:+.3f}'
        except Exception:
            tf, pos = None, '(no transform)'
        collisions = link.findall('collision')
        shapes = [describe(c) for c in collisions]
        print(f'{name:26s} {pos:32s} {shapes[0] if shapes else "-"}')
        for extra in shapes[1:]:
            print(f"{'':26s} {'':32s} {extra}")
        if tf is None:
            continue
        r_link = rot_quat(tf.rotation)
        t_link = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        for c in collisions:
            origin = c.find('origin')
            xyz = np.array(floats(origin.get('xyz') if origin is not None else None))
            r_col = rot_rpy(*floats(origin.get('rpy') if origin is not None else None))
            try:
                pts = local_points(c.find('geometry')[0])
            except Exception as err:
                print(f'  (skipped a collision of {name}: {err})')
                continue
            world = (r_link @ (r_col @ pts.T + xyz[:, None])).T + t_link
            (platform if any(k in name for k in PLATFORM) else tools).extend(world.tolist())

    print('\nFootprint, convex hull of the collision geometry in base_footprint (current arm pose):')
    report('platform (chassis, wheels, casters)', platform)
    report('arm and tools                       ', tools)
    report('whole robot                         ', platform + tools)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
