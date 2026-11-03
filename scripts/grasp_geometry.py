#!/usr/bin/env python3
"""Geometry of the grasp of an article on a shelf: tool axes, quaternions, article sizes."""
import math

import yaml

OPENING = 0.085          # [m] opening of the gripper


def matrix_to_quat(r):
    """Quaternion (x, y, z, w) of a rotation matrix given as three rows."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return ((r[2][1] - r[1][2]) * s, (r[0][2] - r[2][0]) * s, (r[1][0] - r[0][1]) * s, 0.25 / s)
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k])
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (r[j][i] + r[i][j]) / s
    q[k] = (r[k][i] + r[i][k]) / s
    q[3] = (r[k][j] - r[j][k]) / s
    return tuple(q)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def frame(x, y, z):
    """Rotation matrix (rows) whose columns are the axes x, y, z of the tool."""
    return [[x[i], y[i], z[i]] for i in range(3)]


def rotate_z(v, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def grasp_axes(kind, normal, yaw, closing):
    """(approach direction, rotation matrix) of the tool for a grasp of an article on a shelf.

    The tool z axis is the approach direction; the fingers close along the tool y axis.
    top:   approach downwards, the fingers close along x (aisle) or y (depth), turned with the article.
    front: approach horizontally into the shelf, the fingers close along the aisle, tool x up (camera on top).
    """
    if kind == 'top':
        z = (0.0, 0.0, -1.0)
        y = rotate_z((1.0, 0.0, 0.0) if closing == 'x' else (0.0, 1.0, 0.0), yaw)
    else:
        z = (-normal[0], -normal[1], 0.0)
        y = cross(z, (0.0, 0.0, 1.0))
    x = cross(y, z)
    return z, frame(x, y, z)


def article_table(path):
    """name -> {'height', 'wide': turned by the yaw jitter of the stock, the article is wider than the opening}."""
    spec = yaml.safe_load(open(path))
    yaw = math.radians(spec['limits']['yaw_jitter_deg'])
    table = {}
    for a in spec['articles']:
        if a['shape'] == 'box':
            width = a['size'][0] * math.cos(yaw) + a['size'][1] * math.sin(yaw)
            height = a['size'][2]
        else:
            width = 2 * a['size'][0]
            height = a['size'][1]
        table[a['name']] = {'height': height, 'wide': width > OPENING}
    return table
