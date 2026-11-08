"""Pose of an article from the 3D points of its visible surface and its known shape.

The articles stand upright on a shelf with their axes along the map axes, turned by a small yaw (up to 15 degrees).
So the pose is three numbers in the horizontal plane (x, y, yaw), seen from above: a cylinder is a circle of known radius
(two numbers, a cylinder has no yaw), a box is a rectangle of known sides. The camera sees only the faces turned
towards it, so the points lie on those, and only those faces are compared with the model: otherwise the same points fit a
rectangle in front of them or behind them, and the depth would be undetermined.

No ROS here: numpy and scipy, so everything can be tested with points made up by the test.
"""
import math

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

MAX_YAW = math.radians(30.0)      # the stock turns the articles by 15 degrees at most; the fit may go further


def intrinsics(width=320, height=240, fov=1.2):
    """(fx, fy, cx, cy) of a pinhole camera with a horizontal field of view `fov` [rad]."""
    fx = (width / 2.0) / math.tan(fov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def backproject(depth, mask, intr, erode=2, z_range=(0.1, 2.0)):
    """Points (N x 3, optical frame: x right, y down, z forward) of the pixels of `mask` that have a depth.

    The mask is eroded by `erode` pixels: at the edge of an article the matching mixes the article with what is
    behind it. depth: H x W float, NaN where there is none."""
    fx, fy, cx, cy = intr
    inside = ndimage.binary_erosion(mask, iterations=erode) if erode > 0 else mask.astype(bool)
    v, u = np.nonzero(inside & np.isfinite(depth) & (depth > z_range[0]) & (depth < z_range[1]))
    z = depth[v, u].astype(np.float64)
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)


def reject_outliers(points, k=3.0, floor=0.004):
    """Drop the points whose depth is far from the median of the others (a wrong match, or the shelf behind)."""
    if len(points) < 10:
        return points
    z = points[:, 2]
    med = np.median(z)
    spread = max(1.4826 * np.median(np.abs(z - med)), floor)
    return points[np.abs(z - med) <= k * spread]


def quat_to_matrix(q):
    """Rotation matrix of the quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_map(points, position, quaternion):
    """Points from a frame whose pose in the map is (position, quaternion) to the map."""
    return points @ quat_to_matrix(quaternion).T + np.asarray(position, dtype=np.float64)


def optical_pose_from_link(position, quaternion, offset=(0.0, 0.025, 0.0)):
    """Pose in the map of the optical frame of the left camera, from the pose of the camera link.

    The link has x forward, y left, z up; the left sensor is `offset` from it; the optical frame has z forward,
    x right, y down. Returns (position, quaternion x y z w)."""
    r_link = quat_to_matrix(quaternion)
    optical = np.array([[0.0, 0.0, 1.0],       # columns: the optical x, y, z axes written in the link frame
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0]])
    r = r_link @ optical
    return np.asarray(position) + r_link @ np.asarray(offset), matrix_to_quat(r)


def matrix_to_quat(r):
    """Quaternion (x, y, z, w) of a rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def fit_circle(xy, radius, camera_xy):
    """Centre (x, y) of a circle of known radius that passes through the points of its near side.

    The start is behind the centroid of the points, as seen from the camera, by the radius: the points of the near
    arc fit a circle on either side, and this picks the right one. Returns (centre, rms of the residual [m])."""
    xy = np.asarray(xy, dtype=np.float64)
    away = xy.mean(axis=0) - np.asarray(camera_xy)
    away /= np.linalg.norm(away)
    start = xy.mean(axis=0) + radius * away

    def residual(c):
        return np.linalg.norm(xy - c, axis=1) - radius

    sol = least_squares(residual, start, loss='soft_l1', f_scale=0.004)
    return sol.x, float(np.sqrt(np.mean(residual(sol.x) ** 2)))


def _distance_to_visible_edges(xy, centre, yaw, size, camera_xy):
    """Distance of each point to the nearest edge of the rectangle that faces the camera."""
    sx, sy = size
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    corners = np.array([[-sx / 2, -sy / 2], [sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2]]) @ rot.T + centre
    normals = np.array([[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]]) @ rot.T       # outward, edge i: corner i -> i+1
    best = np.full(len(xy), np.inf)
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        if np.dot(normals[i], np.asarray(camera_xy) - (a + b) / 2) <= 0:                 # the edge faces away: not seen
            continue
        ab = b - a
        t = np.clip(((xy - a) @ ab) / (ab @ ab), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(xy - (a + t[:, None] * ab), axis=1))
    return best


def fit_rectangle(xy, size, camera_xy):
    """(x, y, yaw) of a rectangle of known sides (along the map x and y at yaw 0) whose faces turned to the camera
    pass through the points. Several starting yaws; the best is kept. Returns (x, y, yaw), rms residual [m]."""
    xy = np.asarray(xy, dtype=np.float64)
    away = xy.mean(axis=0) - np.asarray(camera_xy)
    away /= np.linalg.norm(away)
    start = xy.mean(axis=0) + 0.5 * min(size) * away
    best = None
    for yaw0 in (-0.4, -0.15, 0.0, 0.15, 0.4):
        def residual(p):
            return _distance_to_visible_edges(xy, p[:2], p[2], size, camera_xy)

        sol = least_squares(residual, [start[0], start[1], yaw0], loss='soft_l1', f_scale=0.004,
                            bounds=([-np.inf, -np.inf, -MAX_YAW], [np.inf, np.inf, MAX_YAW]))
        if best is None or sol.cost < best.cost:
            best = sol
    rms = float(np.sqrt(np.mean(_distance_to_visible_edges(xy, best.x[:2], best.x[2], size, camera_xy) ** 2)))
    return best.x, rms


def estimate_pose(depth, mask, intr, cam_pos, cam_quat, shape, size, min_points=30):
    """(x, y, yaw, number of points) of an article in the map from a depth image and the mask of its pixels, or None.

    depth: H x W [m] along the optical axis; (cam_pos, cam_quat): pose of the optical frame in the map; shape 'cylinder'
    (size: radius, height) or 'box' (size: x, y, z, the sides along the map axes at yaw 0)."""
    pts = reject_outliers(backproject(depth, mask, intr))
    if len(pts) < min_points:
        return None
    xy = to_map(pts, cam_pos, cam_quat)[:, :2]
    cam_xy = np.asarray(cam_pos)[:2]
    if shape == 'cylinder':
        c, _ = fit_circle(xy, size[0], cam_xy)
        return float(c[0]), float(c[1]), 0.0, len(pts)
    p, _ = fit_rectangle(xy, (size[0], size[1]), cam_xy)
    return float(p[0]), float(p[1]), float(p[2]), len(pts)
