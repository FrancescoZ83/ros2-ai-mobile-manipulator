#!/usr/bin/env python3
"""Checks of geometry.py with points made up here (no simulation): the fit must find the pose that made them.

The depth noise is added along the viewing ray, like the stereo error. The measured stereo error in the working range is
8 mm (median of the absolute error), so 8 mm is used as the harder case. The gripper opens 85 mm and the articles are
50-80 mm wide: the centre of an article must be known to about 10 mm.
"""
import math
import sys

import numpy as np

import geometry as g

RNG = np.random.default_rng(0)
ok_all = True


def report(name, passed, text):
    global ok_all
    ok_all &= passed
    print(f'{"OK  " if passed else "FAIL"}  {name}: {text}')


def check_frames():
    # a camera link 1 m high looking along +y of the map (yaw 90 degrees): the optical z axis must be +y
    yaw = math.radians(90)
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    pos, qo = g.optical_pose_from_link((1.0, 2.0, 1.0), q, offset=(0.0, 0.025, 0.0))
    r = g.quat_to_matrix(qo)
    z_axis, x_axis, y_axis = r @ [0, 0, 1], r @ [1, 0, 0], r @ [0, 1, 0]
    good = (np.allclose(z_axis, [0, 1, 0], atol=1e-9) and np.allclose(x_axis, [1, 0, 0], atol=1e-9) and
            np.allclose(y_axis, [0, 0, -1], atol=1e-9))
    # the left sensor is 0.025 m to the left of the link: looking along +y, left is -x
    good &= np.allclose(pos, [1.0 - 0.025, 2.0, 1.0], atol=1e-9)
    q_back = g.matrix_to_quat(g.quat_to_matrix(q))
    good &= np.allclose(q_back, q, atol=1e-9) or np.allclose(q_back, -np.array(q), atol=1e-9)
    report('frames', good, 'optical z forward, x right, y down; the left sensor on the left; quaternion round trip')


def check_backproject():
    intr = g.intrinsics()
    depth = np.full((240, 320), np.nan)
    depth[100:140, 120:200] = 0.4
    mask = np.zeros((240, 320), bool)
    mask[100:140, 120:200] = True
    pts = g.backproject(depth, mask, intr, erode=2)
    fx, fy, cx, cy = intr
    u = pts[:, 0] * fx / pts[:, 2] + cx
    v = pts[:, 1] * fy / pts[:, 2] + cy
    good = (len(pts) == 36 * 76 and np.allclose(pts[:, 2], 0.4) and u.min() >= 122 and u.max() <= 197 and
            v.min() >= 102 and v.max() <= 137 and abs(fx - 233.9) < 0.1)
    report('backproject', good, f'{len(pts)} points of 36 x 76 pixels (mask eroded by 2), fx {fx:.1f}, projection back inside the mask')


def noisy(points, camera, sigma):
    """Noise along the viewing ray."""
    ray = points - camera
    ray /= np.linalg.norm(ray, axis=1, keepdims=True)
    return points + ray * RNG.normal(0.0, sigma, (len(points), 1))


def camera_for(centre, distance, spread):
    angle = RNG.uniform(-spread, spread)
    return centre + distance * np.array([math.sin(angle), -math.cos(angle)])       # in front of the article (towards -y)


def check_circle(sigma, trials=200):
    errors = []
    for _ in range(trials):
        r = RNG.choice([0.030, 0.033, 0.035])
        centre = RNG.uniform(-0.3, 0.3, 2)
        cam = camera_for(centre, RNG.uniform(0.25, 0.5), math.radians(25))
        to_cam = cam - centre
        base = math.atan2(to_cam[1], to_cam[0])
        half = math.acos(min(r / np.linalg.norm(to_cam), 1.0)) * 0.9          # the arc the camera sees
        theta = base + RNG.uniform(-half, half, 150)
        pts = centre + r * np.stack([np.cos(theta), np.sin(theta)], axis=1)
        found, _ = g.fit_circle(noisy(np.c_[pts, np.zeros(len(pts))], np.r_[cam, 0.0], sigma)[:, :2], r, cam)
        errors.append(np.linalg.norm(found - centre))
    p50, p95 = np.percentile(errors, [50, 95]) * 1000
    report(f'circle, noise {sigma * 1000:.0f} mm', p95 < 10.0, f'centre error median {p50:.1f} mm, 95th percentile {p95:.1f} mm (limit 10)')


def check_rectangle(sigma, trials=200):
    pos_err, yaw_err = [], []
    for _ in range(trials):
        size = [(0.070, 0.050), (0.060, 0.060), (0.060, 0.080)][RNG.integers(3)]
        centre = RNG.uniform(-0.3, 0.3, 2)
        yaw = RNG.uniform(-math.radians(15), math.radians(15))
        cam = camera_for(centre, RNG.uniform(0.25, 0.5), math.radians(25))
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]])
        corners = np.array([[-size[0] / 2, -size[1] / 2], [size[0] / 2, -size[1] / 2],
                            [size[0] / 2, size[1] / 2], [-size[0] / 2, size[1] / 2]]) @ rot.T + centre
        normals = np.array([[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]]) @ rot.T
        pts = []
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            if np.dot(normals[i], cam - (a + b) / 2) > 0:
                t = RNG.uniform(0.0, 1.0, 80)[:, None]
                pts.append(a + t * (b - a))
        pts = np.vstack(pts)
        found, _ = g.fit_rectangle(noisy(np.c_[pts, np.zeros(len(pts))], np.r_[cam, 0.0], sigma)[:, :2], size, cam)
        pos_err.append(np.linalg.norm(found[:2] - centre))
        yaw_err.append(abs(found[2] - yaw))
    p50, p95 = np.percentile(pos_err, [50, 95]) * 1000
    y50, y95 = np.degrees(np.percentile(yaw_err, [50, 95]))
    # The yaw has no limit here. The box fits the opening (the widest, 70 x 50 mm turned by 15 degrees, is 80.6 mm across
    # along the row, below 85 mm), but a straight approach to a turned box closes the pads on two corners and the box
    # tilts by about 30 degrees in the retreat (README, the cause of the tilt of the cereal box): the grasp needs it.
    report(f'rectangle, noise {sigma * 1000:.0f} mm', p95 < 10.0,
           f'centre error median {p50:.1f} mm, 95th percentile {p95:.1f} mm (limit 10); '
           f'yaw error median {y50:.1f}, 95th percentile {y95:.1f} degrees (the grasp of a box needs it)')


def check_outliers():
    z = np.r_[RNG.normal(0.40, 0.003, 300), RNG.normal(0.60, 0.002, 30)]          # 10 percent: the shelf behind
    pts = np.c_[RNG.uniform(-0.03, 0.03, 330), RNG.uniform(-0.05, 0.05, 330), z]
    kept = g.reject_outliers(pts)
    report('outliers', len(kept) <= 305 and kept[:, 2].max() < 0.45, f'{len(pts) - len(kept)} of 30 wrong points dropped, {len(kept)} kept')


def render_box(cam_pos, forward, centre, base_z, size, yaw, intr, shape='box'):
    """Exact depth image (along the optical axis) of an upright box or cylinder, by tracing a ray through every pixel.

    Returns (depth with NaN outside, mask, rotation of the optical frame in the map)."""
    fx, fy, cx, cy = intr
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rot = np.stack([right, down, forward], axis=1)                          # columns: optical x, y, z in the map
    v, u = np.indices((240, 320))
    d_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], axis=-1).reshape(-1, 3)
    d = d_cam @ rot.T                                                       # ray directions in the map (z component of d_cam is 1)
    c, s = math.cos(yaw), math.sin(yaw)
    inv = np.array([[c, s], [-s, c]])                                       # map -> box frame (rotation by -yaw), horizontal
    height = size[2] if shape == 'box' else size[1]
    o = np.r_[inv @ (np.asarray(cam_pos)[:2] - centre), cam_pos[2] - (base_z + height / 2.0)]
    dl = np.c_[d[:, :2] @ inv.T, d[:, 2]]
    t = np.full(len(dl), np.inf)
    if shape == 'box':
        half = np.array([size[0] / 2, size[1] / 2, height / 2])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-half - o) / dl, (half - o) / dl
        tn, tf = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
        hit = (tn <= tf) & (tf > 0.1)
        t[hit] = np.maximum(tn[hit], 0.1)
    else:
        a = dl[:, 0] ** 2 + dl[:, 1] ** 2
        b = 2 * (o[0] * dl[:, 0] + o[1] * dl[:, 1])
        cc = o[0] ** 2 + o[1] ** 2 - size[0] ** 2
        disc = b * b - 4 * a * cc
        with np.errstate(invalid='ignore', divide='ignore'):
            t_side = (-b - np.sqrt(disc)) / (2 * a)
        z_hit = o[2] + t_side * dl[:, 2]
        hit = (disc > 0) & (t_side > 0.1) & (np.abs(z_hit) <= height / 2)
        t[hit] = t_side[hit]
    depth = np.where(np.isfinite(t), t, np.nan).reshape(240, 320)
    return depth, np.isfinite(depth), rot


def check_pipeline(shape, trials=60):
    """The code of the real chain on exact images: depth -> points -> map -> fit, camera as in the dataset."""
    intr = g.intrinsics()
    names = {'box': [(0.070, 0.050, 0.160), (0.060, 0.060, 0.170), (0.060, 0.080, 0.090)],
             'cylinder': [(0.033, 0.100), (0.030, 0.180), (0.035, 0.075)]}[shape]
    pos_err, yaw_err, found = [], [], 0
    for _ in range(trials):
        size = names[RNG.integers(len(names))]
        centre = np.array([RNG.uniform(-0.3, 0.3), 0.0])
        yaw = RNG.uniform(-math.radians(15), math.radians(15)) if shape == 'box' else 0.0
        height = size[2] if shape == 'box' else size[1]
        base_z = 0.0
        dist = RNG.uniform(0.25, 0.55)
        # the camera looks at a point of the row within 0.3 m of the article, at the height of a shelf level + 0.085
        aim = np.array([centre[0] + RNG.uniform(-0.1, 0.1), 0.0, 0.085 + RNG.uniform(-0.05, 0.05)])
        cam = np.array([aim[0] + RNG.uniform(-0.03, 0.03), -dist, aim[2]])
        err = [math.radians(RNG.uniform(-1, 1)), math.radians(RNG.uniform(-2, 2)), math.radians(RNG.uniform(-6, 6))]
        # forward = +y turned by the yaw, pitch and roll errors of the arm
        fwd = np.array([-math.sin(err[2]), math.cos(err[2]), 0.0])
        fwd = fwd * math.cos(err[1]) + np.array([0.0, 0.0, math.sin(err[1])])
        fwd /= np.linalg.norm(fwd)
        depth, mask, rot = render_box(cam, fwd, centre, base_z, size, yaw, intr, shape)
        if mask.sum() < 120:
            continue
        q = g.matrix_to_quat(rot)
        est = g.estimate_pose(depth, mask, intr, cam, q, shape, size)
        if est is None:
            continue
        found += 1
        pos_err.append(np.hypot(est[0] - centre[0], est[1] - centre[1]))
        yaw_err.append(abs(math.degrees((est[2] - yaw + math.pi) % (2 * math.pi) - math.pi)))
    p50, p95 = np.percentile(pos_err, [50, 95]) * 1000
    text = f'{found} images; centre error median {p50:.1f} mm, 95th percentile {p95:.1f} mm (limit 10)'
    if shape == 'box':
        y50, y95 = np.percentile(yaw_err, [50, 95])
        text += f'; yaw error median {y50:.1f}, 95th percentile {y95:.1f} degrees (the grasp of a box needs it)'
    report(f'pipeline on exact images of a {shape}', p95 < 10.0, text)


if __name__ == '__main__':
    check_frames()
    check_backproject()
    check_outliers()
    for sigma in (0.003, 0.008):
        check_circle(sigma)
        check_rectangle(sigma)
    check_pipeline('cylinder')
    check_pipeline('box')
    sys.exit(0 if ok_all else 1)
