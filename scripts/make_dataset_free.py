#!/usr/bin/env python3
"""Generate the segmentation dataset with a free camera: no robot, no Nav2, no MoveIt.

Needs: `ros2 launch amr_manipulator_gazebo dataset.launch.py` (world + dataset_camera + bridge); `generate_dataset.sh`
starts it and restarts it between batches.
The articles of the warehouse are removed and the shelves of one row are filled at random: that is an arrangement.
Each arrangement is photographed many times from poses like those of the arm (the camera looks at the picking face of
the row, at the height of a shelf level, 0.23 to 0.53 m from the slot). All the images of an arrangement go to the
same split, so that the validation and test sets contain arrangements the network has never seen.

An arrangement is saved only when it is complete and the world is the one its metadata describe: its images are kept
in memory, the scene is read at the end, and a label of a class that was not created is an error. If the world cannot be
put right, or is too slow, the arrangement is dropped and the script exits with code 75: the world is to be restarted.
"""
import argparse
import hashlib
import math
import os
import random
import sys
import time

import rclpy
import yaml

import semantic_goals as sg
import stocker
from dataset_common import Capture, Stats, Stock, next_index, pixel_counts, save, split_of

CAMERA = 'dataset_camera'
ARTICLE_CENTRE = 0.085          # height of the centre of an article above the shelf level [m]
MAX_TILT = (math.radians(1.0), math.radians(2.0), math.radians(6.0))     # roll, pitch, yaw error of the arm
RESTART = 75                    # exit code: the world must be restarted


def quaternion(roll, pitch, yaw):
    """(x, y, z, w) of Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def camera_pose(slot, dist, rng):
    """(x, y, z, roll, pitch, yaw) of a camera that looks at the picking face of `slot`, `dist` metres away from it."""
    nx, ny = sg.NORMALS[slot['face']]
    cx = slot['pose'][0] + rng.uniform(-0.03, 0.03)                       # the centre of the view, as the `offset` of the arm
    cz = slot['pose'][2] + ARTICLE_CENTRE + rng.uniform(-0.05, 0.05)
    roll, pitch, yaw_error = (rng.uniform(-m, m) for m in MAX_TILT)
    yaw = math.atan2(-ny, -nx) + yaw_error
    return cx + nx * dist, slot['pose'][1] + ny * dist, cz, roll, pitch, yaw


def move_camera(x, y, z, q, tries=3):
    """Move the camera; the answer of the service is unreliable, so the call is repeated once if it says no."""
    for _ in range(tries):
        if stocker.set_pose(CAMERA, x, y, z, *q):
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='directory of the dataset')
    ap.add_argument('--arrangements', type=int, default=200, help='how many random fillings')
    ap.add_argument('--shots', type=int, default=20, help='images per arrangement')
    ap.add_argument('--dist', type=float, nargs=2, default=[0.23, 0.53], help='range of the camera distance to the slot [m]')
    ap.add_argument('--spread', type=float, default=0.3, help='the camera aims at slots within this distance of the centre of the arrangement along the row [m]')
    ap.add_argument('--half-width', type=float, default=0.7, help='slots within this distance of the centre of the arrangement are filled [m]')
    ap.add_argument('--min-rtf', type=float, default=0.6, help='the world is too slow (restart it) below this real time factor')
    ap.add_argument('--first-index', type=int, help='number of the first image (default: after the last one in --out)')
    ap.add_argument('--seed', type=int, default=1, help='use another seed for every run on the same directory')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    rng = random.Random(args.seed)
    picking = {}                          # row letter -> (name, slot) of the slots of its picking face
    for name, slot in sem['slots'].items():
        if slot['pickable']:
            picking.setdefault(slot['shelf'][0], []).append((name, slot))

    rclpy.init()
    capture = Capture()
    stock = Stock(sem, classes, rng)
    stats = Stats()
    first = args.first_index if args.first_index is not None else next_index(args.out)
    index, failures, dropped, t0, code = first, 0, 0, time.monotonic(), 0
    try:
        if CAMERA not in stocker.models(CAMERA):
            raise stocker.WorldError(f'there is no {CAMERA} in the world')
        stock.clear_original()
        for k in range(args.arrangements):
            rtf = stocker.real_time_factor()
            if rtf is not None and rtf < args.min_rtf:
                raise stocker.WorldError(f'the real time factor is {rtf:.2f}')
            row = rng.choice(sorted(picking))
            group_id = f's{args.seed}-{k}'
            centre = rng.choice(picking[row])[1]['pose'][0]
            records, fill = stock.refill(row, centre, args.half_width)
            created = {r['label'] for r in records}
            targets = [(n, s) for n, s in picking[row] if abs(s['pose'][0] - centre) <= args.spread]
            pending, last = [], None
            for shot in range(args.shots):
                slot_name, slot = rng.choice(targets)
                dist = rng.uniform(*args.dist)
                x, y, z, roll, pitch, yaw = camera_pose(slot, dist, rng)
                q = quaternion(roll, pitch, yaw)
                if not move_camera(x, y, z, q):
                    print(f'{group_id}: the camera could not be moved, shot skipped', flush=True)
                    failures += 1
                    continue
                grabbed = capture.grab(settle=1.0 if shot == 0 else 0.3)       # the first one waits for the articles
                if grabbed is None:
                    failures += 1
                    continue
                image, labels, _ = grabbed
                digest = hashlib.md5(image.tobytes()).digest()
                if digest == last:                                              # the camera did not move
                    failures += 1
                    continue
                last = digest
                pixels = pixel_counts(labels)
                if set(pixels) - {0} - created:
                    raise stocker.WorldError(f'{group_id}: labels of classes that were not created: {sorted(set(pixels) - {0} - created)}')
                pending.append((image, labels, pixels, {
                    'group': group_id, 'row': row, 'target_slot': slot_name, 'dist': round(dist, 3), 'fill': round(fill, 2),
                    'camera_in_map': {'position': [round(x, 4), round(y, 4), round(z, 4)], 'rpy': [roll, pitch, yaw], 'quaternion': list(q)},
                    'articles': records}))
            try:
                stock.verify(records)             # the world was the one of the metadata until the last shot
            except stocker.WorldError:
                dropped += 1
                raise
            split = split_of(group_id)
            for image, labels, pixels, meta in pending:
                meta.update(index=index, pixels=pixels)
                save(args.out, split, index, image, labels, meta)
                stats.add(split, group_id, pixels)
                index += 1
            print(f'{group_id} ({split}): {len(pending)} images, {len(records)} articles; {index - first} in all, '
                  f'{(time.monotonic() - t0) / max(index - first, 1):.2f} s each, {failures} shots lost', flush=True)
    except stocker.WorldError as err:
        print(f'WORLD: {err}; the arrangement in progress is dropped, restart the world', flush=True)
        code = RESTART
    finally:
        try:
            stock.clear_current()
        except stocker.WorldError as err:
            print(f'the articles could not be removed at the end: {err}', flush=True)
        images = stats.write(args.out, args.seed, first, failures, time.monotonic() - t0)
        print(f'done: {images} images in {time.monotonic() - t0:.0f} s, {failures} shots lost, {dropped} arrangements dropped; '
              f'stats in {args.out}/stats_seed{args.seed}.json', flush=True)
        capture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
