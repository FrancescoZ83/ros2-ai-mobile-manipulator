#!/usr/bin/env python3
"""Tables from the JSON of pose_eval.py: the error of the estimated pose, by class, by distance, by source of error.

The error of the centre is split along the viewing direction (the normal of the shelf face: the depth) and across it
(along the row). The grasp closes along the row, so that is the direction where the error costs most.
"""
import json
import sys

import numpy as np

VARIANTS = (('A', 'depth of the simulator (noise 10 mm), true mask'), ('B', 'stereo depth, true mask'), ('C', 'stereo depth, network mask (everything)'))
CLASSES = ('cereal_box', 'juice_carton', 'soup_can', 'water_bottle', 'parcel', 'jam_jar')


def errors(rows, variant):
    """Arrays (across, depth, yaw) of the errors in mm and degrees, and the number of rows without an estimate."""
    across, depth, yaw, missing = [], [], [], 0
    for r in rows:
        est = r.get(variant)
        if est is None:
            missing += 1
            continue
        e = np.array(est[:2]) - np.array(r['truth'][:2])
        n = np.array(r['normal'])
        d = float(e @ n)
        across.append(float(np.linalg.norm(e - d * n)) * 1000)
        depth.append(d * 1000)
        if r['shape'] == 'box':
            dy = est[2] - r['truth'][2]
            yaw.append(abs(np.degrees((dy + np.pi) % (2 * np.pi) - np.pi)))
    return np.array(across), np.array(depth), np.array(yaw), missing


def line(label, rows, variant):
    across, depth, yaw, missing = errors(rows, variant)
    if len(across) == 0:
        return f'{label:14s} {len(rows):5d}  no estimate'
    total = np.hypot(across, depth)
    text = (f'{label:14s} {len(rows):5d} {100 * len(across) / len(rows):5.0f} %   centre error median {np.median(total):5.1f} mm, 95th {np.percentile(total, 95):5.1f} mm'
            f'   across {np.median(across):4.1f}/{np.percentile(across, 95):5.1f}   depth bias {np.median(depth):+5.1f}, |depth| 95th {np.percentile(np.abs(depth), 95):5.1f}')
    if len(yaw):
        text += f'   yaw median {np.median(yaw):4.1f} deg, 95th {np.percentile(yaw, 95):4.1f}'
    return text


def main():
    rows = json.load(open(sys.argv[1]))
    print(f'{len(rows)} articles tried, {len({(r["arrangement"], r["shot"]) for r in rows})} views\n')
    for variant, what in VARIANTS:
        if not any(r.get(variant) for r in rows):                 # not measured in this file
            continue
        print(f'{variant}: {what}')
        print(f'{"":14s} {"n":>5s} {"found":>6s}')
        for cls in CLASSES:
            sub = [r for r in rows if r['class'] == cls]
            if sub:
                print(line(cls, sub, variant))
        print(line('all', rows, variant))
        edges = np.quantile([r['dist'] for r in rows], [0, 1 / 3, 2 / 3, 1])
        for lo, hi in zip(edges[:-1], edges[1:]):
            sub = [r for r in rows if lo <= r['dist'] <= hi]
            print(line(f'{lo:.2f}-{hi:.2f} m', sub, variant))
        print()


if __name__ == '__main__':
    main()
