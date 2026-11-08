#!/usr/bin/env python3
"""The tilt of an article in time, from a trace of the poses of the world (pick_oracle.py --trace-tilt).

The file is the text of `ign topic -e -t /world/amr_world/pose/info`: one message every 20 ms of simulated time, each with the
pose of every entity. For the article (a model called ds_*) and for the links of the gripper, the position and the quaternion
at every message. The tilt is the angle between the z axis of the article and the vertical; `along` and `across` are its parts
in the vertical plane through the approach (rotation about the axis between the pads) and across it (about the approach).
"""
import math
import re
import sys

import numpy as np


def parse(path, names):
    """{name: (t [s], position (n, 3), quaternion (n, 4) as x y z w)} for the entities called `names` (a regex)."""
    pattern = re.compile(names)
    series = {}
    text = open(path).read()
    for msg in text.split('\nheader {')[1:]:
        t = re.search(r'sec: (\d+)\s+nsec: (\d+)', msg)
        t = float(t.group(1)) + float(t.group(2)) * 1e-9 if t else math.nan
        for block in re.findall(r'^pose \{\n(.*?)^\}', msg, re.M | re.S):
            name = re.search(r'name: "([^"]+)"', block)
            if not name or not pattern.fullmatch(name.group(1)):
                continue
            pos = re.search(r'position \{(.*?)\}', block, re.S)
            ori = re.search(r'orientation \{(.*?)\}', block, re.S)

            def field(part, key, default=0.0):
                m = re.search(rf'\b{key}: (\S+)', part.group(1) if part else '')
                return float(m.group(1)) if m else default
            series.setdefault(name.group(1), []).append(
                (t, [field(pos, k) for k in 'xyz'], [field(ori, 'x'), field(ori, 'y'), field(ori, 'z'), field(ori, 'w', 1.0)]))
    return {n: (np.array([r[0] for r in rows]), np.array([r[1] for r in rows]), np.array([r[2] for r in rows]))
            for n, rows in series.items()}


def z_axis(q):
    x, y, z, w = q.T
    return np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], axis=1)


def tilt(q, approach):
    """(tilt [deg], along [deg], across [deg]) of the z axis of the article: along is the lean towards `approach` (x, y)."""
    z = z_axis(q)
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    side = np.array([-a[1], a[0]])
    total = np.degrees(np.arccos(np.clip(z[:, 2], -1, 1)))
    along = np.degrees(np.arctan2(z[:, :2] @ a, z[:, 2]))
    across = np.degrees(np.arctan2(z[:, :2] @ side, z[:, 2]))
    return total, along, across


if __name__ == '__main__':
    path = sys.argv[1]
    every = parse(path, r'.*')
    for n in sorted(every):
        if n.startswith('ds_') or 'gripper' in n or 'finger' in n or 'pad' in n:
            print(n, len(every[n][0]))
