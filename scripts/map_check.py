#!/usr/bin/env python3
"""Compare a saved SLAM map with the boxes of the SDF world that cross the LiDAR plane."""
import argparse
import math
import os
import xml.etree.ElementTree as ET
from collections import deque

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO = os.path.join(HERE, '..', 'amr_manipulator_gazebo')


def read_pgm(path):
    """Binary PGM (P5) -> (width, height, rows of pixel values, row 0 = top)."""
    data = open(path, 'rb').read()
    tokens, pos = [], 0
    while len(tokens) < 4:                       # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos)
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end].decode())
        pos = end
    pos += 1                                     # single whitespace after maxval
    w, h = int(tokens[1]), int(tokens[2])
    return w, h, [list(data[pos + r * w:pos + (r + 1) * w]) for r in range(h)]


def load_map(yaml_path):
    """Map yaml + pgm -> dict with the occupied-cell grid (True = occupied)."""
    meta = yaml.safe_load(open(yaml_path))
    w, h, pix = read_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))
    thr = meta['occupied_thresh']
    # negate 0: darker pixel = more occupied; occupancy probability p = (255 - pixel) / 255
    occ = [[(255 - v) / 255.0 > thr for v in row] for row in pix]
    return {'w': w, 'h': h, 'res': meta['resolution'], 'ox': meta['origin'][0], 'oy': meta['origin'][1], 'occ': occ}


def pose(text):
    """'x y z r p y' -> (x, y, z, yaw); roll and pitch must be zero."""
    x, y, z, r, p, yaw = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    if abs(r) > 1e-6 or abs(p) > 1e-6:
        raise ValueError('rotated about x or y: not supported')
    return x, y, z, yaw


def compose(parent, child):
    """Pose of `child` (given in `parent`) in the parent's own frame, planar rotation only."""
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw


def sdf_boxes(sdf_path, plane):
    """Boxes of the world whose z-range contains `plane`: (model, name, cx, cy, yaw, sx, sy)."""
    boxes = []
    for model in ET.parse(sdf_path).getroot().find('world').findall('model'):
        m_pose = pose(model.findtext('pose'))
        for link in model.findall('link'):
            l_pose = compose(m_pose, pose(link.findtext('pose')))
            for col in link.findall('collision'):
                box = col.find('geometry/box')
                if box is None:                               # ground plane
                    continue
                cx, cy, cz, yaw = compose(l_pose, pose(col.findtext('pose')))
                sx, sy, sz = [float(v) for v in box.findtext('size').split()]
                if cz - sz / 2 <= plane <= cz + sz / 2:
                    boxes.append((model.get('name'), f"{link.get('name')}/{col.get('name')}", cx, cy, yaw, sx, sy))
    return boxes


def rasterize(boxes, m):
    """Grid of the same shape as the map: owner model name where a cell centre is inside a box, else None."""
    grid = [[None] * m['w'] for _ in range(m['h'])]
    for model, _, cx, cy, yaw, sx, sy in boxes:
        reach = 0.5 * math.hypot(sx, sy)
        c0, c1 = int((cx - reach - m['ox']) / m['res']), int((cx + reach - m['ox']) / m['res']) + 1
        r0, r1 = int(m['h'] - (cy + reach - m['oy']) / m['res']) - 1, int(m['h'] - (cy - reach - m['oy']) / m['res']) + 1
        cs, sn = math.cos(yaw), math.sin(yaw)
        for r in range(max(r0, 0), min(r1, m['h'] - 1) + 1):
            for c in range(max(c0, 0), min(c1, m['w'] - 1) + 1):
                dx = m['ox'] + (c + 0.5) * m['res'] - cx
                dy = m['oy'] + (m['h'] - r - 0.5) * m['res'] - cy
                if abs(cs * dx + sn * dy) <= sx / 2 and abs(-sn * dx + cs * dy) <= sy / 2:
                    grid[r][c] = model
    return grid


def reachable(solid, start):
    """Cells not in `solid` that can be reached from `start` moving in 4 directions."""
    h, w = len(solid), len(solid[0])
    seen = {start}
    todo = deque([start])
    while todo:
        r, c = todo.popleft()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and (rr, cc) not in seen and solid[rr][cc] is None:
                seen.add((rr, cc))
                todo.append((rr, cc))
    return seen


def signed_distance(boxes, x, y):
    """Distance [m] from (x, y) to the closest box, negative if inside it (positive = free side)."""
    best = float('inf')
    for _, _, cx, cy, yaw, sx, sy in boxes:
        dx, dy = x - cx, y - cy
        u = math.cos(yaw) * dx + math.sin(yaw) * dy
        v = -math.sin(yaw) * dx + math.cos(yaw) * dy
        gx, gy = abs(u) - sx / 2, abs(v) - sy / 2
        d = math.hypot(max(gx, 0.0), max(gy, 0.0)) if max(gx, gy) > 0 else max(gx, gy)
        best = min(best, d)
    return best


def percentile(sorted_values, p):
    """p-th percentile (0..100) of an already sorted list, nearest rank."""
    return sorted_values[min(int(p / 100.0 * len(sorted_values)), len(sorted_values) - 1)]


def within(cells, r, c, k):
    """True if any cell of the boolean grid `cells` lies within k cells (square window) of (r, c)."""
    h, w = len(cells), len(cells[0])
    return any(cells[rr][cc] for rr in range(max(r - k, 0), min(r + k, h - 1) + 1)
               for cc in range(max(c - k, 0), min(c + k, w - 1) + 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(GAZEBO, 'maps', 'warehouse.yaml'))
    ap.add_argument('--sdf', default=os.path.join(GAZEBO, 'worlds', 'warehouse.sdf'))
    ap.add_argument('--plane', type=float, default=0.275, help='height of the LiDAR plane [m]')
    ap.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0], help='free point of the world [m]')
    ap.add_argument('--tol', type=int, default=1, help='tolerance of the SDF -> map check [cells]')
    ap.add_argument('--tol-m', type=float, default=0.10, help='tolerance of the map -> SDF check [m]')
    ap.add_argument('--min-fraction', type=float, default=0.95)
    args = ap.parse_args()

    m = load_map(args.map)
    boxes = sdf_boxes(args.sdf, args.plane)
    solid = rasterize(boxes, m)
    sdf_occ = [[v is not None for v in row] for row in solid]
    n_map = sum(sum(row) for row in m['occ'])
    print(f"map {m['w']}x{m['h']} cells, {m['res']} m, origin ({m['ox']}, {m['oy']}), {n_map} occupied cells")
    print(f"sdf {len(boxes)} boxes cross z = {args.plane} m, {sum(sum(row) for row in sdf_occ)} cells")

    # 1. Map -> SDF: distance of every occupied cell centre from the nearest box.
    #    Negative = inside a box, positive = on the free side. A double wall would show up as a tail.
    dist = []
    for r in range(m['h']):
        for c in range(m['w']):
            if m['occ'][r][c]:
                x, y = m['ox'] + (c + 0.5) * m['res'], m['oy'] + (m['h'] - r - 0.5) * m['res']
                dist.append((signed_distance(boxes, x, y), x, y))
    values = sorted(d for d, _, _ in dist)
    print('\noccupied map cells, signed distance from the nearest SDF box [m]:')
    print('  percentiles  ' + '  '.join(f'p{p}: {percentile(values, p):+.3f}' for p in (5, 25, 50, 75, 95, 100)))
    for limit in (0.05, args.tol_m):
        print(f'  within {limit:.2f} m of a box: {100.0 * sum(abs(v) <= limit for v in values) / max(n_map, 1):5.1f} %')
    good = sum(abs(v) <= args.tol_m for v in values) / max(n_map, 1)
    far = sorted(dist, reverse=True)[:5]
    print('  farthest cells (d, x, y):', ', '.join(f'({d:+.3f}, {x:+.2f}, {y:+.2f})' for d, x, y in far))

    # 2. SDF -> map: the exposed faces of the world (touching free space that can be reached
    #    from the start) that have an occupied map cell within the tolerance.
    start = (int(m['h'] - (args.start[1] - m['oy']) / m['res']), int((args.start[0] - m['ox']) / m['res']))
    free = reachable(solid, start)
    per_model = {}
    for r in range(m['h']):
        for c in range(m['w']):
            if solid[r][c] is None:
                continue
            if any((rr, cc) in free for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1))):
                total, hit = per_model.get(solid[r][c], (0, 0))
                per_model[solid[r][c]] = (total + 1, hit + within(m['occ'], r, c, args.tol))
    total = sum(t for t, _ in per_model.values())
    covered = sum(h for _, h in per_model.values()) / max(total, 1)
    print(f'\nexposed SDF edge cells covered by an occupied map cell (+-{args.tol} cell), per model:')
    for name, (t, h) in sorted(per_model.items(), key=lambda kv: kv[1][1] / kv[1][0]):
        print(f'  {name:16s} {h:4d} / {t:4d}  {100.0 * h / t:5.1f} %')

    print(f'\nmap cells near the SDF : {100 * good:5.1f} %   (>= {100 * args.min_fraction:.0f} % needed)')
    print(f'SDF edges in the map  : {100 * covered:5.1f} %   (>= {100 * args.min_fraction:.0f} % needed)')
    print('RESULT:', 'PASS' if good >= args.min_fraction and covered >= args.min_fraction else 'FAIL')


if __name__ == '__main__':
    main()
