#!/usr/bin/env python3
"""Turn names from the semantic map (shelf S3, packing_table, door P1, zone packing) into robot poses."""
import argparse
import math
import os

import yaml

import map_check

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO = os.path.join(HERE, '..', 'amr_manipulator_gazebo')

# Platform footprint in base_footprint, measured with robot_geometry.py (same polygon as nav2_params.yaml)
FOOTPRINT = [(-0.4, -0.25), (-0.1, -0.315), (0.1, -0.315), (0.4, -0.25),
             (0.4, 0.25), (0.1, 0.315), (-0.1, 0.315), (-0.4, 0.25)]
HALF_WIDTH = 0.315      # [m] side of the platform from the centre line
FRONT = 0.4             # [m] front of the platform from base_footprint
INSCRIBED = 0.315       # [m] inscribed radius: closer than this to an obstacle is costly for Nav2
CIRCUMSCRIBED = 0.472   # [m] radius of the circle swept by a turn on the spot
NORMALS = {'north': (0.0, 1.0), 'south': (0.0, -1.0), 'east': (1.0, 0.0), 'west': (-1.0, 0.0)}


def load(path):
    """The semantic map as a dict."""
    return yaml.safe_load(open(path))


def pick_face(sem, shelf):
    """Face of a shelf unit that the robot works from: the one whose slots are pickable."""
    faces = {s['face'] for s in sem['slots'].values() if s['shelf'] == shelf and s['pickable']}
    if len(faces) != 1:
        raise ValueError(f'shelf {shelf}: expected one pickable face, found {sorted(faces)}')
    return faces.pop()


def shelf_station(sem, name, gap, facing):
    """Pose (x, y, yaw) in the aisle in front of the pick face of a shelf unit.

    gap: free distance between the platform and the shelf face [m].
    facing: False = bow along the aisle (yaw 0, no turn between neighbouring units, the arm turns);
            True = bow towards the shelf.
    """
    shelf = sem['shelves'][name]
    face = pick_face(sem, name)
    if face not in ('north', 'south'):
        raise ValueError(f'shelf {name}: only faces on the long sides are supported, not {face}')
    nx, ny = NORMALS[face]
    face_y = shelf['pose'][1] + ny * shelf['size'][1] / 2.0
    if facing:
        return shelf['pose'][0], face_y + ny * (FRONT + gap), math.atan2(-ny, -nx)
    return shelf['pose'][0], face_y + ny * (HALF_WIDTH + gap), 0.0


def table_station(sem, name, gap):
    """Pose in front of a bench, on the side that faces the storage zone, bow towards the bench."""
    table = sem['tables'][name]
    side = 1.0 if table['pose'][0] < 0.0 else -1.0          # +x side for the receiving table, -x for packing
    x = table['pose'][0] + side * (table['size'][0] / 2.0 + FRONT + gap)
    return x, table['pose'][1], 0.0 if side < 0 else math.pi


def open_spot(sem, boxes, zone, margin=0.5):
    """The point of a zone farthest from any obstacle at the LiDAR height (grid 0.1 m), yaw 0."""
    (x0, x1), (y0, y1) = sem['zones'][zone]['x'], sem['zones'][zone]['y']
    best = None
    for i in range(int((x1 - x0 - 2 * margin) / 0.1) + 1):
        for j in range(int((y1 - y0 - 2 * margin) / 0.1) + 1):
            x, y = x0 + margin + 0.1 * i, y0 + margin + 0.1 * j
            d = map_check.signed_distance(boxes, x, y)
            if best is None or d > best[0]:
                best = (d, x, y)
    return best[1], best[2], 0.0


def resolve(sem, boxes, name, gap=0.25, facing=False):
    """(kind, (x, y, yaw)) for a name of the semantic map."""
    if name in sem['shelves']:
        return 'shelf', shelf_station(sem, name, gap, facing)
    if name in sem['tables']:
        return 'table', table_station(sem, name, gap)
    if name in sem['bins']:
        return 'bin', table_station(sem, sem['bins'][name]['table'], gap)
    if name in sem['doors']:
        x, y = sem['doors'][name]['pose']
        return 'door', (x, y, 0.0)
    if name in sem['zones']:
        return 'zone', open_spot(sem, boxes, name)
    known = [*sem['shelves'], *sem['tables'], *sem['bins'], *sem['doors'], *sem['zones']]
    raise KeyError(f"unknown name '{name}', known: {', '.join(known)}")


def names(sem):
    """Every name that resolve() understands."""
    return [*sem['shelves'], *sem['tables'], *sem['bins'], *sem['doors'], *sem['zones']]


def clearance(boxes, x, y, yaw):
    """Smallest distance [m] between the platform polygon at a pose and the obstacles (negative = overlap)."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for (x0, y0), (x1, y1) in zip(FOOTPRINT, FOOTPRINT[1:] + FOOTPRINT[:1]):
        n = max(int(math.hypot(x1 - x0, y1 - y0) / 0.04), 1)
        pts += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    return min(map_check.signed_distance(boxes, x + c * px - s * py, y + s * px + c * py) for px, py in pts)


def turn_clearance(boxes, x, y):
    """Smallest platform-obstacle distance over a full turn on the spot (steps of 10 degrees)."""
    return min(clearance(boxes, x, y, math.radians(a)) for a in range(0, 360, 10))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*', help='names to resolve (default: all)')
    ap.add_argument('--map', default=os.path.join(GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--sdf', default=os.path.join(GAZEBO, 'worlds', 'warehouse.sdf'))
    ap.add_argument('--gap', type=float, default=0.25, help='free distance platform - shelf face / bench [m]')
    ap.add_argument('--facing', action='store_true', help='bow towards the shelf instead of along the aisle')
    ap.add_argument('--plane', type=float, default=0.275, help='LiDAR height [m]')
    args = ap.parse_args()

    sem = load(args.map)
    boxes = map_check.sdf_boxes(args.sdf, args.plane)
    print(f"{'name':16s} {'kind':6s} {'x':>7s} {'y':>7s} {'yaw[deg]':>9s} {'at pose[m]':>11s} {'full turn[m]':>13s}")
    for name in args.names or names(sem):
        kind, (x, y, yaw) = resolve(sem, boxes, name, args.gap, args.facing)
        at, turn = clearance(boxes, x, y, yaw), turn_clearance(boxes, x, y)
        note = 'overlap!' if at < 0 else '' if turn >= INSCRIBED else 'cannot turn here' if turn < 0.05 else 'tight turn'
        print(f'{name:16s} {kind:6s} {x:+7.3f} {y:+7.3f} {math.degrees(yaw):9.1f} {at:11.3f} {turn:13.3f}  {note}')
    print(f'\nclearance = distance from the platform polygon to the nearest obstacle at the LiDAR height '
          f'(inscribed radius {INSCRIBED} m, a full turn sweeps {CIRCUMSCRIBED} m)')


if __name__ == '__main__':
    main()
