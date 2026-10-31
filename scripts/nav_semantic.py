#!/usr/bin/env python3
"""Drive to places of the semantic map by name ("S3", "packing_table", "P1", "receiving")."""
import argparse
import math
import os
import sys

import map_check
import semantic_goals as sg
from nav_goto import run_goals

# A tour of every kind of place: both shelf rows, the two benches, the three zones
TOURS = {
    'shelves': ['S1', 'S2', 'S3', 'S4', 'N3', 'N2', 'N1'],
    'all': ['S1', 'S2', 'S3', 'S4', 'N3', 'N2', 'N1', 'receiving_table', 'packing_table',
            'receiving', 'packing', 'storage'],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*', help='places to visit, in order')
    ap.add_argument('--tour', choices=sorted(TOURS), help='visit a predefined list of places instead')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--sdf', default=os.path.join(sg.GAZEBO, 'worlds', 'warehouse.sdf'))
    ap.add_argument('--gap', type=float, default=0.25, help='free distance platform - shelf face / bench [m]')
    ap.add_argument('--facing', action='store_true', help='bow towards the shelf instead of along the aisle')
    ap.add_argument('--timeout', type=float, default=150.0, help='wall-clock seconds allowed per goal')
    ap.add_argument('--repeat', type=int, default=1, help='visit the whole list this many times')
    args = ap.parse_args()
    if bool(args.names) == bool(args.tour):
        ap.error('give place names or --tour, not both')
    if args.repeat < 1:
        ap.error('--repeat must be at least 1')

    sem = sg.load(args.map)
    boxes = map_check.sdf_boxes(args.sdf, 0.275)
    goals = []
    for name in args.names or TOURS[args.tour]:
        try:
            _, (x, y, yaw) = sg.resolve(sem, boxes, name, args.gap, args.facing)
        except KeyError as err:
            ap.error(err.args[0])
        goals.append((name, x, y, yaw))
        print(f'{name:16s} -> ({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.0f} deg)')
    print()
    sys.exit(0 if run_goals(goals, args.timeout, args.repeat) else 1)


if __name__ == '__main__':
    main()
