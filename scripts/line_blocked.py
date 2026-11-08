#!/usr/bin/env python3
"""Why is there no complete straight approach to a slot? (needs move_group, the static scene of the warehouse)

For some slots and classes, with the robot at the facing station moved along the shelf by each offset (the arrival error
along the aisle): all the collision-free IK solutions of the pre-grasp, the fraction of the straight approach that each
completes, and, for the best incomplete one, the first state of the line (same line without collision checking) that is in
collision and with what. The neighbours of the shelf are not in this scene, only the structure of the warehouse.
"""
import argparse
import math
import os

import numpy as np
import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath

import semantic_goals as sg
from gap_check import Gap
from grasp_geometry import grasp_axes
from moveit_check import Checker
from reach_check import APPROACH, LINK

import yaml


class Blocked(Gap):
    def __init__(self):
        super().__init__()
        self.checker = Checker()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slots', nargs='+', default=['S1.L3.n1', 'S1.L3.n3', 'S1.L3.n5'])
    ap.add_argument('--cases', nargs='+', default=['cereal_box:0.0', 'water_bottle:0.04', 'water_bottle:0.0'], help='class:grasp_dz')
    ap.add_argument('--offsets', type=float, nargs='+', default=[-0.08, -0.04, 0.0, 0.04, 0.08], help='robot shift along the aisle [m]')
    ap.add_argument('--gap', type=float, default=sg.SHELF_GAP)
    ap.add_argument('--timeout', type=float, default=0.15)
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()
    sem = sg.load(args.map)
    classes = {a['name']: a for a in yaml.safe_load(open(args.articles))['articles']}
    rclpy.init()
    node = Blocked()
    for name in args.slots:
        slot = sem['slots'][name]
        x0, y0, yaw = sg.shelf_station(sem, slot['shelf'], args.gap, True)
        approach, rot = grasp_axes('front', sg.NORMALS[slot['face']], 0.0, 'x')
        for case in args.cases:
            cname, dz = case.split(':')
            dz = float(dz)
            cls = classes[cname]
            height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
            centre = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + height / 2.0 + dz)
            for dx in args.offsets:
                base = (x0 + dx, y0, yaw)
                pre = tuple(c - APPROACH * a for c, a in zip(centre, approach))
                cands = node.candidates(pre, rot, base, args.timeout)
                fractions = []
                for q in cands:
                    req = GetCartesianPath.Request()
                    req.header.frame_id = 'map'
                    req.group_name = 'arm'
                    req.link_name = LINK
                    req.waypoints = [node.pose(centre, rot)]
                    req.max_step = 0.01
                    req.revolute_jump_threshold = 0.5
                    req.avoid_collisions = True
                    node.base_state(req.start_state, q, base)
                    res = node.call(node.path, req)
                    fractions.append(0.0 if res is None else res.fraction)
                line = f'{name} {cname:12s} dz {dz:.2f} robot {1000 * dx:+4.0f} mm: IK {len(cands)}'
                if cands:
                    line += f', complete {sum(f >= 0.999 for f in fractions)}, fractions ' + ' '.join(f'{f:.2f}' for f in fractions)
                if cands and max(fractions) < 0.999:
                    q = cands[int(np.argmax(fractions))]
                    req = GetCartesianPath.Request()
                    req.header.frame_id = 'map'
                    req.group_name = 'arm'
                    req.link_name = LINK
                    req.waypoints = [node.pose(centre, rot)]
                    req.max_step = 0.01
                    req.revolute_jump_threshold = 0.5
                    req.avoid_collisions = False
                    node.base_state(req.start_state, q, base)
                    res = node.call(node.path, req)
                    pts = res.solution.joint_trajectory.points
                    for k, p in enumerate(pts):
                        valid, contacts = node.checker.check_state(list(p.positions), base, True)
                        if not valid:
                            line += f'\n      first collision at point {k} of {len(pts)}: {contacts[:4]}'
                            break
                    else:
                        line += f'\n      no collision on the {len(pts)} points that the line has without collision checking (it reaches {res.fraction:.2f}): the arm cannot follow it'
                print(line, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
