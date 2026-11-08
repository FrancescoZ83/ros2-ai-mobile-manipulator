#!/usr/bin/env python3
"""How natural are the postures of the pre-grasp, as a function of the distance of the robot from the shelf? (needs move_group)

For a generic 0.17 m tall box in every pickable slot of some shelf units, with the robot at the facing station for each gap:
all the collision-free IK solutions of the pre-grasp (10 cm before the grasp, tool horizontal into the shelf) from a grid of seeds,
those whose straight approach is complete, and the one the picking code takes: the one with the shortest line (`--best-pregrasp`),
and the one the swing-aware rule takes (among the lines within 1 rad of the best, the nearest to the joints of the look pose). For each:
the base joint, the swing from the look pose (sum of |dq|), and whether the shoulder is turned the other way round (|pan| > 1.6 rad).
The look pose is chosen as look_solution does (the IK solution with the smallest largest joint), and the swing of the look pose is
measured from the travel pose of the SRDF.
"""
import argparse
import math
import os

import numpy as np
import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK

import semantic_goals as sg
from grasp_geometry import grasp_axes
from moveit_check import named_states, normalised
from reach_check import APPROACH, JOINTS, LINK, Analyst

SEEDS = [[pan, -1.4, 1.5 * e, -1.5708, -1.5708, w3] for pan in (-2.7, -1.2, 0.0, 1.2, 2.7) for e in (1, -1) for w3 in (1.5708, -1.5708)]
SHORT_LINE = 1.0
FLIPPED = 1.6          # |pan| above this: the shoulder is turned the other way round


class Gap(Analyst):
    def candidates(self, position, rot, base, timeout):
        """All the different collision-free IK solutions of a TCP pose (frame map) from the grid of seeds."""
        found = []
        for seed in SEEDS:
            q = self.solve(position, rot, base, [seed], timeout)
            if q is None:
                continue
            q = normalised(q)
            if not any(max(abs(a - b) for a, b in zip(q, other)) < 0.05 for other in found):
                found.append(q)
        return found

    def line(self, q_pre, position, rot, base):
        """Joint travel [rad] of the straight line from q_pre to `position`, or None if it is not complete and free."""
        req = GetCartesianPath.Request()
        req.header.frame_id = 'map'
        req.group_name = 'arm'
        req.link_name = LINK
        req.waypoints = [self.pose(position, rot)]
        req.max_step = 0.01
        req.revolute_jump_threshold = 0.5
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.5
        self.base_state(req.start_state, q_pre, base)
        res = self.call(self.path, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < 0.999:
            return None
        pts = res.solution.joint_trajectory.points
        return float(sum(np.abs(np.array(b.positions) - np.array(a.positions)).sum() for a, b in zip(pts, pts[1:])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gaps', type=float, nargs='+', default=[0.20, 0.25, 0.30, 0.35, 0.40])
    ap.add_argument('--shelves', nargs='+', default=['S1', 'S3'])
    ap.add_argument('--timeout', type=float, default=0.15)
    ap.add_argument('--align', action='store_true', help='the station of each slot is in front of the slot (x of the slot), not of the centre of the unit')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    args = ap.parse_args()
    sem = sg.load(args.map)
    travel = named_states()['travel']
    slots = [(n, s) for n, s in sem['slots'].items() if s['pickable'] and s['shelf'] in args.shelves]
    print(f'{len(slots)} slots of {args.shelves}, travel pose {np.round(travel, 2).tolist()}', flush=True)
    rclpy.init()
    node = Gap()
    for gap in args.gaps:
        rows = []
        for name, slot in slots:
            x, y, yaw = sg.shelf_station(sem, slot['shelf'], gap, True)
            if args.align:
                x = slot['pose'][0]
            base = (x, y, yaw)
            center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + 0.085)
            approach, rot = grasp_axes('front', sg.NORMALS[slot['face']], 0.0, 'x')
            look_q = node.candidates(tuple(c - 0.30 * a for c, a in zip(center, approach)), rot, base, args.timeout)
            if look_q:
                look = min(look_q, key=lambda q: max(abs(v) for v in q))
                look_swing = float(np.abs(np.array(look) - np.array(travel)).sum())
                natural = [q for q in look_q if abs(q[0]) <= FLIPPED] or look_q
                look_nat = min(natural, key=lambda q: max(abs(v) for v in q))
                look_near = min(look_q, key=lambda q: float(np.abs(np.array(q) - np.array(travel)).sum()))
                look_near_swing = float(np.abs(np.array(look_near) - np.array(travel)).sum())
            else:
                look, look_swing, look_near, look_near_swing, look_nat = None, None, None, None, None
            pre_q = node.candidates(tuple(c - APPROACH * a for c, a in zip(center, approach)), rot, base, args.timeout)
            lines = [(node.line(q, center, rot, base), q) for q in pre_q]
            lines = [(t, q) for t, q in lines if t is not None]
            row = {'slot': name, 'level': slot['level'], 'ik': len(pre_q), 'complete': len(lines), 'look': look is not None,
                   'look_swing': look_swing, 'look_near_swing': look_near_swing}
            if lines and look is not None:
                best_t = min(t for t, _ in lines)
                by_line = min(lines, key=lambda l: l[0])[1]
                near = min((l for l in lines if l[0] <= best_t + SHORT_LINE),
                           key=lambda l: float(np.abs(np.array(l[1]) - np.array(look)).sum()))[1]
                near2 = min((l for l in lines if l[0] <= best_t + SHORT_LINE),
                            key=lambda l: float(np.abs(np.array(l[1]) - np.array(look_near)).sum()))[1]
                near3 = min((l for l in lines if l[0] <= best_t + SHORT_LINE),
                            key=lambda l: float(np.abs(np.array(l[1]) - np.array(look_nat)).sum()))[1]
                unflipped = [l[0] for l in lines if abs(l[1][0]) <= FLIPPED]
                row['unflipped_extra'] = (min(unflipped) - best_t) if unflipped else None
                row['best_line'] = best_t
                row['look_flipped'] = abs(look[0]) > FLIPPED
                row['nat_look_flipped'] = abs(look_nat[0]) > FLIPPED
                for tag, q, ref in (('line', by_line, look), ('swing', near, look), ('both', near2, look_near), ('nat', near3, look_nat)):
                    row[tag] = {'pan': q[0], 'swing': float(np.abs(np.array(q) - np.array(ref)).sum()), 'flipped': abs(q[0]) > FLIPPED}
                row['natural_available'] = any(abs(q[0]) <= FLIPPED for _, q in lines)
            rows.append(row)
        print(f'--- gap {gap:.2f} m, robot at the facing station', flush=True)
        for level in (1, 2, 3):
            r = [w for w in rows if w['level'] == level]
            ok = [w for w in r if 'line' in w]
            ls = [w['look_swing'] for w in r if w['look_swing'] is not None]
            ln = [w['look_near_swing'] for w in r if w['look_near_swing'] is not None]
            def med(tag, key):
                v = [w[tag][key] for w in ok]
                return float(np.median(v)) if v else float('nan')
            print(f'  level {level}: reached {len(ok):2d}/{len(r):2d}  look pose {sum(w["look"] for w in r):2d}/{len(r)} (swing from travel: comfortable {np.median(ls) if ls else float("nan"):.1f} rad, nearest {np.median(ln) if ln else float("nan"):.1f} rad)  '
                  f'complete candidates median {np.median([w["complete"] for w in r]):.0f}  natural available {sum(w.get("natural_available", False) for w in ok):2d}/{len(ok):2d}')
            print(f'           shortest line: flipped {sum(w["line"]["flipped"] for w in ok):2d}/{len(ok):2d}, swing median {med("line", "swing"):5.1f} rad   '
                  f'swing-aware: flipped {sum(w["swing"]["flipped"] for w in ok):2d}/{len(ok):2d}, swing median {med("swing", "swing"):5.1f} rad   '
                  f'with the nearest look pose too: flipped {sum(w["both"]["flipped"] for w in ok):2d}/{len(ok):2d}, swing median {med("both", "swing"):5.1f} rad', flush=True)
            ex = [w['unflipped_extra'] for w in ok if w.get('unflipped_extra') is not None]
            print(f'           line travel [rad]: best median {np.median([w["best_line"] for w in ok]) if ok else float("nan"):.1f}; the best line without the shoulder turned over is longer by a median of '
                  f'{np.median(ex) if ex else float("nan"):.1f} rad (maximum {max(ex) if ex else float("nan"):.1f}); longer by more than 1 rad in {sum(e > 1.0 for e in ex)}/{len(ex)}')
            print(f'           look pose flipped (comfortable rule): {sum(w["look_flipped"] for w in ok):2d}/{len(ok):2d}   '
                  f'look pose and pre-grasp without the shoulder turned over: look flipped {sum(w["nat_look_flipped"] for w in ok):2d}/{len(ok):2d}, '
                  f'pre-grasp flipped {sum(w["nat"]["flipped"] for w in ok):2d}/{len(ok):2d}, swing median {med("nat", "swing"):5.1f} rad', flush=True)
        missed = [w['slot'] for w in rows if 'line' not in w]
        if missed:
            print('  not reached:', ', '.join(missed), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
