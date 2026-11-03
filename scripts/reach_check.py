#!/usr/bin/env python3
"""Reach analysis: can the arm grasp the pickable shelf slots from the pick stations? (needs move_group)"""
import argparse
import math
import os
import random

import rclpy
from geometry_msgs.msg import Pose, Transform
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.node import Node

import semantic_goals as sg
from grasp_geometry import article_table, grasp_axes, matrix_to_quat

JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
LINK = 'gripper_tcp'
APPROACH = 0.10          # [m] the pre-grasp pose is this far from the grasp pose, against the approach direction
# Joint values with the tool pointing down (home90) and, by rows, pan towards a shelf on the right / left
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708]


class Analyst(Node):
    def __init__(self):
        super().__init__('reach_check')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.path = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        for client in (self.ik, self.path):
            if not client.wait_for_service(timeout_sec=30.0):
                raise SystemExit('move_group is not running (no /compute_ik or /compute_cartesian_path)')
        self.rng = random.Random(7)

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        return future.result()

    @staticmethod
    def base_state(state, q, base):
        x, y, yaw = base
        state.joint_state.name = JOINTS
        state.joint_state.position = list(q)
        state.multi_dof_joint_state.header.frame_id = 'map'
        state.multi_dof_joint_state.joint_names = ['world_joint']
        t = Transform()
        t.translation.x, t.translation.y = x, y
        t.rotation.z, t.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        state.multi_dof_joint_state.transforms = [t]

    @staticmethod
    def pose(position, rot):
        p = Pose()
        p.position.x, p.position.y, p.position.z = position
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = matrix_to_quat(rot)
        return p

    def solve(self, position, rot, base, seeds, timeout):
        """First collision-free IK solution of the TCP pose (frame map) for one of the seeds, or None."""
        for seed in seeds:
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.ik_link_name = LINK
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = 'map'
            req.ik_request.pose_stamped.pose = self.pose(position, rot)
            self.base_state(req.ik_request.robot_state, seed, base)
            req.ik_request.timeout.sec, req.ik_request.timeout.nanosec = int(timeout), int((timeout % 1) * 1e9)
            res = self.call(self.ik, req)
            if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    def approach_ok(self, q_pre, position, rot, base):
        """Does a straight line from the pre-grasp joints to the grasp pose go all the way, free of collisions?"""
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
        return res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS and res.fraction > 0.999

    def look(self, slot, article, station, seeds, timeout, dist):
        """True if the TCP can be put `dist` in front of the article, tool horizontal into the shelf, free of collisions.

        The camera of the gripper is 0.08 m behind the TCP and looks along the tool z axis."""
        center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + article['height'] / 2.0)
        approach, rot = grasp_axes('front', sg.NORMALS[slot['face']], 0.0, 'x')
        return self.solve(tuple(c - dist * a for c, a in zip(center, approach)), rot, station, seeds, timeout) is not None

    def grasp(self, kind, slot, article, station, seeds, timeout):
        """True if the article of the slot can be approached and reached with this kind of grasp from the station."""
        height = article['height']
        center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + height / 2.0)
        normal = sg.NORMALS[slot['face']]
        approach, rot = grasp_axes(kind, normal, slot['yaw'], 'y' if article['wide'] else 'x')
        pre = tuple(c - APPROACH * a for c, a in zip(center, approach))
        q_pre = self.solve(pre, rot, station, seeds, timeout)
        return q_pre is not None and self.approach_ok(q_pre, center, rot, station)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gaps', type=float, nargs='+', default=[0.15, 0.25, 0.35], help='platform-shelf gaps to compare [m]')
    ap.add_argument('--orientations', nargs='+', choices=['along', 'facing'], default=['along', 'facing'])
    ap.add_argument('--grasps', nargs='+', choices=['top', 'front', 'look'], default=['top', 'front'],
                    help='look: only a collision-free IK solution of the observation pose (see --look-dist)')
    ap.add_argument('--look-dist', type=float, default=0.30, help='TCP distance to the article centre for look [m]')
    ap.add_argument('--shelves', nargs='+', help='only these shelf units (default: all)')
    ap.add_argument('--generic', action='store_true', help='every pickable slot with a generic 0.17 m tall box, not the real stock')
    ap.add_argument('--offsets', default='0,0,0',
                    help="errors of the arrival to try, 'dx,dy,dyaw;dx,dy,dyaw;...': dx along the aisle [m], "
                         "dy away from the shelf [m], dyaw [rad] (write it as --offsets='...')")
    ap.add_argument('--timeout', type=float, default=0.15, help='IK timeout per seed [s]')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    sem = sg.load(args.map)
    articles = article_table(args.articles)
    shelves = args.shelves or list(sem['shelves'])
    slots = [(n, s) for n, s in sem['slots'].items() if s['pickable'] and s['shelf'] in shelves
             and (args.generic or s['article'])]
    print(f'{len(slots)} {"slots (generic article)" if args.generic else "real articles"} on the pickable faces of {len(shelves)} shelf units')

    rclpy.init()
    node = Analyst()
    offsets = [tuple(float(v) for v in o.split(',')) for o in args.offsets.split(';')]
    for orientation, gap, kind, offset in [(o, g, k, f) for o in args.orientations for g in args.gaps
                                           for k in args.grasps for f in offsets]:
        by_level, failed = {1: [0, 0], 2: [0, 0], 3: [0, 0]}, []
        for name, slot in slots:
            article = {'height': 0.17, 'wide': False} if args.generic or not slot['article'] else articles[slot['article']]
            x, y, yaw = sg.shelf_station(sem, slot['shelf'], gap, orientation == 'facing')
            normal = sg.NORMALS[slot['face']]
            x, y, yaw = x + offset[0] + normal[0] * offset[1], y + normal[1] * offset[1], yaw + offset[2]
            # Seeds: tool down with the first joint towards the shelf, and a second posture
            pan = {'along': -1.5708 if slot['face'] == 'north' else 1.5708, 'facing': 0.0}[orientation]
            seeds = [[pan] + HOME[1:], [pan, -1.2, 1.6, -2.0, -1.5708, 1.5708]]
            if kind == 'look':
                ok = node.look(slot, article, (x, y, yaw), seeds, args.timeout, args.look_dist)
            else:
                ok = node.grasp(kind, slot, article, (x, y, yaw), seeds, args.timeout)
            by_level[slot['level']][1] += 1
            by_level[slot['level']][0] += ok
            if not ok:
                failed.append(name + ('' if args.generic else f' ({slot["article"]})'))
        total = [sum(v[i] for v in by_level.values()) for i in (0, 1)]
        levels = '  '.join(f'level {k}: {v[0]:2d}/{v[1]:2d}' for k, v in by_level.items())
        print(f'{orientation:6s} gap {gap:.2f}  {kind:5s} {"at " + format(args.look_dist, ".2f") + " m" if kind == "look" else "grasp"}, error dx {offset[0]:+.2f} dy {offset[1]:+.2f} '
              f'dyaw {math.degrees(offset[2]):+4.0f}: {total[0]:3d}/{total[1]:3d}   {levels}', flush=True)
        if failed and len(failed) <= 12:
            print('         not reached:', ', '.join(failed), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
