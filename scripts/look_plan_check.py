#!/usr/bin/env python3
"""Can the arm be planned from the travel pose to the look pose of a slot, with the robot off its station? (needs move_group)

For a slot and a class, the robot at the facing station of the slot moved by each (dx, dy, dyaw) (dx along the aisle, dy
away from the shelf, dyaw the heading error): all the collision-free IK solutions of the look pose (the TCP `dist` in front
of the centre of the article, tool horizontal into the shelf) and, for each, the planning of OMPL from the travel pose
(the same request as the MoveGroup goal of the picking code, but plan only): error code and time. With --neighbours the
other slots of the face get a box of the size of the class of the article (a worst case of the planning scene of a pick).
"""
import argparse
import math
import os

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningScene
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

import semantic_goals as sg
from gap_check import Gap
from grasp_geometry import grasp_axes
from moveit_check import Checker, named_states
from reach_check import JOINTS


class Planner(Gap):
    def __init__(self):
        super().__init__()
        self.plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        for c in (self.plan, self.scene):
            if not c.wait_for_service(timeout_sec=30.0):
                raise SystemExit('move_group has no ' + c.srv_name)

    def set_neighbours(self, boxes, remove=()):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        for name, (x, y, z), size in boxes:
            o = CollisionObject()
            o.id = name
            o.header.frame_id = 'map'
            o.operation = CollisionObject.ADD
            o.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
            o.primitive_poses.append(p)
            req.scene.world.collision_objects.append(o)
        for name in remove:
            o = CollisionObject()
            o.id = name
            o.operation = CollisionObject.REMOVE
            req.scene.world.collision_objects.append(o)
        self.call(self.scene, req)

    def plan_to(self, start, goal, base, seconds=5.0, attempts=5):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = 'arm'
        r.num_planning_attempts = attempts
        r.allowed_planning_time = seconds
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 0.5
        self.base_state(r.start_state, start, base)
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=0.002, tolerance_below=0.002, weight=1.0)
            for j, v in zip(JOINTS, goal)])]
        res = self.call(self.plan, req)
        if res is None:
            return None, None
        out = res.motion_plan_response
        return out.error_code.val, out.planning_time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slot', nargs='+', default=['S1.L1.n5'])
    ap.add_argument('--cls', default='water_bottle')
    ap.add_argument('--dist', type=float, default=0.30)
    ap.add_argument('--poses', default='0,0,0', help="'dx,dy,dyaw_deg;...': the robot off its station [m, m, deg]")
    ap.add_argument('--neighbours', action='store_true', help='a box of the class in every other slot of the face (of the first slot)')
    ap.add_argument('--quiet', action='store_true', help='print only the poses that fail to plan, and the counts')
    ap.add_argument('--timeout', type=float, default=0.15)
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()
    sem = sg.load(args.map)
    classes = {a['name']: a for a in yaml.safe_load(open(args.articles))['articles']}
    cls = classes[args.cls]
    height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
    size = cls['size'] if cls['shape'] == 'box' else [2 * cls['size'][0], 2 * cls['size'][0], cls['size'][1]]
    travel = named_states()['travel']
    rclpy.init()
    node = Planner()
    checker = Checker()
    counts = [0, 0]
    for slot_name in args.slot:
        slot = sem['slots'][slot_name]
        if args.neighbours:
            boxes = [(f'nb_{n}', (s['pose'][0], s['pose'][1], s['pose'][2] + size[2] / 2.0), size)
                     for n, s in sem['slots'].items() if s['shelf'] == slot['shelf'] and s['pickable'] and n != slot_name]
            node.set_neighbours(boxes)
        x0, y0, yaw0 = sg.slot_station(sem, slot_name)
        centre = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + height / 2.0)
        approach, rot = grasp_axes('front', sg.NORMALS[slot['face']], 0.0, 'x')
        target = tuple(c - args.dist * a for c, a in zip(centre, approach))
        for spec in args.poses.split(';'):
            dx, dy, dyaw = (float(v) for v in spec.split(','))
            base = (x0 + dx, y0 + dy, yaw0 + math.radians(dyaw))
            cands = node.candidates(target, rot, base, args.timeout)
            if not args.quiet:
                print(f'{slot_name} {args.cls}: robot dx {1000 * dx:+.0f} mm dy {1000 * dy:+.0f} mm dyaw {dyaw:+.1f} deg: {len(cands)} look poses', flush=True)
            for q in cands:
                code, secs = node.plan_to(travel, q, base)
                counts[0] += 1
                counts[1] += code == 1
                if code == 1 and args.quiet:
                    continue
                line = f'    {slot_name} dx {1000 * dx:+.0f} dyaw {dyaw:+.1f}: pan {q[0]:+.2f} lift {q[1]:+.2f} elbow {q[2]:+.2f} w1 {q[3]:+.2f}  -> code {code} in {secs if secs is None else round(secs, 2)} s'
                print(line, flush=True)
    print(f'planned {counts[1]} of {counts[0]} look poses', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
