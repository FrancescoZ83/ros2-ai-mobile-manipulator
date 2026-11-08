#!/usr/bin/env python3
"""The pose of the articles from the camera of the robot, measured against the true pose of the simulation.

As make_dataset.py: Nav2 takes the robot to the station of a shelf, MoveIt puts the arm in a look pose, the shelves are
filled at random. Then, as in pose_eval.py, the pose of every visible article of the picking face is estimated (stereo
depth, with the true mask and with the mask of the network) and compared with the true one.

The pose of the camera is the one the grasp will have: the arm's transforms from the robot's frame to the camera
(`base_footprint` to `stereo_left_optical_frame`, from the joints), and the pose of the robot in the world from the
simulator, not from Nav2: the localization of Nav2 is off by centimetres, and it is the pose of the article relative to the
robot that matters for the grasp, not its pose in the map.

Needs the simulation (world:=warehouse.sdf stereo:=true dataset:=true), Nav2, move_group and the stereo node;
`pose_eval_robot.sh` starts them.
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
import rclpy
import torch
import yaml
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drive_test                                                 # noqa: E402  (its parse_pose reads the true pose of a model)
import semantic_goals as sg                                       # noqa: E402
from make_dataset import navigate                                 # noqa: E402
from moveit_check import Checker, joint_constraints, look_solution, named_states   # noqa: E402
from nav2_simple_commander.robot_navigator import BasicNavigator  # noqa: E402
from pose_eval import PoseCapture, ROOT, geo, predict, view_rows  # noqa: E402
from dataset_common import Stock                                  # noqa: E402
from model import UNet                                            # noqa: E402


def robot_in_world():
    """(x, y, yaw) of the robot model in the world, from the simulator."""
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/amr_world/pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=15).stdout
    return drive_test.parse_pose(out, 'amr_manipulator')


def camera_in_base(capture, stamp, timeout=10.0):
    """Position and quaternion of the left optical frame in base_footprint at the time of a frame."""
    when = rclpy.time.Time(seconds=stamp[0], nanoseconds=stamp[1], clock_type=rclpy.clock.ClockType.ROS_TIME)
    end = time.monotonic() + timeout
    while time.monotonic() < end and not capture.tf_buffer.can_transform('base_footprint', 'stereo_left_optical_frame', when):
        rclpy.spin_once(capture, timeout_sec=0.1)
    t = capture.tf_buffer.lookup_transform('base_footprint', 'stereo_left_optical_frame', when).transform
    return (np.array([t.translation.x, t.translation.y, t.translation.z]),
            np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))


def camera_in_world(position, quaternion, robot):
    """Pose of the camera in the world, from its pose in base_footprint and the pose (x, y, yaw) of the robot."""
    x, y, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    r_robot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return r_robot @ position + np.array([x, y, 0.0]), geo.matrix_to_quat(r_robot @ geo.quat_to_matrix(quaternion))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='JSON file with every estimate')
    ap.add_argument('--name', default='base', help='network: segmentation/runs/<name>.pt')
    ap.add_argument('--units', nargs='+', default=['S1', 'N2', 'S3', 'N1'])
    ap.add_argument('--targets', type=int, default=4, help='look poses per unit')
    ap.add_argument('--restocks', type=int, default=4, help='random fillings per look pose')
    ap.add_argument('--dist', type=float, nargs=2, default=[0.15, 0.40], help='range of the TCP distance to the slot [m]')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    by_name = {c['name']: c for c in classes}
    rng = random.Random(args.seed)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    ck = torch.load(os.path.join(ROOT, 'segmentation', 'runs', args.name + '.pt'), map_location=device, weights_only=True)
    model = UNet(3, 7, 16).to(device)
    model.load_state_dict(ck['model'])
    model.eval()
    intr = geo.intrinsics()

    rclpy.init()
    arm, capture, nav = Checker(), PoseCapture(), BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    stock = Stock(sem, classes, rng)
    stock.clear_original()
    stow = joint_constraints(named_states()['travel'])
    results, views, lost, t0 = [], 0, 0, time.monotonic()
    try:
        for unit in args.units:
            _, (x, y, yaw) = sg.resolve(sem, None, unit)
            arm.execute(stow, 1.0)
            if not navigate(nav, x, y, yaw):
                print(f'{unit}: the robot did not reach the station, skipped', flush=True)
                continue
            face_slots = {n: s for n, s in sem['slots'].items() if s['shelf'] == unit and s['pickable']}
            for t in range(args.targets):
                reached = False
                for attempt in range(4):
                    slot_name = rng.choice(sorted(face_slots))
                    slot = face_slots[slot_name]
                    dist = rng.uniform(*args.dist)
                    offset = (rng.uniform(-0.03, 0.03), 0.0, rng.uniform(-0.03, 0.03))
                    center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + 0.085)
                    sol = look_solution(arm, center, sg.NORMALS[slot['face']], dist, offset)
                    if sol is None:
                        continue
                    if arm.execute(joint_constraints(sol), 1.0)[0].result.error_code.val == 1:
                        reached = True
                        break
                if not reached:
                    lost += 1
                    print(f'{unit}-{t}: no look pose reached', flush=True)
                    continue
                for k in range(args.restocks):
                    records, _ = stock.refill(unit, center[0], 0.4)
                    grabbed = capture.grab()
                    if grabbed is None:
                        lost += 1
                        continue
                    image, labels, stamp = grabbed
                    stereo = capture.depth('stereo', stamp)
                    if stereo is None:
                        lost += 1
                        continue
                    pos_b, quat_b = camera_in_base(capture, stamp)
                    cam_pos, cam_quat = camera_in_world(pos_b, quat_b, robot_in_world())
                    views += 1
                    results += view_rows(records, face_slots, by_name, sg.NORMALS[slot['face']], cam_pos, cam_quat,
                                         {'B': (stereo, labels), 'C': (stereo, predict(model, image, device))}, intr,
                                         {'arrangement': f'{unit}-{t}', 'shot': k})
                    json.dump(results, open(args.out, 'w'))
                print(f'{unit}-{t}: {views} views, {len(results)} articles tried, {lost} lost, {time.monotonic() - t0:.0f} s', flush=True)
    finally:
        try:
            stock.clear_current()
        except Exception as err:
            print('the articles could not be removed:', err)
        json.dump(results, open(args.out, 'w'))
        for node in (arm, capture, nav):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
