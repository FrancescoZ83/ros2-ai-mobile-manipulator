#!/usr/bin/env python3
"""Generate the segmentation dataset in the running simulation.

Needs: the simulation with stereo:=true dataset:=true, Nav2 with the saved map, and move_group.
The articles of the warehouse are removed (restart the simulation to get them back) and the shelves are filled at
random, again and again. For every look pose of the camera, several random fillings are photographed.
"""
import argparse
import math
import os
import random
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

import semantic_goals as sg
from dataset_common import Capture, Stats, Stock, pixel_counts, save, split_of
from moveit_check import Checker, joint_constraints, look_solution, named_states


def navigate(nav, x, y, yaw, timeout=120.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = x, y
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    nav.goToPose(pose)
    start = time.monotonic()
    while not nav.isTaskComplete():
        if time.monotonic() - start > timeout:
            nav.cancelTask()
            return False
    return nav.getResult() == TaskResult.SUCCEEDED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='directory of the dataset')
    ap.add_argument('--units', nargs='+', help='shelf units to photograph (default: all)')
    ap.add_argument('--rounds', type=int, default=1, help='how many times every unit is visited')
    ap.add_argument('--targets', type=int, default=3, help='look poses per visit')
    ap.add_argument('--restocks', type=int, default=4, help='random fillings photographed per look pose')
    ap.add_argument('--dist', type=float, nargs=2, default=[0.15, 0.40], help='range of the TCP distance to the slot [m]')
    ap.add_argument('--view-half-width', type=float, default=0.4, help='articles within this distance of the camera along the aisle are refilled [m]')
    ap.add_argument('--first-index', type=int, default=0, help='number of the first image: to add to a dataset that already exists')
    ap.add_argument('--seed', type=int, default=1, help='use another seed for every run on the same directory')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    units = args.units or list(sem['shelves'])
    rng = random.Random(args.seed)

    rclpy.init()
    arm, capture, nav = Checker(), Capture(), BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    stock = Stock(sem, classes, rng)
    stock.clear_original()
    stats = Stats()
    index, failures, t0 = args.first_index, 0, time.monotonic()
    stow = joint_constraints(named_states()['travel'])
    try:
        for visit in range(args.rounds):
            for unit in rng.sample(units, len(units)):
                _, (x, y, yaw) = sg.resolve(sem, None, unit)
                arm.execute(stow, 1.0)
                if not navigate(nav, x, y, yaw):
                    print(f'{unit}: the robot did not reach the station, skipped', flush=True)
                    failures += 1
                    continue
                pick = [n for n, s in sem['slots'].items() if s['shelf'] == unit and s['pickable']]
                for t in range(args.targets):
                    pose_id = f's{args.seed}-{unit}-{visit}-{t}'
                    reached = False
                    for attempt in range(4):             # some random targets are out of reach: try another
                        slot_name = rng.choice(pick)
                        slot = sem['slots'][slot_name]
                        dist = rng.uniform(*args.dist)
                        offset = (rng.uniform(-0.03, 0.03), 0.0, rng.uniform(-0.03, 0.03))
                        center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + 0.085)
                        sol = look_solution(arm, center, sg.NORMALS[slot['face']], dist, offset)
                        if sol is None:
                            print(f'{pose_id}: no collision-free look pose for {slot_name} at {dist:.2f} m (attempt {attempt + 1})', flush=True)
                            continue
                        res = arm.execute(joint_constraints(sol), 1.0)[0]
                        if res.result.error_code.val != 1:
                            print(f'{pose_id}: MoveIt failed with {res.result.error_code.val} (attempt {attempt + 1})', flush=True)
                            continue
                        reached = True
                        break
                    if not reached:
                        failures += 1
                        continue
                    split = split_of(pose_id)
                    for k in range(args.restocks):
                        records, fill = stock.refill(unit, center[0], args.view_half_width)
                        grabbed = capture.grab()
                        if grabbed is None:
                            failures += 1
                            continue
                        image, labels, stamp = grabbed
                        try:
                            position, quaternion = capture.camera_pose(stamp)
                        except Exception as err:        # the pose at the time of the frame is not known: no sample
                            print(f'{pose_id}: no camera pose at the time of the frame ({err}), skipped', flush=True)
                            failures += 1
                            continue
                        pixels = pixel_counts(labels)
                        meta = {'index': index, 'group': pose_id, 'unit': unit, 'target_slot': slot_name, 'dist': round(dist, 3), 'fill': round(fill, 2),
                            'camera_in_map': {'position': position, 'quaternion': quaternion},
                            'articles': records, 'pixels': pixels}
                        save(args.out, split, index, image, labels, meta)
                        stats.add(split, pose_id, pixels)
                        index += 1
                        if (index - args.first_index) % 10 == 0:
                            print(f'{index} images, {(time.monotonic() - t0) / (index - args.first_index):.1f} s each, {failures} failures', flush=True)
    finally:
        stock.clear_current()
        images = stats.write(args.out, args.seed, args.first_index, failures, time.monotonic() - t0)
        print(f'done: {images} images in {time.monotonic() - t0:.0f} s, {failures} failures; stats in {args.out}/stats_seed{args.seed}.json')
        for node in (arm, capture, nav):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
