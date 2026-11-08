#!/usr/bin/env python3
"""The skills of the mission, as ROS 2 actions (amr_manipulator_task/action): the leaves of the behavior tree of mission_executor.

  skills/navigate_to   drive to the station in front of a slot or a bin (Nav2), refine the localization
  skills/pick_article  from the station: look (images, shift of the map, inventory and neighbours from the image), pre-grasp,
                       approach, close, retreat (Picker.pick of pick_oracle.py)
  skills/carry_article the article is attached to the TCP in the planning scene, the arm goes to the travel pose, held is checked
  skills/place_in_bin  over the bin, release, back (Picker.place)
  skills/recover       open the gripper and stow the arm, or stow it keeping the article

One skill at a time. Needs the simulation, Nav2, move_group and the stereo node (mission.sh starts them). The article is found in the
simulator by its slot (the model of the stock, to make it dynamic when it is picked, as pick_oracle.py does) and nothing else of the
stock is used: the neighbours and the inventory come from the images.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
import traceback

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pick_oracle as po                                          # noqa: E402
import semantic_goals as sg                                       # noqa: E402
import stocker                                                    # noqa: E402
from dataset_common import Stock                                  # noqa: E402
from amr_manipulator_task.action import CarryArticle, NavigateTo, PickArticle, PlaceInBin, Recover   # noqa: E402


class Skills(Node):
    def __init__(self, picker):
        super().__init__('skill_server')
        self.picker = picker
        self.sem = picker.sem
        self.job = None                                           # the article being handled: rec, cls, dz
        self.carrying = False                                     # an article is in the gripper, carried (set by the carry, cleared when it is placed or released)
        self.last_row = None
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        for kind, name, callback in ((NavigateTo, 'navigate_to', self.navigate_to), (PickArticle, 'pick_article', self.pick_article),
                                     (CarryArticle, 'carry_article', self.carry_article), (PlaceInBin, 'place_in_bin', self.place_in_bin),
                                     (Recover, 'recover', self.recover)):
            ActionServer(self, kind, 'skills/' + name, execute_callback=self.guarded(kind, callback), callback_group=group)

    def guarded(self, kind, callback):
        """One skill at a time; an exception is a failed skill with its message as the outcome, not a dead server."""
        def run(goal_handle):
            with self.lock:
                result = kind.Result()
                try:
                    callback(goal_handle.request, result)
                except Exception as err:                          # noqa: BLE001
                    traceback.print_exc()
                    result.success = False
                    result.outcome = f'exception: {err}'
                self.get_logger().info(f'{kind.__name__}: {"ok" if result.success else "failed"} ({result.outcome})')
                goal_handle.succeed()
                return result
        return run

    def trace(self, skill, detail):
        """One line of JSON per skill in /tmp/skills_trace.jsonl: what the skill measured (the result of the action has only the outcome)."""
        with open('/tmp/skills_trace.jsonl', 'a') as f:
            f.write(json.dumps({'time': time.time(), 'skill': skill, 'detail': detail}, default=str) + '\n')

    def find_record(self, slot, class_name):
        """The record of the article that stands in `slot`, from the simulator: the model of the stock whose name ends with the slot."""
        suffix = '_' + slot.replace('.', '_')
        names = [n for n in stocker.models(Stock.PREFIX) if n.endswith(suffix)]
        if not names or class_name not in self.picker.by_name:
            return None
        state = po.entity_state(names[0])
        if state is None:
            return None
        (x, y, z), q = state
        true_class = class_name
        try:                                                      # what the simulator really put there (mission_setup.py), which an order may get wrong
            true_class = json.load(open('/tmp/mission_stock.json')).get(slot, class_name)
        except OSError:
            pass
        cls = self.picker.by_name[true_class]
        return {'name': names[0], 'class': true_class, 'ordered_class': class_name, 'label': cls['label'], 'slot': slot,
                'pose_map': [float(x), float(y), float(z), 2.0 * math.atan2(q[2], q[3])]}

    def watch_drive(self, stop, samples):
        """Every second of a drive with an article: where the robot and the article are in the simulator (to see where and when it falls)."""
        while not stop.is_set():
            try:
                x, y, _, yaw = po.robot_gt()
                art = po.entity_state(self.job['rec']['name'])
                samples.append({'t': round(time.time(), 1), 'robot': [round(x, 2), round(y, 2), round(yaw, 2)],
                                'article': None if art is None else [round(float(v), 2) for v in art[0]]})
            except Exception:                                 # noqa: BLE001
                pass
            stop.wait(1.0)

    def navigate_to(self, goal, result):
        stop, samples = threading.Event(), []
        watcher = None
        if self.carrying and self.job is not None:
            watcher = threading.Thread(target=self.watch_drive, args=(stop, samples), daemon=True)
            watcher.start()
        try:
            self.drive(goal, result)
        finally:
            stop.set()
            if watcher is not None:
                watcher.join(timeout=5.0)
                self.trace('navigate_to', {'place': goal.place, 'outcome': result.outcome, 'row': self.last_row, 'samples': samples})

    def drive(self, goal, result):
        place = goal.place
        station = self.picker.bin_station(place) if place in self.sem['bins'] else sg.resolve(self.sem, None, place)[1]
        row = self.picker.navigate(place, *station)
        self.last_row = {k: (v.name if hasattr(v, 'name') else v) for k, v in row.items()}
        result.error_mm = float(1000 * row['err_xy'])
        result.success = row['result'].name == 'SUCCEEDED'
        result.outcome = 'reached' if result.success else 'failed'
        if result.success:
            self.picker.refine_localization(5)                    # AMCL does not update while it stands still
            self.picker.sync_map_odom()

    def pick_article(self, goal, result):
        rec = self.find_record(goal.slot, goal.article_class)
        if rec is None:
            result.success, result.outcome = False, 'no_article'
            return
        cls = self.picker.by_name[rec['class']]
        self.job = {'rec': rec, 'cls': cls, 'dz': cls.get('grasp_dz', 0.0)}
        self.picker.arm.execute(self.picker.stow, 1.0)
        self.picker.sync_map_odom()
        out = self.picker.pick(rec, self.sem['slots'][goal.slot], [rec])
        result.success = out['outcome'] == 'success'
        result.outcome = out['outcome']
        result.article_to_tcp_mm = float(out.get('article_to_tcp_mm', -1.0))
        result.tilt_deg = float(out.get('tilt_deg', 0.0))
        self.trace('pick_article', {k: out.get(k) for k in ('class', 'slot', 'outcome', 'article_to_tcp_mm', 'tilt_deg', 'width_mm', 'grip_state', 'vision', 'arrival', 'pregrasp', 'approach')})

    def carry_article(self, goal, result):
        if self.job is None:
            result.success, result.outcome = False, 'no_article'
            return
        self.picker.carry_scale = goal.speed_scale if goal.speed_scale > 0.0 else po.CARRY_SCALE
        res = self.picker.carry(self.job['rec'], self.job['cls'], self.job['dz'])
        result.success = res['held']
        self.carrying = res['held']
        result.outcome = res['outcome']
        result.article_to_tcp_mm = float(res['article_to_tcp_mm'] if res['article_to_tcp_mm'] is not None else -1.0)
        self.trace('carry_article', res)

    def place_in_bin(self, goal, result):
        if self.job is None:
            result.success, result.outcome = False, 'no_article'
            return
        res = self.picker.place(self.job['rec'], self.job['cls'], self.job['dz'], goal.bin)
        result.success = res['outcome'] == 'delivered'
        if result.success:
            self.job = None                                       # this article is in the bin: the next order has its own
        self.carrying = self.carrying and not result.success and res['outcome'] not in ('article_lost', 'missed', 'lost_in_drive')
        result.outcome = res['outcome']
        self.trace('place_in_bin', res)
        if 'article' in res:
            result.x, result.y, result.z = (float(v) for v in res['article'])

    def recover(self, goal, result):
        if goal.what == 'release_and_stow':
            if self.job is not None:                              # an article attached to the TCP in the scene is no more
                self.picker.attach_article(self.job['rec'], self.job['cls'], self.job['dz'], attach=False)
            self.picker.release_and_stow()
            self.carrying = False
            result.success, result.outcome = True, 'released'
        elif goal.what == 'stow':
            code = self.picker.arm.execute(self.picker.stow, self.picker.carry_scale)[0].result.error_code.val
            result.success, result.outcome = code == 1, 'stowed' if code == 1 else f'code {code}'
        else:
            result.success, result.outcome = False, f"unknown action '{goal.what}'"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--network', default='base', help='the recognition network segmentation/runs/<name>.pt')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()
    import random
    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    rclpy.init()
    picker = po.Picker(sem, classes, random.Random(args.seed))
    # the setting of the best series: station in front of the slot, best pre-grasp, the shift of the map, the inventory and the neighbours from the images
    picker.opt.update(best_pregrasp=True, nav=True, localize=True, inventory_images=True, scene_images=True, swing_aware=True)
    from vision_target import Vision                              # heavy imports (torch, the network)
    picker.vision = Vision(args.network, po.NAV_PRIOR_RADIUS)
    node = Skills(picker)
    node.get_logger().info('skills ready')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        picker.vision.capture.destroy_node()
        picker.arm.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
