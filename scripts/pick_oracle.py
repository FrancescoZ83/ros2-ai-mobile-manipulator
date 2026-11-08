#!/usr/bin/env python3
"""Pick articles from a shelf with the true pose of the article (no vision), and check what the gripper holds.

One attempt: the arm goes to the pre-grasp pose (10 cm before the grasp pose, tool horizontal into the shelf, fingers
closing along the aisle, tool x up), the gripper opens, a straight line takes the TCP to the centre of the article, the
gripper closes, a straight line takes it back (10 cm back and 2 cm up), and after a second the article is looked for in
the simulator: is it with the gripper, has it left its slot, did the neighbours move? Then the gripper opens and the arm
goes back to the travel pose.

The poses are given to MoveIt in the frame of the robot (`base_footprint`): the true pose of the article in the map is
taken to that frame with the transform map to base_footprint. Needs `pick_oracle.sh`: the simulation with the robot at the
station and an exact map to odom, and move_group.
"""
import argparse
import json
import math
import os
import random
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, Twist, TransformStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'amr_manipulator_perception', 'pose'))
import geometry as geo                                            # noqa: E402
import semantic_goals as sg                                       # noqa: E402
import stocker                                                    # noqa: E402
from dataset_common import Stock                                  # noqa: E402
import grasp_geometry                                             # noqa: E402
from grasp_geometry import grasp_axes, matrix_to_quat             # noqa: E402
from localization import localize, localize_images                 # noqa: E402
from moveit_check import PAN_LIMIT, Checker, joint_constraints, look_candidates, look_solution, named_states   # noqa: E402

APPROACH = 0.10              # [m] pre-grasp distance
RETREAT_UP = 0.02            # [m] the retreat goes this much up, so the article leaves the board
STATES = ['OPEN', 'OPENING', 'CLOSING', 'GRASP', 'EMPTY']
RAMP_WAIT = 0.8              # [s] after the gripper reports GRASP, before the retreat: the grip force is still ramping up
HELD_WITHIN = 0.04           # [m] the article centre is this close to the TCP: it is held
# postures of the pre-grasps that gave a short straight approach (elbow up or down, wrist 1 turned the short way): seeds of the IK
NAV_PRIOR_RADIUS = 0.06    # [m] with Nav2: the believed map is off by the localization error, 3 cm in the first run
GOOD_SEEDS = ([-1.1, -1.3, 1.1, -3.0, -0.45, -1.571], [-0.9, -1.6, 1.5, -3.0, -0.7, -1.571], [-2.0, -1.86, -1.1, -0.18, 0.45, -1.571])
SHORT_LINE = 1.0             # [rad] with --swing-aware: lines whose joint travel is within this of the best are equally good
# [m] TCP distance of the look poses of the vision, tried in this order until the estimate is reliable; at level 1 the look pose is at the
# edge of the reach of the arm: there is IK at 0.24-0.28 m and none from 0.32 m when the robot is 17 mm nearer to the shelf, so 0.26 m comes before 0.22 m
LOOK_DISTS = (0.30, 0.26, 0.22, 0.38)
SEEN_RADIUS = 0.05          # [m] a detection belongs to a slot of the face when it is this close to it once the shift is applied (the jitter of the
                             # stock is 14 mm, the error of the estimate a few more)
SCENE_MARGIN = 0.004         # [m] the articles are a little bigger in the planning scene than they are
PLACE_GAP = 0.15             # [m] platform - table at the packing table: from here the whole drop line over the bin is within reach (place_check.py)
PLACE_BACK = 0.10            # [m] the pre-place pose is this far before the drop point, on the line
PLACE_MARGIN = 0.03          # [m] the lowest point of the article is this far over the rim of the bin at the drop
PLACE_DXS = (-0.04, -0.08, 0.0)  # [m] the drop point is this far from the centre of the bin, towards the robot, the first that has a pre-place pose with a complete line
# postures of the pre-place poses that gave a complete line (wrist 1 turned the long way, as at the shelves); with only the two default seeds the IK found the other branch
PLACE_SEEDS = ([-0.35, -1.27, 1.2, -3.07, -1.17, -1.571], [-3.0, -1.88, -1.2, -0.07, 1.47, -1.571])
HELD_CARRY = 0.12            # [m] a bottle gripped above its centre swings about the pads when the tool turns to the travel pose (it hung 76 mm from where it was
                             # gripped and came back); the article is lost only when it is farther than this from the TCP or the gripper says it is empty
HELD_ARRIVED = 0.06          # [m] at the table, with the tool horizontal again, it has to be near where it was gripped for the drop height to be right
CARRY_SCALE = 0.25           # speed scale of the arm with an article in the gripper: at 0.5 a bottle slipped out of the pads on the way to the travel pose
BIN_RIM = 0.87               # [m] top of the walls of a bin
MOVED = 0.01                 # [m] a neighbour that moved more than this was disturbed


def world_poses():
    """name -> (x, y, z) of every entity in the world, from one pose/info message of the simulator."""
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/amr_world/pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=20).stdout
    poses = {}
    for block in re.findall(r'^pose \{\n(.*?)^\}', out, re.M | re.S):
        name = re.search(r'name: "([^"]+)"', block)
        pos = re.search(r'position \{(.*?)\}', block, re.S)
        if not name:
            continue
        p = pos.group(1) if pos else ''
        vals = [re.search(rf'\b{k}: (\S+)', p) for k in 'xyz']
        poses[name.group(1)] = tuple(float(m.group(1)) if m else 0.0 for m in vals)
    return poses


def entity_state(name):
    """(position, quaternion x y z w) of the entity called name in the world, from the simulator, or None."""
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/amr_world/pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=20).stdout
    for block in re.findall(r'^pose \{\n(.*?)^\}', out, re.M | re.S):
        label = re.search(r'name: "([^"]+)"', block)
        if not label or label.group(1) != name:
            continue
        pos = re.search(r'position \{(.*?)\}', block, re.S)
        ori = re.search(r'orientation \{(.*?)\}', block, re.S)

        def field(text, key, default=0.0):
            m = re.search(rf'\b{key}: (\S+)', text or '')
            return float(m.group(1)) if m else default
        p, o = (pos.group(1) if pos else ''), (ori.group(1) if ori else '')
        return np.array([field(p, k) for k in 'xyz']), [field(o, 'x'), field(o, 'y'), field(o, 'z'), field(o, 'w', 1.0)]
    return None


def robot_gt():
    """(x, y, z, yaw) of the robot model in the world, from the simulator."""
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/amr_world/pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=20).stdout
    for block in re.findall(r'^pose \{\n(.*?)^\}', out, re.M | re.S):
        name = re.search(r'name: "([^"]+)"', block)
        if not name or name.group(1) != 'amr_manipulator':
            continue
        pos = re.search(r'position \{(.*?)\}', block, re.S)
        ori = re.search(r'orientation \{(.*?)\}', block, re.S)

        def field(text, key):
            m = re.search(rf'\b{key}: (\S+)', text or '')
            return float(m.group(1)) if m else 0.0
        o, p = (ori.group(1) if ori else ''), (pos.group(1) if pos else '')
        x, y, z, w = (field(o, k) for k in 'xyzw')
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return field(p, 'x'), field(p, 'y'), field(p, 'z'), yaw
    raise RuntimeError('the robot is not in pose/info')


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Picker:
    def __init__(self, sem, classes, rng):
        self.sem, self.by_name, self.rng = sem, {c['name']: c for c in classes}, rng
        self.arm = Checker()
        self.tf = Buffer()
        TransformListener(self.tf, self.arm)
        self.stock = Stock(sem, classes, rng)
        self.gripper = []                                         # last state of the gripper controller
        self.cmd = self.arm.create_publisher(Float64MultiArray, '/gripper_controller/command', 10)
        self.arm.create_subscription(Float64MultiArray, '/gripper_controller/state', self.on_gripper, 10)
        self.stow = joint_constraints(named_states()['travel'])
        self.vision = None                                        # a vision_target.Vision: the pose of the article from the images
        # what is tried: grip force (0 = the controller's 25 N), the grasp point moved by dz (up) and dd (along the approach),
        # the retreat (up, and the speed scale), the classes
        self.opt = {'force': 0.0, 'dz': None, 'dd': 0.0, 'retreat_up': RETREAT_UP, 'retreat_scale': 0.3, 'classes': None, 'levels': None, 'lift_first': 0.0, 'best_pregrasp': False, 'nav': False, 'localize': False, 'swing_aware': False, 'trace_tilt': False, 'scene_neighbours': True, 'align_yaw': False, 'yaw_deg': None, 'yaw_error_deg': 0.0, 'inventory_images': False, 'scene_images': False}
        # map to odom: set from the simulator, not fixed. The base moves a few centimetres when the arm swings (the wheels
        # do not see it), so a fixed transform puts the shelves of the planning scene in the wrong place.
        self.broadcaster = TransformBroadcaster(self.arm)
        self.map_odom = None
        self.arm.create_timer(0.05, self.publish_map_odom)
        self.apply_scene = self.arm.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.arm.create_client(GetPlanningScene, '/get_planning_scene')
        self.shift = np.zeros(2)                                  # with --localize: the shift of the believed map found from the images (x, y)
        self.world_base = None                                    # the objects of the scene as move_group had them, before any shift
        self.scene_ids = []
        self.carry_scale = CARRY_SCALE                            # speed scale of the arm with an article in the gripper
        self.navigator = None                                     # a nav2_simple_commander BasicNavigator, made when the first drive is asked for
        # A zero velocity command all the time, as Nav2 sends when it is not driving: the wheels hold the base while the arm swings
        self.cmd_vel = self.arm.create_publisher(Twist, '/cmd_vel', 10)
        self.arm.create_timer(0.05, lambda: self.cmd_vel.publish(Twist()))

    def publish_map_odom(self):
        if self.map_odom is None:
            return
        msg = TransformStamped()
        msg.header.stamp = self.arm.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'map', 'odom'
        (msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z) = self.map_odom[0]
        (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w) = self.map_odom[1]
        self.broadcaster.sendTransform(msg)

    def believed_pose(self):
        """(x, y, z, yaw) of base_footprint in the map as the transforms say (AMCL and the odometry), not the simulator."""
        self.spin(0.3)
        t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        q = t.rotation
        return (t.translation.x, t.translation.y, t.translation.z,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def stamp_pose(self, out, stage):
        """Note the pose of the robot at a stage of the attempt: the simulator's in out['robot_gt'] and, with --nav, the one
        the transforms give in out['robot_belief']. Returns the pose to work with (see sync_map_odom)."""
        pose = self.sync_map_odom()
        out['robot_gt'][stage] = np.round(robot_gt() if self.opt['nav'] else pose, 4).tolist()
        if self.opt['nav']:
            out.setdefault('robot_belief', {})[stage] = np.round(pose, 4).tolist()
        return pose

    def sync_map_odom(self):
        """Put map to odom where the simulator says the robot is (the arm must be at rest). Returns the robot pose.
        With --nav the transform is AMCL's, nothing is published and the pose returned is the one the transforms give."""
        if self.opt['nav']:
            return self.believed_pose()
        x, y, z, yaw = robot_gt()
        self.spin(0.3)
        t = self.tf.lookup_transform('odom', 'base_footprint', rclpy.time.Time()).transform
        r_ob = geo.quat_to_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        t_ob = np.array([t.translation.x, t.translation.y, t.translation.z])
        r_mo = rot_z(yaw) @ r_ob.T
        t_mo = np.array([x, y, z]) - r_mo @ t_ob
        self.map_odom = (t_mo.tolist(), geo.matrix_to_quat(r_mo).tolist())
        self.spin(0.5)                                            # the transform is published and received
        return x, y, z, yaw

    def reset_base(self, station, tolerance=0.01):
        """Put the robot back at its station when the simulator says it is elsewhere (its wheels do not hold it: it slides and
        turns when the arm swings or pushes). Teleport with the set_pose service, checked on the pose of the simulator.
        Returns (distance [m], yaw error [rad]) before the move."""
        x, y, yaw = station
        gx, gy, gz, gyaw = robot_gt()
        off = (math.hypot(gx - x, gy - y), abs((gyaw - yaw + math.pi) % (2 * math.pi) - math.pi))
        for _ in range(3):
            if math.hypot(robot_gt()[0] - x, robot_gt()[1] - y) < tolerance:
                break
            stocker.set_pose('amr_manipulator', x, y, gz, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            self.spin(2.0)
        return off

    def localization_error(self):
        """(distance [mm], yaw [degrees]) between the pose the transforms give and the simulator's."""
        bx, by, _, byaw = self.believed_pose()
        gx, gy, _, gyaw = robot_gt()
        return (round(1000 * math.hypot(bx - gx, by - gy), 1),
                round(math.degrees(abs((byaw - gyaw + math.pi) % (2 * math.pi) - math.pi)), 2))

    def refine_localization(self, updates):
        """AMCL does not update while the robot stands still (it needs 0.25 m or 0.2 rad of motion): ask for `updates` updates
        without motion, each with the scan of that moment. Returns the localization error before and after."""
        before = self.localization_error()
        client = self.arm.create_client(Empty, '/request_nomotion_update')
        done = 0
        if client.wait_for_service(timeout_sec=5.0):
            for _ in range(updates):
                rclpy.spin_until_future_complete(self.arm, client.call_async(Empty.Request()), timeout_sec=5.0)
                self.spin(1.0)
                done += 1
        self.spin(1.0)
        after = self.localization_error()
        return {'updates': done, 'before_mm': before[0], 'before_yaw_deg': before[1], 'after_mm': after[0], 'after_yaw_deg': after[1]}

    def offset_from(self, station):
        """(distance [m], yaw error [rad]) of the robot, in the simulator, from the station; nothing is moved."""
        gx, gy, gz, gyaw = robot_gt()
        return math.hypot(gx - station[0], gy - station[1]), abs((gyaw - station[2] + math.pi) % (2 * math.pi) - math.pi)

    def gt_map_to_base(self):
        """(rotation, translation) that take a point of the map to base_footprint, from the simulator."""
        x, y, z, yaw = robot_gt()
        r = rot_z(-yaw)
        return r, -r @ np.array([x, y, z])

    def on_gripper(self, msg):
        if len(msg.data) == 9:
            self.gripper = list(msg.data)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.arm, timeout_sec=0.05)

    def state(self):
        self.spin(0.2)
        return STATES[int(self.gripper[0])] if self.gripper else None

    def wait_state(self, wanted, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state() in wanted:
                return True
        return False

    def command(self, close, force=0.0):
        for _ in range(3):                                        # the publisher needs a moment to be matched
            self.cmd.publish(Float64MultiArray(data=[1.0 if close else 0.0, force]))
            self.spin(0.1)

    def map_to_base(self):
        """(rotation, translation) that take a point of the map to base_footprint."""
        self.spin(0.3)
        t = self.tf.lookup_transform('base_footprint', 'map', rclpy.time.Time()).transform
        return (geo.quat_to_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
                np.array([t.translation.x, t.translation.y, t.translation.z]))

    def tcp_in_map(self):
        pos, _ = self.arm.forward(self.arm.current())
        r, t = self.gt_map_to_base()
        return r.T @ (np.array(pos) - t)

    def relative_state(self, name, height, approach_map):
        """Where the article is with respect to the TCP, in mm along the approach (depth), across (along the row) and up, and
        how much it is tilted from the vertical [degrees]: read in the simulator."""
        state = entity_state(name)
        if state is None:
            return None
        origin, q = state
        centre = origin + geo.quat_to_matrix(q) @ np.array([0.0, 0.0, height / 2.0])
        rel = centre - self.tcp_in_map()
        approach = np.array(approach_map)
        across = np.cross(approach, [0.0, 0.0, 1.0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)))))
        return {'depth_mm': round(1000 * float(rel @ approach), 1), 'across_mm': round(1000 * float(rel @ across), 1),
                'up_mm': round(1000 * float(rel[2]), 1), 'tilt_deg': round(tilt, 1)}

    def line_to(self, target, quat, scale=0.3):
        """Straight line of the TCP to `target` (frame of the robot), then executed. (ok, fraction or error code)."""
        pos, _ = self.arm.forward(self.arm.current())
        res = self.arm.straight_line(pos, quat, tuple(target), scale)
        pts = res.solution.joint_trajectory.points
        if res.error_code.val != 1 or res.fraction < 0.999 or len(pts) < 2:
            return False, round(res.fraction, 3)
        first, last = np.array(pts[0].positions), np.array(pts[-1].positions)
        end = pts[-1].time_from_start
        plan = {'points': len(pts), 'seconds': round(end.sec + end.nanosec * 1e-9, 2),
                'joint_change': [round(float(v), 2) for v in (last - first)],
                'largest_step': round(float(max(np.abs(np.array(b.positions) - np.array(a.positions)).max() for a, b in zip(pts, pts[1:]))), 3)}
        code = self.arm.run_path(res.solution)
        self.spin(1.0)                                            # the arm settles; the server judges after 0.2 s of wall time
        err = float(np.linalg.norm(np.array(self.arm.forward(self.arm.current())[0]) - np.array(target)))
        return (code == 1 or err < 0.008), {'code': code, 'tcp_error_mm': round(1000 * err, 1), 'plan': plan}

    def choose_pregrasp(self, candidates, centre, quat):
        """The IK solutions of the pre-grasp whose straight approach to `centre` (frame of the robot) is complete, the one
        that moves the joints least first. With --swing-aware, among the solutions whose line moves the joints at most
        SHORT_LINE rad more than the best one, the one nearest to where the arm is now goes first: the arm has to get to the
        pre-grasp from the look pose, and a solution with the base joint turned the other way round is a swing of 3 rad, which
        the planner often cannot plan (-2) or the arm cannot follow (-4). Returns (list of joints, list of what each candidate gave)."""
        now = np.array(self.arm.current())
        seen, report = [], []
        for cand in candidates:
            if any(max(abs(a - b) for a, b in zip(cand, other)) < 0.02 for other in seen):
                continue                                          # the same solution again
            seen.append(cand)
            res = self.arm.straight_line(None, quat, tuple(centre), 0.3, start=cand)
            pts = res.solution.joint_trajectory.points
            complete = res.error_code.val == 1 and res.fraction >= 0.999 and len(pts) >= 2
            travel = float(sum(np.abs(np.array(b.positions) - np.array(a.positions)).sum() for a, b in zip(pts, pts[1:]))) if complete else None
            report.append({'joints': [round(float(v), 3) for v in cand], 'fraction': round(res.fraction, 3),
                           'points': len(pts), 'travel': None if travel is None else round(travel, 2),
                           'swing': round(float(np.abs(np.array(cand) - now).sum()), 2)})
        good = sorted((r['travel'], r['joints']) for r in report if r['travel'] is not None)
        if self.opt['swing_aware'] and good:
            short = good[0][0] + SHORT_LINE
            by = {tuple(r['joints']): r for r in report}
            near = sorted((j for t, j in good if t <= short), key=lambda j: by[tuple(j)]['swing'])
            good = [(0, j) for j in near] + [(t, j) for t, j in good if t > short]
        return [joints for _, joints in good], report

    def pick(self, rec, slot, records):
        """One attempt on the article `rec` (a record of Stock.refill), standing in `slot`; `records` are all the articles
        of the shelves. Returns a dict."""
        cls = self.by_name[rec['class']]
        height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
        ax, ay, az, _ = rec['pose_map']
        centre_map = np.array([ax, ay, az + height / 2.0])
        out = {'class': rec['class'], 'slot': rec['slot'], 'article': rec['name'], 'robot_gt': {}, 'rel': {}}
        dz = self.opt['dz'] if self.opt['dz'] is not None else cls.get('grasp_dz', 0.0)    # the grasp point above the centre
        out['grasp_dz'] = dz
        self.stamp_pose(out, 'start')
        if self.world_base is not None:                           # --localize: the scene as it was, not as the last attempt shifted it
            self.shift = np.zeros(2)
            self.shift_world(self.shift)
        # All the articles in the planning scene, the one to pick too: the paths of the look pose and of the pre-grasp must
        # not sweep through it (a bottle at level 3 was knocked off the shelf by the arm on the way to the pre-grasp). It is
        # taken out before the approach, which has to go in.
        # With --scene-images the neighbours are not known yet: before the first look the scene has the article to pick at its
        # nominal place (the task says which one it is), after it the ones that the image shows (scene_from_images).
        scene = self.nominal_scene(rec, slot) if self.opt['scene_images'] else records
        self.last_seen = None
        out['in_scene'] = self.set_scene_articles(scene, None)
        approach_map, rot_map = self.grasp_frame(rec, slot)

        def targets():
            """Orientation, approach direction, grasp point and pre-grasp point in the frame of the robot, from where the
            simulator says the robot is now."""
            r, t = self.map_to_base() if self.opt['nav'] else self.gt_map_to_base()
            quat = matrix_to_quat((r @ np.array(rot_map)).tolist())
            approach = r @ np.array(approach_map)
            centre = r @ centre_map + t + dz * (r @ np.array([0.0, 0.0, 1.0])) + self.opt['dd'] * approach
            return quat, approach, centre, centre - APPROACH * approach

        quat, approach, centre, pre = targets()
        self.make_dynamic(rec)
        before = world_poses()
        start = np.array(before[rec['name']])
        others = {n: p for n, p in before.items() if n.startswith(Stock.PREFIX) and n != rec['name']}

        def finish(result):
            """Which articles moved, and where the links of the arm are, whatever the outcome."""
            later = world_poses()
            result['moved_mm'] = {n: round(1000 * float(np.linalg.norm(np.array(later[n]) - np.array(p))), 1)
                                  for n, p in before.items() if n.startswith(Stock.PREFIX) and n in later
                                  and np.linalg.norm(np.array(later[n]) - np.array(p)) > MOVED}
            result['arm_links_gt'] = {n: np.round(p, 3).tolist() for n, p in later.items() if n.startswith(('ur_', 'gripper'))}
            return result

        self.command(close=False)
        self.wait_state(('OPEN',), 3.0)
        if self.vision is not None:                               # the pose of the article is the one the images give
            estimate = self.see(rec, cls, slot, height, records, out, scene)
            if estimate is None:
                out['outcome'] = 'vision_failed'
                return finish(out)
            centre_map = np.array([estimate['x'], estimate['y'], centre_map[2]])     # the height: the shelf and the class
            if self.opt['scene_images']:
                scene = self.scene_from_images(rec, records, out)
            self.sync_map_odom()
            quat, approach, centre, pre = targets()
        # The pre-grasp is a look pose at APPROACH from the article: the joints come from look_solution (IK seeded near
        # the right posture, normalised, the most comfortable one) and the goal is in joints. A goal of pose lets MoveIt
        # pick any branch of the IK, and the paths wind the elbow and wrist 1 by several turns.
        if self.opt['best_pregrasp']:
            # ... or, with --best-pregrasp, the solution whose approach line is complete with the least joint travel
            candidates = look_candidates(self.arm, tuple(centre_map), sg.NORMALS[slot['face']], APPROACH, extra_seeds=GOOD_SEEDS)
            self.set_scene_articles(scene, rec['name'])           # the approach is judged as it will be run: the article is not an obstacle
            ranked, out['candidates'] = self.choose_pregrasp(candidates, centre, quat)
            self.set_scene_articles(scene, None)                 # but on the way to the pre-grasp it is
            if not candidates:
                out['pregrasp'] = 'no collision-free IK'
            elif not ranked:
                out['pregrasp'] = 'no solution with a complete approach'
        else:
            joints = look_solution(self.arm, tuple(centre_map), sg.NORMALS[slot['face']], APPROACH)
            ranked = [] if joints is None else [joints]
            if joints is None:
                out['pregrasp'] = 'no collision-free IK'
        if not ranked:
            out['outcome'] = 'pregrasp_failed'
            return finish(out)
        codes = []
        for joints in ranked[:3]:        # the next best solution if the planner cannot reach this one
            out['pregrasp_joints'] = [round(float(v), 3) for v in joints]
            for _ in range(4):           # the planner sometimes returns a path that touches the shelf (-2): plan again
                res, _ = self.arm.execute(joint_constraints(joints), 0.5)
                codes.append(res.result.error_code.val)
                if codes[-1] == 1:
                    break
            if codes[-1] == 1:
                break
        out['pregrasp'] = codes
        if codes[-1] != 1:
            out['outcome'] = 'pregrasp_failed'
            return finish(out)
        out['rel']['after_pregrasp'] = self.relative_state(rec['name'], height, approach_map)
        self.set_scene_articles(scene, rec['name'])               # now the target is not an obstacle: the fingers go around it
        # The base may have moved while the arm swung: the grasp point is taken again from the simulator
        self.stamp_pose(out, 'pregrasp')
        quat, approach, centre, pre = targets()
        ok, info = self.line_to(centre, quat)
        out['approach'] = info
        if not ok:
            out['outcome'] = 'approach_failed'
            return finish(out)
        out['robot_gt']['grasp'] = np.round(robot_gt(), 4).tolist()
        out['tcp_error_mm'] = info['tcp_error_mm']
        poses = world_poses()                                     # where everything really is, from the simulator
        out['diag'] = {'article_centre_gt': (np.array(poses[rec['name']]) + [0, 0, height / 2.0]).round(4).tolist(),
                       'tcp_in_world_from_fk_and_gt': self.tcp_in_map().round(4).tolist(),
                       'tcp_in_robot_frame': np.round(self.arm.forward(self.arm.current())[0], 4).tolist()}
        out['rel']['before_closing'] = self.relative_state(rec['name'], height, approach_map)
        tracer = None
        if self.opt['trace_tilt']:                                # every pose of the world from the closing to the end of the retreat
            out['trace_file'] = f'/tmp/trace_{rec["name"]}_{int(time.time())}.txt'
            tracer = subprocess.Popen(['ign', 'topic', '-e', '-t', '/world/amr_world/pose/info'],
                                      stdout=open(out['trace_file'], 'w'), stderr=subprocess.DEVNULL)
            self.spin(0.5)
        self.command(close=True, force=self.opt['force'])
        self.wait_state(('GRASP', 'EMPTY'), 4.0)
        out['grip_state'] = self.state()
        out['width_mm'] = round(1000 * self.gripper[6], 1) if self.gripper else None
        if out['grip_state'] == 'GRASP':
            self.spin(RAMP_WAIT)                                  # the force ramps to its value in grip_ramp (0.25 s): wait for it
            out['force_N'] = [round(self.gripper[4], 1), round(self.gripper[5], 1)]
        out['rel']['closed'] = self.relative_state(rec['name'], height, approach_map)
        up = self.gt_map_to_base()[0] @ np.array([0.0, 0.0, 1.0])
        lift = self.opt['lift_first']
        if lift > 0.0:                                            # straight up first, so that the article leaves the board
            here = np.array(self.arm.forward(self.arm.current())[0])
            ok, out['lift'] = self.line_to(here + lift * up, quat, self.opt['retreat_scale'])
            out['rel']['lifted'] = self.relative_state(rec['name'], height, approach_map)
        ok, info = self.line_to(pre + (self.opt['retreat_up'] + lift) * up, quat, self.opt['retreat_scale'])
        out['retreat'] = info
        self.spin(1.0)
        if tracer is not None:
            tracer.terminate()
            tracer.wait(timeout=10)
        out['rel']['retreated'] = self.relative_state(rec['name'], height, approach_map)
        after = world_poses()
        article = np.array(after[rec['name']])
        rel = out['rel']['retreated'] or {'depth_mm': 1e9, 'across_mm': 0.0, 'up_mm': 0.0, 'tilt_deg': 0.0}
        # the article is expected below the TCP by dz and behind it by dd (the grasp point is moved by them)
        dev = (rel['depth_mm'] + 1000 * self.opt['dd'], rel['across_mm'], rel['up_mm'] + 1000 * dz)
        out['article_to_tcp_mm'] = round(math.sqrt(sum(v * v for v in dev)), 1)       # how far it is from where it should be
        out['tilt_deg'] = rel['tilt_deg']                         # the centre is taken from the pose and the orientation of the article
        out['article_moved_mm'] = round(1000 * float(np.linalg.norm(article - start)), 1)
        out['lifted_mm'] = round(1000 * float(article[2] - start[2]), 1)
        out['neighbours_moved'] = sorted(n for n, p in others.items()
                                         if n in after and np.linalg.norm(np.array(after[n]) - np.array(p)) > MOVED)
        held = out['grip_state'] == 'GRASP' and out['article_to_tcp_mm'] < 1000 * HELD_WITHIN and out['article_moved_mm'] > 50
        out['outcome'] = 'success' if held else ('empty' if out['grip_state'] == 'EMPTY' else 'lost')
        return finish(out)

    def shift_world(self, shift):
        """Move the static objects of the planning scene (shelves, walls, benches: boxes in the frame map) to `shift` from where
        move_group had them. They are drawn in the believed map, so with a localization error along the shelf they stand
        that far from where the shelf is for the robot; the shift found from the images puts them back. Always absolute:
        the first call keeps the objects as they are, the next ones start from those."""
        if self.world_base is None:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = self.get_scene.call_async(request)
            rclpy.spin_until_future_complete(self.arm, future, timeout_sec=10.0)
            objects = future.result().scene.world.collision_objects
            self.world_base = [o for o in objects if o.id not in self.scene_ids and not o.id.startswith(Stock.PREFIX)]
        moved = []
        for base in self.world_base:
            obj = CollisionObject(id=base.id, operation=CollisionObject.ADD)
            obj.header = base.header
            obj.primitives = base.primitives
            for at in base.primitive_poses:
                new = Pose()
                new.position.x, new.position.y, new.position.z = at.position.x + shift[0], at.position.y + shift[1], at.position.z
                new.orientation = at.orientation
                obj.primitive_poses.append(new)
            moved.append(obj)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = moved
        future = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self.arm, future, timeout_sec=10.0)
        return len(moved)

    def set_scene_articles(self, records, keep_out):
        """Put the articles in the planning scene of MoveIt, except `keep_out` (the one to pick), after taking away the ones of
        the last attempt: the planner and the IK then keep the arm away from the neighbours, which the arm used to hit (they
        are static: the reaction went to the base, that slid and turned by up to a metre)."""
        old = [CollisionObject(id=i, operation=CollisionObject.REMOVE) for i in self.scene_ids]
        new = []
        for rec in (records if self.opt['scene_neighbours'] else []):
            if rec['name'] == keep_out:
                continue
            cls = self.by_name[rec['class']]
            x, y, z, yaw = rec['pose_map']
            obj = CollisionObject(id=rec['name'], operation=CollisionObject.ADD)
            obj.header.frame_id = 'map'
            if cls['shape'] == 'box':
                sx, sy, sz = (v + 2 * SCENE_MARGIN for v in cls['size'])
                prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])
                height = cls['size'][2]
            else:
                radius, height_c = cls['size']
                prim = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height_c + 2 * SCENE_MARGIN, radius + SCENE_MARGIN])
                height = height_c
            pose = Pose()
            shift = (0.0, 0.0) if rec.get('believed') else self.shift  # a record from the images is already in the believed map
            pose.position.x, pose.position.y, pose.position.z = x + shift[0], y + shift[1], z + height / 2.0
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)
            new.append(obj)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = old + new
        request = ApplyPlanningScene.Request(scene=scene)
        future = self.apply_scene.call_async(request)
        rclpy.spin_until_future_complete(self.arm, future, timeout_sec=10.0)
        self.scene_ids = [o.id for o in new]
        return len(new)

    def nominal_scene(self, rec, slot):
        """The scene before the first look: only the article to pick, at the nominal place of its slot."""
        return [{'name': rec['name'], 'class': rec['class'], 'slot': rec['slot'],
                 'pose_map': [slot['pose'][0], slot['pose'][1], slot['pose'][2], 0.0]}]

    def scene_from_images(self, rec, records, out):
        """The articles of the planning scene as the last look saw them (--scene-images): one record per detection that falls on
        a slot of the face and level (within SEEN_RADIUS of it once the shift is applied), with the class and the x, y, yaw of
        the estimate (already in the believed map: 'believed') and the height of the board of the slot. The article to pick
        keeps its name, so that it can be left out of the approach. What is not in the image is not in the scene: the slots
        beyond the view, the ones whose mask touches the border, the other levels. `out['scene_images']` says what that is
        against the stock of the simulator (which is not used)."""
        seen = self.last_seen
        mine = self.sem['slots'][rec['slot']]
        # every slot of the board, both faces: the camera sees through the unit and the articles of the other face are 0.5 m behind
        grid = {n: {'slot': n, 'x': g['pose'][0], 'y': g['pose'][1]} for n, g in self.sem['slots'].items()
                if (g['shelf'], g['level']) == (mine['shelf'], mine['level'])}
        taken = {}
        unmatched = 0
        loose = []                                                # the detections that no slot takes, to see what they are
        for i, det in enumerate(seen['detections']):
            here = np.array([det['x'], det['y']])
            if i == seen['target']:
                name = rec['slot']
            else:
                dist = {n: float(np.linalg.norm(here - np.array([g['x'], g['y']]) - seen['shift'])) for n, g in grid.items()}
                name = min(dist, key=dist.get)
                if dist[name] > SEEN_RADIUS or name == rec['slot']:
                    unmatched += 1
                    loose.append({'class': det['class'], 'nearest': name, 'distance_mm': round(1000 * dist[name], 1),
                                  'offset_mm': [round(1000 * float(v), 1) for v in here - np.array([grid[name]['x'], grid[name]['y']]) - seen['shift']],
                                  'points': det['points'], 'fraction': det['fraction']})
                    continue
            if name in taken and i != seen['target']:
                unmatched += 1
                loose.append({'class': det['class'], 'nearest': name, 'taken_twice': True})
                continue
            taken[name] = det
        scene = []
        for name, det in taken.items():
            z = self.sem['slots'][name]['pose'][2]
            scene.append({'name': rec['name'] if name == rec['slot'] else f'seen_{name}', 'class': det['class'], 'slot': name,
                          'pose_map': [float(det['x']), float(det['y']), z, float(det['yaw'])], 'believed': True})
        stock = {r['slot']: r for r in records if r['name'] != rec['name']}
        face = {n for n, g in self.sem['slots'].items() if (g['shelf'], g['level'], g['face']) == (mine['shelf'], mine['level'], mine['face'])}
        out['scene_images'] = {
            'objects': len(scene), 'unmatched': unmatched, 'loose': loose, 'detections': len(seen['detections']),
            'missing': sorted(n for n in stock if n not in taken),
            'missing_on_face': sorted(n for n in stock if n not in taken and n in face),
            'wrong_class': sorted(n for n in taken if n in stock and stock[n]['class'] != taken[n]['class']),
            'extra': sorted(n for n in taken if n not in stock and n != rec['slot'])}
        return scene

    def navigate(self, name, x, y, yaw, timeout=200.0):
        """Drive to a pose of the map with Nav2 (nav_goto.run_goal): the dict of its measurements."""
        import nav_goto
        from nav2_simple_commander.robot_navigator import BasicNavigator
        from rclpy.parameter import Parameter
        if self.navigator is None:
            self.navigator = BasicNavigator()
            self.navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.nav_buffer = Buffer()
            TransformListener(self.nav_buffer, self.navigator)
        return nav_goto.run_goal(self.navigator, self.nav_buffer, name, x, y, yaw, timeout)

    def attach_article(self, rec, cls, dz, attach=True):
        """The article in the gripper as an object attached to the TCP in the planning scene of MoveIt (a box of its size, its centre
        dz below the TCP; the tool x axis is up, y across the fingers, z the approach), so that the paths of the carry do not sweep
        it through the shelf or the bin. attach=False takes it away (it has been released)."""
        if cls['shape'] == 'box':
            width, depth, height = cls['size']
        else:
            width = depth = 2 * cls['size'][0]
            height = cls['size'][1]
        aco = AttachedCollisionObject(link_name='gripper_tcp', touch_links=['gripper_tcp', 'gripper_base_link', 'gripper_left_finger_link', 'gripper_right_finger_link'])
        aco.object.id = 'held_' + rec['name']
        aco.object.header.frame_id = 'gripper_tcp'
        if attach:
            aco.object.operation = CollisionObject.ADD
            aco.object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[height + 2 * SCENE_MARGIN, width + 2 * SCENE_MARGIN, depth + 2 * SCENE_MARGIN]))
            pose = Pose()
            pose.position.x = -dz
            pose.orientation.w = 1.0
            aco.object.primitive_poses.append(pose)
        else:
            aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        if not attach:                                            # REMOVE of an attached object only detaches it: it stays in the world where it was
            gone = CollisionObject(id=aco.object.id, operation=CollisionObject.REMOVE)
            gone.header.frame_id = 'map'
            scene.world.collision_objects.append(gone)
        future = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self.arm, future, timeout_sec=10.0)

    def held_distance(self, rec, height, dz):
        """Distance [mm] of the centre of the article from where it is when held (dz behind the TCP along the tool x axis, which is up at
        the grasp), from the simulator, or None."""
        state = entity_state(rec['name'])
        if state is None:
            return None
        origin, q = state
        centre = origin + geo.quat_to_matrix(q) @ np.array([0.0, 0.0, height / 2.0])
        pos, quat = self.arm.forward(self.arm.current())
        r, t = self.gt_map_to_base()
        up_in_tool = r.T @ (geo.quat_to_matrix(quat) @ np.array([1.0, 0.0, 0.0]))     # the tool x axis, in the map: up at the grasp, not in the travel pose
        return round(1000 * float(np.linalg.norm(centre - (self.tcp_in_map() - dz * up_in_tool))), 1)

    def carry(self, rec, cls, dz):
        """The article in the gripper goes to the carrying pose: it is attached to the TCP in the scene, the arm goes to the travel pose
        and the simulator says whether it is still held. Returns a dict with `held` and `outcome` ('carrying' or 'lost_in_carry')."""
        height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
        res = {}
        self.set_scene_articles([], None)                         # nothing of the shelves is of interest any more
        self.attach_article(rec, cls, dz)
        goal, _ = self.arm.execute(self.stow, self.carry_scale)
        self.spin(1.0)
        res['code'] = goal.result.error_code.val
        res['article_to_tcp_mm'] = self.held_distance(rec, height, dz)
        res['rel'] = self.relative_state(rec['name'], height, (1.0, 0.0, 0.0))          # where it is in the pads: depth (towards the table), across, up, tilt
        res['gripper'] = self.state()
        res['held'] = not (res['article_to_tcp_mm'] is None or res['article_to_tcp_mm'] > 1000 * HELD_CARRY or res['gripper'] != 'GRASP')   # it slipped out while the arm moved
        res['outcome'] = 'carrying' if res['held'] else 'lost_in_carry'
        return res

    def bin_station(self, bin_name):
        """(x, y, yaw) in front of a bin of the packing table, the platform PLACE_GAP from the table: the arm reaches the bin straight ahead."""
        b = self.sem['bins'][bin_name]
        tx, _, tyaw = sg.table_station(self.sem, b['table'], PLACE_GAP)
        return tx, b['pose'][1], tyaw

    def place(self, rec, cls, dz, bin_name):
        """At the station of the bin, with the article carried: the article over the bin (straight line at the height where it clears the
        rim), the gripper opens, the arm goes back. Returns a dict; `outcome` is 'delivered' if the simulator has the article inside the bin."""
        height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
        b = self.sem['bins'][bin_name]
        res = {'bin': bin_name}
        res['article_to_tcp_mm_arrived'] = self.held_distance(rec, height, dz)
        res['rel_arrived'] = self.relative_state(rec['name'], height, (1.0, 0.0, 0.0))
        if res['article_to_tcp_mm_arrived'] is None or res['article_to_tcp_mm_arrived'] > 1000 * HELD_ARRIVED:     # it slipped out in the drive
            res['outcome'] = 'lost_in_drive'
            return res
        self.sync_map_odom()
        if self.world_base is not None:                           # the shift found at the shelf does not hold here
            self.shift = np.zeros(2)
            self.shift_world(self.shift)
        normal = (-1.0, 0.0)
        approach_map, rot_map = grasp_axes('front', normal, 0.0, 'x')
        r, t = self.map_to_base()
        quat = matrix_to_quat((r @ np.array(rot_map)).tolist())
        res['tries'] = []
        for dx in PLACE_DXS:
            centre_map = np.array([b['pose'][0] + dx, b['pose'][1], BIN_RIM + PLACE_MARGIN + height / 2.0 + dz])
            centre = r @ centre_map + t
            candidates = look_candidates(self.arm, tuple(centre_map), normal, PLACE_BACK, extra_seeds=PLACE_SEEDS)
            ranked, report = self.choose_pregrasp(candidates, centre, quat)
            res['tries'].append({'drop_dx': dx, 'ik': len(candidates), 'complete': len(ranked)})
            if ranked:
                res['candidates'] = report
                break
        if not ranked:
            res['outcome'] = 'no_place_pose'
            return res
        codes = []
        for joints in ranked[:3]:
            for _ in range(3):
                goal, _ = self.arm.execute(joint_constraints(joints), self.carry_scale)
                codes.append(goal.result.error_code.val)
                if codes[-1] == 1:
                    break
            if codes[-1] == 1:
                break
        res['preplace'] = codes
        if codes[-1] != 1:
            res['outcome'] = 'preplace_failed'
            return res
        ok, res['drop_line'] = self.line_to(centre, quat, self.carry_scale)
        if not ok:
            res['outcome'] = 'drop_line_failed'
            return res
        res['robot_gt'] = np.round(robot_gt(), 4).tolist()
        self.command(close=False)
        self.wait_state(('OPEN',), 4.0)
        self.attach_article(rec, cls, dz, attach=False)
        self.spin(2.0)                                            # it falls and settles
        after = entity_state(rec['name'])
        ok_back, res['back_line'] = self.line_to(centre - PLACE_BACK * (r @ np.array(approach_map)), quat, self.carry_scale)
        if after is None:
            res['outcome'] = 'article_lost'
            return res
        x, y, z = after[0]
        res['article'] = [round(float(x), 3), round(float(y), 3), round(float(z), 3)]
        inside = abs(x - b['pose'][0]) < 0.15 and abs(y - b['pose'][1]) < 0.15 and 0.74 < z < BIN_RIM
        res['outcome'] = 'delivered' if inside else 'missed'
        return res

    def carry_laps(self, rec, cls, dz, bin_name, laps):
        """Carry the article between the station of its slot and the station of the bin `laps` times (2 drives each) and note, after every
        drive, how far it is from where it is held and what the gripper says: does it slip, or does it fall at once? A test of the drive."""
        height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
        out = {'carry': self.carry(rec, cls, dz), 'legs': []}
        if not out['carry']['held']:
            out['outcome'] = 'lost_in_carry'
            return out
        stops = ((bin_name, self.bin_station(bin_name)), (rec['slot'], sg.resolve(self.sem, None, rec['slot'])[1]))
        for lap in range(laps):
            for name, station in stops:
                row = self.navigate(name, *station)
                self.spin(1.0)
                leg = {'to': name, 'result': row['result'].name, 'time_s': round(row['time'], 1), 'path_m': round(row['length'], 2),
                       'article_to_tcp_mm': self.held_distance(rec, height, dz), 'gripper': self.state(),
                       'width_mm': round(1000 * self.gripper[6], 1) if self.gripper else None,
                       'force_N': [round(self.gripper[4], 1), round(self.gripper[5], 1)] if self.gripper else None,
                       'rel': self.relative_state(rec['name'], height, (1.0, 0.0, 0.0))}
                out['legs'].append(leg)
                print(f"   leg {len(out['legs'])} to {name}: {leg['result']}, article {leg['article_to_tcp_mm']} mm from where it is held, gripper {leg['gripper']}, width {leg['width_mm']}", flush=True)
                if leg['article_to_tcp_mm'] is None or leg['article_to_tcp_mm'] > 1000 * HELD_ARRIVED or leg['gripper'] != 'GRASP':
                    out['outcome'] = 'lost'
                    return out
        out['outcome'] = 'kept'
        return out

    def deliver(self, rec, cls, dz, bin_name):
        """carry, Nav2 to the bin, place: a dict with what each step gave and `outcome`."""
        res = {'bin': bin_name, 'carry': self.carry(rec, cls, dz)}
        if not res['carry']['held']:
            res['outcome'] = 'lost_in_carry'
            return res
        row = self.navigate(f'{bin_name} station', *self.bin_station(bin_name))
        res['drive'] = {'result': row['result'].name, 'time_s': round(row['time'], 1), 'path_m': round(row['length'], 2),
                        'error_mm': [round(1000 * row['along'], 1), round(1000 * row['across'], 1)], 'yaw_deg': round(row['err_yaw'], 1)}
        if row['result'].name != 'SUCCEEDED':
            res['outcome'] = 'drive_failed'
            return res
        res['localization'] = self.refine_localization(5)
        res.update(self.place(rec, cls, dz, bin_name))
        res['carry']['article_to_tcp_mm_arrived'] = res.pop('article_to_tcp_mm_arrived')
        res['carry']['rel_arrived'] = res.pop('rel_arrived')
        return res

    def grasp_frame(self, rec, slot):
        """(approach direction, rotation matrix) of the tool in the map. The approach is straight into the shelf; with --align-yaw
        a box is approached turned by its yaw, so that the pads close on two of its faces and not on two corners."""
        normal = sg.NORMALS[slot['face']]
        if self.opt['align_yaw'] and self.by_name[rec['class']]['shape'] == 'box':
            normal = grasp_geometry.rotate_z(normal + (0.0,), rec['pose_map'][3] + math.radians(self.opt['yaw_error_deg']))[:2]
        return grasp_axes('front', normal, 0.0, 'x')

    def see(self, rec, cls, slot, height, records, out, scene=None):
        """Look at the slot from a look pose and estimate the pose of the article from the images (see vision_target.py).

        The look pose is at the nominal pose of the slot, as the arm would do for a target of the semantic map. The next
        distance is tried when the estimate is not reliable. Returns the estimate (world x, y) or None."""
        nominal = np.array([slot['pose'][0], slot['pose'][1], slot['pose'][2] + height / 2.0])
        tries = []
        # With --best-pregrasp the look pose is chosen near the branch of the pre-grasp of the nominal slot: the arm goes from
        # the look pose to the pre-grasp, and where only a pre-grasp with the shoulder turned over has a short line a look pose
        # with the shoulder the other way round leaves a swing of 12 rad that the planner cannot plan.
        branch = self.branch_pregrasp(rec, cls, slot, nominal, records if scene is None else scene) if self.opt['best_pregrasp'] else None
        out['branch'] = None if branch is None else [round(float(v), 3) for v in branch]
        for dist in LOOK_DISTS:
            cands = look_candidates(self.arm, tuple(nominal), sg.NORMALS[slot['face']], dist, extra_seeds=GOOD_SEEDS if branch else ())
            if not cands:
                tries.append({'dist': dist, 'failed': 'no collision-free IK'})
                continue
            natural = [q for q in cands if abs(q[0]) <= PAN_LIMIT] or cands
            poses = [min(natural, key=lambda q: max(abs(v) for v in q))]
            if branch is not None:
                near = min(cands, key=lambda q: float(np.abs(np.array(q) - np.array(branch)).sum()))
                if max(abs(a - b) for a, b in zip(near, poses[0])) > 0.02:
                    poses.insert(0, near)                         # the one near the branch first, the comfortable one if it fails
            codes = []
            for joints in poses:
                for _ in range(3):
                    res, _ = self.arm.execute(joint_constraints(joints), 0.5)
                    codes.append(res.result.error_code.val)
                    if codes[-1] == 1:
                        break
                if codes[-1] == 1:
                    break
            if codes[-1] != 1:
                tries.append({'dist': dist, 'failed': f'the look pose was not reached {codes}'})
                continue
            self.sync_map_odom()                                  # the arm is at rest: the pose of the robot is read now
            if self.opt['localize']:
                estimate = self.localized_estimate(rec, cls, slot, nominal, records)
            else:
                estimate = self.vision.estimate(cls, nominal, self.believed_pose() if self.opt['nav'] else robot_gt())
            estimate['look_dist'] = dist
            tries.append(estimate)
            if 'failed' not in estimate:
                break
        out['vision'] = {'tries': tries, 'normal': list(sg.NORMALS[slot['face']])}
        good = [t for t in tries if 'failed' not in t]
        if not good:
            return None
        true = rec['pose_map']
        err = np.array([good[-1]['x'] - true[0], good[-1]['y'] - true[1]])
        if self.opt['nav']:       # the estimate is in the believed map, the truth in the real one: compare them in the frame of the robot
            bx, by, _, byaw = self.believed_pose()
            gx, gy, _, gyaw = robot_gt()
            est_robot = rot_z(-byaw)[:2, :2] @ (np.array([good[-1]['x'] - bx, good[-1]['y'] - by]))
            true_robot = rot_z(-gyaw)[:2, :2] @ (np.array([true[0] - gx, true[1] - gy]))
            err = rot_z(gyaw)[:2, :2] @ (est_robot - true_robot)       # back on the axes of the map
        out['vision']['error_mm'] = [round(1000 * float(err[0]), 1), round(1000 * float(err[1]), 1)]
        return good[-1]

    def branch_pregrasp(self, rec, cls, slot, nominal, records):
        """The joints of the pre-grasp of the nominal slot that the picking will most likely use, or None: among the solutions
        whose straight line is complete and moves the joints at most SHORT_LINE rad more than the best one, the one with the
        base joint within +-PAN_LIMIT (the best line if there is none) and the shortest line."""
        normal = sg.NORMALS[slot['face']]
        approach_map, rot_map = self.grasp_frame(rec, slot)
        r, t = self.map_to_base() if self.opt['nav'] else self.gt_map_to_base()
        quat = matrix_to_quat((r @ np.array(rot_map)).tolist())
        dz = self.opt['dz'] if self.opt['dz'] is not None else cls.get('grasp_dz', 0.0)
        centre = r @ nominal + t + dz * (r @ np.array([0.0, 0.0, 1.0])) + self.opt['dd'] * (r @ np.array(approach_map))
        candidates = look_candidates(self.arm, tuple(nominal), normal, APPROACH, extra_seeds=GOOD_SEEDS)
        if not candidates:
            return None
        self.set_scene_articles(records, rec['name'])             # the line is judged as it will be run: the article is not an obstacle
        swing, self.opt['swing_aware'] = self.opt['swing_aware'], False
        try:
            _, report = self.choose_pregrasp(candidates, centre, quat)
        finally:
            self.opt['swing_aware'] = swing
            self.set_scene_articles(records, None)
        lines = [c for c in report if c['travel'] is not None]
        if not lines:
            return None
        eligible = [c for c in lines if c['travel'] <= min(c['travel'] for c in lines) + SHORT_LINE]
        natural = [c for c in eligible if abs(c['joints'][0]) <= PAN_LIMIT]
        return min(natural or eligible, key=lambda c: c['travel'])['joints']

    def localized_estimate(self, rec, cls, slot, nominal, records):
        """The pose of the article to pick when the believed map may be off along the shelf (see localization.py).

        All the articles in the image are detected; the inventory (the class in each slot of this face and level, here taken
        from the stock of the simulator) says where each of them should be; the shift of the believed map that puts the
        most of them on the slots of their class is found, and with it the article of the slot to pick. The shift along the
        face is applied to the static objects of the planning scene and to the neighbours (the LiDAR fixes the direction
        across the face, so that part is not touched). Returns the estimate like Vision.estimate, or {'failed': reason}."""
        base = rec['slot'].rsplit('.', 1)[0]
        inventory = [{'slot': r['slot'], 'label': self.by_name[r['class']]['label'],
                      'x': self.sem['slots'][r['slot']]['pose'][0], 'y': self.sem['slots'][r['slot']]['pose'][1]}
                     for r in records if r['slot'].rsplit('.', 1)[0] == base]
        by_label = {c['label']: c for c in self.by_name.values()}
        detections, info = self.vision.detect(by_label, self.believed_pose())
        out = {'detections': len(detections), 'info': info}
        if 'failed' in info:
            return dict(out, failed=info['failed'])
        normal = sg.NORMALS[slot['face']]
        grid = [{'slot': n, 'x': s['pose'][0], 'y': s['pose'][1]} for n, s in self.sem['slots'].items()
                if (s['shelf'], s['level'], s['face']) == (slot['shelf'], slot['level'], slot['face'])]
        if self.opt['inventory_images']:                          # the class of the other slots is read from the image, not taken from the stock
            found = localize_images(detections, grid, rec['slot'], self.by_name[rec.get('ordered_class', rec['class'])]['label'], normal)      # the class that was asked for, not the true one
            if 'inventory' in found:
                labels = {c['label']: c['name'] for c in self.by_name.values()}
                stock = {r['slot']: r['class'] for r in records}
                out['inventory'] = {n: labels[v] for n, v in found['inventory'].items()}
                out['inventory_wrong'] = sorted(n for n, c in out['inventory'].items() if stock.get(n) != c)
        else:
            found = localize(detections, inventory, rec['slot'], normal)
        if 'failed' in found:
            return dict(out, failed=found['failed'])
        out.update(score=found['score'], ambiguous=found['ambiguous'], shift_mm=[round(1000 * v, 1) for v in found['shift']])
        if found['ambiguous']:
            return dict(out, failed='two shifts explain as many articles with another article for the slot')
        det = detections[found['target']]
        self.last_seen = {'detections': detections, 'target': found['target'], 'shift': np.array(found['shift'], dtype=float), 'grid': grid}
        tangent = np.array([-normal[1], normal[0]])
        self.shift = float(np.dot(found['shift'], tangent)) * tangent
        out['shift_applied_mm'] = [round(1000 * float(v), 1) for v in self.shift]
        out['scene_objects_moved'] = self.shift_world(self.shift)
        if self.opt['nav']:                                       # what the shift should have been: the believed map against the real one
            bx, by, _, byaw = self.believed_pose()
            gx, gy, _, gyaw = robot_gt()
            seen = rot_z(-gyaw)[:2, :2] @ (nominal[:2] - np.array([gx, gy]))
            out['shift_true_mm'] = [round(1000 * float(v), 1) for v in (np.array([bx, by]) + rot_z(byaw)[:2, :2] @ seen - nominal[:2])]
        out.update(x=det['x'], y=det['y'], yaw=det['yaw'], points=det['points'], mask_pixels=det['mask_pixels'],
                   fraction=det['fraction'], prior_distance=round(float(np.hypot(det['x'] - nominal[0] - found['shift'][0],
                                                                                det['y'] - nominal[1] - found['shift'][1])), 3))
        return out

    def make_dynamic(self, rec):
        """The articles of the stock are static: they cannot be pushed or lifted, and a grasp of one pulls the robot instead.
        The one to pick is created again, where it stands, as a dynamic body."""
        name = rec['name']
        self.stock.remove_present(Stock.PREFIX, only={name})
        x, y, z, yaw = rec['pose_map']
        for _ in range(4):
            stocker.create(name, rec['class'], x, y, z + 0.002, yaw, static=False)
            time.sleep(0.5)
            if name in stocker.models(Stock.PREFIX):
                break
        else:
            raise RuntimeError(f'{name} could not be created again')
        self.spin(1.5)                                            # it settles on the board

    def place_alone(self, slots, centre_x, max_offset, only=None):
        """Clear the shelves and put one article of a random class in a random slot of the face (`only`: in that slot): its record."""
        self.stock.clear_current()
        usable = [n for n, s in slots.items() if abs(s['pose'][0] - centre_x) <= max_offset
                  and (not self.opt['levels'] or int(n.split('.')[1][1:]) in self.opt['levels'])]
        name = only or self.rng.choice(sorted(usable))
        slot = slots[name]
        pool = [c for c in self.by_name.values() if not self.opt['classes'] or c['name'] in self.opt['classes']]
        cls = self.rng.choice(pool)
        self.stock.count += 1
        article = f"{Stock.PREFIX}{self.stock.count}_{name.replace('.', '_')}"
        yaw = math.radians(self.opt['yaw_deg'] or 0.0)
        self.stock.run([(stocker.create, article, cls['name'], slot['pose'][0], slot['pose'][1], slot['pose'][2], yaw)])
        rec = {'name': article, 'class': cls['name'], 'label': cls['label'], 'slot': name,
               'pose_map': [slot['pose'][0], slot['pose'][1], slot['pose'][2], yaw]}
        for _ in range(3):
            time.sleep(0.5)
            if rec['name'] in stocker.models(Stock.PREFIX):
                break
            self.stock.run([(stocker.create, article, cls['name'], slot['pose'][0], slot['pose'][1], slot['pose'][2], yaw)])
        self.stock.verify([rec])
        return rec

    def release_and_stow(self):
        self.command(close=False)
        self.wait_state(('OPEN',), 4.0)
        self.arm.execute(self.stow, 1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--unit', default='S1')
    ap.add_argument('--trials', type=int, default=6)
    ap.add_argument('--max-offset', type=float, default=0.8, help='only articles this close to the centre of the unit along the row [m]')
    ap.add_argument('--slot', help='pick the article of this slot (S1.L3.n5), from the station in front of it (semantic_goals.slot_station) instead of the station of the unit; the stock is made again until the slot has an article (of --classes, if given)')
    ap.add_argument('--scene-neighbours', action=argparse.BooleanOptionalAction, default=True, help='the articles of the shelves are in the planning scene of MoveIt (--no-scene-neighbours: they are on the shelves of the simulator but not in the scene, to tell the physical effect of the neighbours from the effect on the plans)')
    ap.add_argument('--yaw-deg', type=float, default=None, help='with --alone: the article stands turned by this angle about the vertical [degrees] (default 0)')
    ap.add_argument('--align-yaw', action='store_true', help='a box is approached turned by its true yaw, so that the pads close on two faces of it (the yaw is the one of the stock, true)')
    ap.add_argument('--yaw-error-deg', type=float, default=0.0, help='with --align-yaw: the approach is turned by the yaw of the box plus this error [degrees] (an estimate of the yaw that is off)')
    ap.add_argument('--scene-images', action='store_true', help='with --localize: the neighbours in the planning scene are the articles that the image shows (scene_from_images), not the stock of the simulator; before the first look only the article to pick is in the scene')
    ap.add_argument('--inventory-images', action='store_true', help='with --localize: the class in each slot is read from the image (localization.localize_images) and not taken from the stock; only the class of the article to pick is given')
    ap.add_argument('--deliver', help='with --nav and a held article: carry it to this bin (bin_1, bin_2; several separated by commas: one is drawn) of the packing table and drop it there')
    ap.add_argument('--carry-laps', type=int, default=0, help='with --deliver: instead of the delivery, drive this many times between the station of the slot and the one of the bin with the article held, noting after each drive whether it is still held')
    ap.add_argument('--alone', action='store_true', help='one article on the shelves, no neighbours')
    ap.add_argument('--classes', nargs='+', help='only these classes are picked (with --alone also the only ones on the shelves; otherwise the neighbours are of any class)')
    ap.add_argument('--levels', nargs='+', type=int, help='only articles at these shelf levels (1, 2, 3) are picked')
    ap.add_argument('--hold', type=float, default=0.0, help='seconds to wait after the retreat, with the article in the gripper, before it is released (to look at it in the GUI)')
    ap.add_argument('--force', type=float, default=0.0, help='grip force [N], 0 = the 25 N of the controller')
    ap.add_argument('--grasp-dz', type=float, default=None, help='the grasp point this much above the centre of the article [m] (default: grasp_dz of the class in articles.yaml)')
    ap.add_argument('--grasp-dd', type=float, default=0.0, help='the grasp point this much deeper along the approach [m]')
    ap.add_argument('--retreat-up', type=float, default=RETREAT_UP, help='the retreat goes this much up [m]')
    ap.add_argument('--retreat-scale', type=float, default=0.3, help='speed scale of the retreat')
    ap.add_argument('--lift-first', type=float, default=0.0, help='lift straight up this much before the retreat [m]')
    ap.add_argument('--best-pregrasp', action='store_true', help='the pre-grasp is the IK solution with the best straight approach (least joint travel)')
    ap.add_argument('--nav', action='store_true', help='the robot has arrived by Nav2 and is not put back: AMCL\'s transforms are used, the simulator only judges (needs --vision)')
    ap.add_argument('--trace-tilt', action='store_true', help='record the poses of the world from the closing to the end of the retreat in /tmp/trace_*.txt (the tilt of the article in time)')
    ap.add_argument('--swing-aware', action=argparse.BooleanOptionalAction, default=True, help='with --best-pregrasp (default on): among the pre-grasps with a short straight line, the one nearest to the joints of the arm now; --no-swing-aware ranks by the line only')
    ap.add_argument('--localize', action='store_true', help='with --nav: find the shift of the believed map along the shelf from all the articles in the image and the inventory, and move the scene by it')
    ap.add_argument('--vision', action='store_true', help='the pose of the article from the images (needs the stereo node and the cameras)')
    ap.add_argument('--amcl-updates', type=int, default=5, help='with --nav: updates of AMCL without motion requested at the station')
    ap.add_argument('--prior-radius', type=float, default=None, help='with --vision: an estimate farther than this from the nominal slot is rejected [m] (default 0.03, with --nav 0.06: the localization error of AMCL adds to the 1.4 cm of the stock)')
    ap.add_argument('--network', default='base', help='with --vision: the network segmentation/runs/<name>.pt')
    ap.add_argument('--out', required=True, help='JSON file with every attempt')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()
    if args.deliver and not args.nav:
        ap.error('--deliver needs --nav: the robot drives to the packing table')
    if args.scene_images and not args.localize:
        ap.error('--scene-images needs --localize: the articles come from the detections')
    if args.localize and not args.nav:
        ap.error('--localize needs --nav: with the pose of the simulator there is no shift to find')
    if args.nav and not args.vision:
        ap.error('--nav needs --vision: the true pose of the article is in the map, and the robot only knows where it is within the error of AMCL')

    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    rng = random.Random(args.seed)
    rclpy.init()
    picker = Picker(sem, classes, rng)
    picker.opt.update(force=args.force, dz=args.grasp_dz, dd=args.grasp_dd, retreat_up=args.retreat_up,
                      retreat_scale=args.retreat_scale, classes=args.classes, levels=args.levels, lift_first=args.lift_first,
                      best_pregrasp=args.best_pregrasp, nav=args.nav, localize=args.localize, swing_aware=args.swing_aware, trace_tilt=args.trace_tilt, scene_neighbours=args.scene_neighbours, align_yaw=args.align_yaw, yaw_deg=args.yaw_deg, yaw_error_deg=args.yaw_error_deg, inventory_images=args.inventory_images, scene_images=args.scene_images)
    if args.vision:
        from vision_target import PRIOR_RADIUS, Vision        # heavy imports (torch, the network): only when asked
        picker.vision = Vision(args.network, args.prior_radius if args.prior_radius is not None else (NAV_PRIOR_RADIUS if args.nav else PRIOR_RADIUS))
    picker.stock.clear_original()
    face_slots = {n: s for n, s in sem['slots'].items() if s['shelf'] == args.unit and s['pickable']}
    if args.slot and args.slot not in face_slots:
        ap.error(f'--slot {args.slot} is not a pickable slot of {args.unit}')
    _, station = sg.resolve(sem, None, args.slot or args.unit)
    centre_x = float(np.mean([s['pose'][0] for s in face_slots.values()]))
    results, failures = [], 0
    try:
        refined = {}
        if args.nav:
            refined = picker.refine_localization(args.amcl_updates)
            print('localization error before / after %d updates: %.1f mm, %.2f deg / %.1f mm, %.2f deg' % (
                refined['updates'], refined['before_mm'], refined['before_yaw_deg'], refined['after_mm'], refined['after_yaw_deg']), flush=True)
            arrival = picker.offset_from(station)
            print('arrived by Nav2: %.3f m and %.1f degrees from the station' % (arrival[0], math.degrees(arrival[1])), flush=True)
        else:
            print('base at the start: %.3f m and %.1f degrees from the station' % (
                lambda d: (d[0], math.degrees(d[1])))(picker.reset_base(station)), flush=True)
        picker.sync_map_odom()
        picker.arm.execute(picker.stow, 1.0)
        for trial in range(args.trials):
            if args.alone:
                rec = picker.place_alone(face_slots, centre_x, args.max_offset, args.slot)
                records = [rec]
            else:
                for _ in range(40):                               # fill again until there is an article of the wanted class and level
                    records, _ = picker.stock.refill(args.unit, centre_x, 0.9)
                    usable = [r for r in records if r['slot'] in face_slots and abs(face_slots[r['slot']]['pose'][0] - centre_x) <= args.max_offset
                              and (not args.slot or r['slot'] == args.slot)
                              and (not args.classes or r['class'] in args.classes)
                              and (not args.levels or int(r['slot'].split('.')[1][1:]) in args.levels)]
                    if usable:
                        break
                if not usable:
                    continue
                rec = rng.choice(usable)
            drift = picker.offset_from(station) if args.nav else picker.reset_base(station)     # back to the station if the base moved
            out = picker.pick(rec, face_slots[rec['slot']], records)
            out['base_moved'] = [round(1000 * drift[0], 1), round(math.degrees(drift[1]), 1)]
            if args.nav:                                          # where it arrived, and what the transforms thought of it
                belief = picker.believed_pose()
                gx, gy, gz, gyaw = robot_gt()
                out['arrival'] = {'refinement': refined, 'offset_mm': out['base_moved'][0], 'yaw_deg': out['base_moved'][1],
                                  'localization_mm': round(1000 * math.hypot(belief[0] - gx, belief[1] - gy), 1),
                                  'localization_yaw_deg': round(math.degrees(abs((belief[3] - gyaw + math.pi) % (2 * math.pi) - math.pi)), 2)}
            out['trial'] = trial
            results.append(out)
            print(f'{trial}: {out["class"]:13s} {out["slot"]:11s} -> {out["outcome"]}  '
                  + ' '.join(f'{k}={out[k]}' for k in ('grip_state', 'width_mm', 'article_to_tcp_mm', 'tilt_deg', 'lifted_mm', 'neighbours_moved') if k in out),
                  flush=True)
            json.dump(results, open(args.out, 'w'), indent=1)
            if args.deliver and args.carry_laps and out['outcome'] == 'success':
                out['laps'] = picker.carry_laps(rec, picker.by_name[rec['class']], out['grasp_dz'], args.deliver.split(',')[0], args.carry_laps)
                print(f'   laps: {out["laps"]["outcome"]}', flush=True)
                json.dump(results, open(args.out, 'w'), indent=1)
            elif args.deliver and out['outcome'] == 'success':
                out['deliver'] = picker.deliver(rec, picker.by_name[rec['class']], out['grasp_dz'], picker.rng.choice(args.deliver.split(',')))
                print(f'   delivery: {out["deliver"]["outcome"]}', flush=True)
                json.dump(results, open(args.out, 'w'), indent=1)
            picker.spin(args.hold)
            failures = 0 if out['outcome'] == 'success' else failures + 1
            if failures >= 8:
                print('eight failures in a row: the arm or the world is in a state that does not recover, stopping', flush=True)
                break
            picker.release_and_stow()
    finally:
        try:
            picker.stock.clear_current()
        except Exception as err:
            print('the articles could not be removed:', err)
        json.dump(results, open(args.out, 'w'), indent=1)
        if picker.vision is not None:
            picker.vision.capture.destroy_node()
        picker.arm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    won = sum(r['outcome'] == 'success' for r in results)
    print(f'== {won} of {len(results)} picked')


if __name__ == '__main__':
    main()
