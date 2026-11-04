#!/usr/bin/env python3
"""Check MoveIt on the AMR manipulator: IK round trip, joint goal, pose goal (plan and execute)."""
import argparse
import math
import os
import random
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Transform
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PositionConstraint)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

import semantic_goals as sg
from grasp_geometry import article_table, grasp_axes, matrix_to_quat

JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
LINK = 'gripper_tcp'
FRAME = 'base_footprint'      # poses are given in the frame of the robot, whatever the world frame is


def named_states():
    """group_state values of the arm in the SRDF: name -> joint values in the order of JOINTS."""
    path = os.path.join(get_package_share_directory('amr_manipulator_moveit_config'), 'config', 'amr_manipulator.srdf')
    states = {}
    for gs in ET.parse(path).getroot().findall('group_state'):
        if gs.get('group') == 'arm':
            vals = {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
            states[gs.get('name')] = [vals[j] for j in JOINTS]
    return states


def angle_between(q1, q2):
    """Angle [rad] of the rotation that takes quaternion q1 to q2 (x, y, z, w tuples)."""
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


class Checker(Node):
    def __init__(self):
        super().__init__('moveit_check', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.run_trajectory = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.q = {}

    def on_joints(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def current(self, timeout=10.0):
        """Joint values of the arm now, in the order of JOINTS."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and not all(j in self.q for j in JOINTS):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not all(j in self.q for j in JOINTS):
            raise SystemExit('no /joint_states for the arm (is the simulation running?)')
        rclpy.spin_once(self, timeout_sec=0.2)
        return [self.q[j] for j in JOINTS]

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def forward(self, q):
        """(position, quaternion) of the TCP in FRAME for joint values q, from MoveIt's FK."""
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = [LINK]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        res = self.call(self.fk, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise SystemExit('compute_fk failed')
        p = res.pose_stamped[0].pose
        return (p.position.x, p.position.y, p.position.z), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def inverse(self, pos, quat, seed, timeout=0.5, avoid_collisions=False, frame=FRAME):
        """(joint values or None, seconds) of MoveIt's IK for a TCP pose, starting from the seed."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = LINK
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.pose_stamped.header.frame_id = frame
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = list(seed)
        req.ik_request.timeout.sec, req.ik_request.timeout.nanosec = 0, int(timeout * 1e9)
        t0 = time.monotonic()
        res = self.call(self.ik, req)
        dt = time.monotonic() - t0
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, dt
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS], dt

    def check_state(self, q, base, whole):
        """(valid, contacts) of the arm at joint values q with the base at base = (x, y, yaw) in the map frame.

        whole: check every link of the robot against the scene; otherwise only the links of the arm group."""
        req = GetStateValidity.Request()
        req.group_name = '' if whole else 'arm'
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        x, y, yaw = base
        req.robot_state.multi_dof_joint_state.header.frame_id = 'map'
        req.robot_state.multi_dof_joint_state.joint_names = ['world_joint']
        t = Transform()
        t.translation.x, t.translation.y = x, y
        t.rotation.z, t.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req.robot_state.multi_dof_joint_state.transforms = [t]
        res = self.call(self.validity, req)
        if res is None:
            raise SystemExit('check_state_validity did not answer')
        return res.valid, sorted({(c.contact_body_1, c.contact_body_2) for c in res.contacts})

    def straight_line(self, pos, quat, target, scale, max_jump=0.5):
        """Time-parameterized straight TCP path from the current state to target: (response, joints of the points)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = 'arm'
        req.link_name = LINK
        end = Pose()
        end.position.x, end.position.y, end.position.z = target
        end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w = quat
        req.waypoints = [end]
        req.max_step = 0.01                  # [m] between two points of the path
        req.revolute_jump_threshold = max_jump   # [rad] a bigger joint jump between two points = another IK branch
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        res = self.call(self.cartesian, req, timeout=20.0)
        if res is None:
            raise SystemExit('compute_cartesian_path did not answer')
        return res

    def run_path(self, trajectory):
        """Execute a RobotTrajectory through MoveIt's trajectory execution; return the error code."""
        if not self.run_trajectory.wait_for_server(timeout_sec=10.0):
            raise SystemExit('no /execute_trajectory: is move_group running?')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.run_trajectory.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise SystemExit('the ExecuteTrajectory goal was rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120.0)
        return result.result().result.error_code.val

    def execute(self, constraints, scale):
        """Plan and execute a MoveGroup goal; return (result, wall seconds)."""
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = 'arm'
        r.goal_constraints = [constraints]
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = scale
        r.max_acceleration_scaling_factor = scale
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        if not self.move.wait_for_server(timeout_sec=10.0):
            raise SystemExit('no /move_action: is move_group running?')
        t0 = time.monotonic()
        future = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise SystemExit('the MoveGroup goal was rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120.0)
        return result.result(), time.monotonic() - t0


def joint_constraints(q, tol=0.002):
    return Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=v, tolerance_above=tol, tolerance_below=tol, weight=1.0)
        for j, v in zip(JOINTS, q)])


def pose_constraints(pos, quat):
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = LINK
    pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002]))
    target = Pose()
    target.position.x, target.position.y, target.position.z = pos
    target.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(target)
    pc.weight = 1.0
    oc = OrientationConstraint()
    oc.header.frame_id = FRAME
    oc.link_name = LINK
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01
    oc.weight = 1.0
    return Constraints(position_constraints=[pc], orientation_constraints=[oc])


def check_ik(node, n, noise, seed_value):
    """FK of random configurations, then IK from a seed that is `noise` rad away: does the pose come back?"""
    rng = random.Random(seed_value)
    limits = [math.pi] * N_J
    done, worst_p, worst_o, times, jump = 0, 0.0, 0.0, [], []
    for _ in range(n):
        q = [rng.uniform(-l, l) for l in limits]
        pos, quat = node.forward(q)
        seed = [x + rng.uniform(-noise, noise) for x in q]
        sol, dt = node.inverse(pos, quat, seed)
        times.append(dt)
        if sol is None:
            continue
        done += 1
        p2, q2 = node.forward(sol)
        worst_p = max(worst_p, math.dist(pos, p2))
        worst_o = max(worst_o, angle_between(quat, q2))
        jump.append(max(abs(a - b) for a, b in zip(sol, q)))
    print(f'IK from {n} random configurations, seed up to {noise} rad away from the answer:')
    print(f'  solved {done} of {n} ({100.0 * done / n:.0f} %), mean time {1000 * sum(times) / n:.0f} ms')
    if done:
        print(f'  pose of the solution: position error max {1000 * worst_p:.3f} mm, orientation error max {math.degrees(worst_o):.4f} deg')
        print(f'  largest joint difference to the configuration used: median {sorted(jump)[len(jump) // 2]:.3f} rad, max {max(jump):.3f} rad')
    return done == n and worst_p < 1e-3 and worst_o < math.radians(0.1)


N_J = len(JOINTS)


def tcp_report(node, target, quat):
    """Print how far the TCP ended from the target pose, after the motion."""
    time.sleep(0.5)
    pos2, quat2 = node.forward(node.current())
    print(f'TCP after: position error {1000 * math.dist(pos2, target):.1f} mm, '
          f'orientation error {math.degrees(angle_between(quat, quat2)):.2f} deg')


def do_pose(node, args):
    """Move the TCP by (dx, dy, dz): IK seeded with the current joints and a joint goal, or a native pose goal."""
    q0 = node.current()
    pos, quat = node.forward(q0)
    target = (pos[0] + args.dx, pos[1] + args.dy, pos[2] + args.dz)
    print(f'TCP in {FRAME}: from ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) to '
          f'({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}), same orientation, mode {args.mode}')
    if args.mode == 'ik':
        sol, _ = node.inverse(target, quat, q0, avoid_collisions=True)
        if sol is None:
            print('IK: no collision-free solution')
            return False
        jump = max(abs(a - b) for a, b in zip(sol, q0))
        print(f'IK from the current joints: largest joint change {jump:.2f} rad')
        if jump > args.max_jump:
            print(f'refused: that is another IK branch (limit {args.max_jump} rad)')
            return False
        constraints = joint_constraints(sol)
    else:
        constraints = pose_constraints(target, quat)       # MoveIt samples its own IK solution
    res, seconds = node.execute(constraints, args.scale)
    ok = report_result(res, seconds)
    tcp_report(node, target, quat)
    print(f'largest joint change of the whole move: {max(abs(a - b) for a, b in zip(node.current(), q0)):.2f} rad')
    return ok


def do_cartesian(node, args):
    """Move the TCP on a straight line by (dx, dy, dz), keeping the orientation."""
    q0 = node.current()
    pos, quat = node.forward(q0)
    target = (pos[0] + args.dx, pos[1] + args.dy, pos[2] + args.dz)
    print(f'TCP in {FRAME}: from ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) to '
          f'({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}), straight line')
    res = node.straight_line(pos, quat, target, args.scale)
    pts = res.solution.joint_trajectory.points
    print(f'path: {100 * res.fraction:.0f} % of the line, {len(pts)} points, error_code {res.error_code.val}')
    if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < 0.999 or len(pts) < 2:
        return False
    end = pts[-1].time_from_start
    steps = [max(abs(a - b) for a, b in zip(p.positions, r.positions)) for r, p in zip(pts, pts[1:])]
    print(f'duration {end.sec + end.nanosec * 1e-9:.1f} s, velocities and accelerations on the points: '
          f'{"yes" if pts[1].velocities and pts[1].accelerations else "NO"}, '
          f'largest joint change from the start {max(max(abs(a - b) for a, b in zip(p.positions, q0)) for p in pts):.3f} rad, '
          f'largest step between points {max(steps):.3f} rad')
    code = node.run_path(res.solution)
    print(f'ExecuteTrajectory: error_code {code} ({"SUCCESS" if code == MoveItErrorCodes.SUCCESS else "FAILED"})')
    tcp_report(node, target, quat)
    return code == MoveItErrorCodes.SUCCESS


LIMITS = [2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi]   # position limits of the joints in the URDF


def normalised(q):
    """Each joint turned by a multiple of 2 pi to the equivalent angle closest to 0 that its limits allow."""
    out = []
    for v, limit in zip(q, LIMITS):
        for k in (-2, -1, 1, 2):
            w = v + k * 2 * math.pi
            if abs(w) < abs(v) and abs(w) <= limit:
                v = w
        out.append(v)
    return out


def look_solution(node, center, normal, dist, offset=(0.0, 0.0, 0.0), tries=5):
    """Joint values that put the TCP `dist` in front of `center` (map frame), tool horizontal into the shelf whose
    pick face has the outward normal `normal`; `offset` moves the TCP target. None if there is no collision-free one.

    The IK restarts from random states when the seed is far from a solution, so it may return another branch,
    or the same posture with a joint turned by 2 pi. So: seeds near the posture of the observation, a few tries
    each, every solution brought to its equivalent joint angles closest to 0, and the one with the smallest
    largest joint is taken."""
    approach, rot = grasp_axes('front', normal, 0.0, 'x')
    target = tuple(c - dist * a + o for c, a, o in zip(center, approach, offset))
    seeds = [[0.0, -1.2, 1.6, -2.0, -1.5708, 1.5708], [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708]]
    candidates = []
    for seed in seeds:
        for _ in range(tries):
            cand, _ = node.inverse(target, matrix_to_quat(rot), seed, timeout=0.5, avoid_collisions=True, frame='map')
            if cand is not None:
                candidates.append(normalised(cand))
    return min(candidates, key=lambda q: max(abs(v) for v in q)) if candidates else None


def do_look(node, args):
    """Put the camera in front of a slot, tool horizontal into the shelf: IK from the current joints, then a joint goal."""
    sem = sg.load(args.map)
    slot = sem['slots'].get(args.slot)
    if slot is None:
        raise SystemExit(f'unknown slot {args.slot}, for example S3.L2.n3')
    article = article_table(args.articles).get(slot['article'], {'height': 0.17})
    center = (slot['pose'][0], slot['pose'][1], slot['pose'][2] + article['height'] / 2.0)
    print(f"slot {args.slot} ({slot['article']}), centre ({center[0]:+.3f}, {center[1]:+.3f}, {center[2]:+.3f}); the TCP "
          f'{args.dist:.2f} m in front of it, the camera is about {args.dist + 0.08:.2f} m from the centre')
    sol = look_solution(node, center, sg.NORMALS[slot['face']], args.dist)
    if sol is None:
        print('IK: no collision-free solution')
        return False
    print(f'IK: the most comfortable collision-free solution (largest joint {max(abs(v) for v in sol):.2f} rad):',
          [round(v, 3) for v in sol])
    res, seconds = node.execute(joint_constraints(sol), args.scale)
    return report_result(res, seconds)


def report_result(res, seconds):
    code = res.result.error_code.val
    print(f'MoveGroup: error_code {code} ({"SUCCESS" if code == MoveItErrorCodes.SUCCESS else "FAILED"}), '
          f'planning {res.result.planning_time:.2f} s, {seconds:.1f} s in total, '
          f'{len(res.result.planned_trajectory.joint_trajectory.points)} trajectory points')
    return code == MoveItErrorCodes.SUCCESS


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('ik', help='forward then inverse kinematics round trip')
    p.add_argument('--n', type=int, default=50)
    p.add_argument('--noise', type=float, default=0.3, help='distance of the IK seed from the answer [rad]')
    p.add_argument('--seed', type=int, default=1)
    p = sub.add_parser('joint', help='plan and execute a joint goal')
    p.add_argument('--named', help='a group_state of the SRDF (travel, home90)')
    p.add_argument('--goal', type=float, nargs=N_J)
    p.add_argument('--scale', type=float, default=1.0, help='velocity and acceleration scaling')
    p = sub.add_parser('pose', help='plan and execute a pose goal: the current TCP pose moved by dx dy dz')
    p.add_argument('--mode', choices=['ik', 'native'], default='ik',
                   help="ik: IK seeded with the current joints, then a joint goal; native: MoveIt's own pose goal")
    p.add_argument('--max-jump', type=float, default=1.5, help='largest joint change accepted from the IK [rad]')
    p = sub.add_parser('cartesian', help='straight line of the TCP by dx dy dz, orientation kept')
    for name, cmd in sub.choices.items():
        if name in ('pose', 'cartesian'):
            for axis in ('dx', 'dy', 'dz'):
                cmd.add_argument(f'--{axis}', type=float, default=0.0, help='[m] in base_footprint')
            cmd.add_argument('--scale', type=float, default=0.5 if name == 'cartesian' else 1.0,
                             help='velocity and acceleration scaling')
    p = sub.add_parser('valid', help='is the robot collision free in this configuration, with the base at this pose?')
    p.add_argument('--base', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'),
                   help='pose of base_footprint in the map frame [m, m, rad]')
    p.add_argument('--named', help='a group_state of the SRDF (travel, home90)')
    p.add_argument('--goal', type=float, nargs=N_J, help='joint values [rad]')
    p.add_argument('--whole', action='store_true', help='check every link of the robot, not only the arm')
    p = sub.add_parser('look', help='put the camera in front of a slot of the shelves (tool horizontal)')
    p.add_argument('--slot', required=True, help='slot of the semantic map, for example S3.L2.n3')
    p.add_argument('--dist', type=float, default=0.30, help='TCP distance to the centre of the article [m]')
    p.add_argument('--scale', type=float, default=0.5, help='velocity and acceleration scaling')
    p.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    p.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    rclpy.init()
    node = Checker()
    ok = True
    if args.cmd == 'ik':
        if not node.fk.wait_for_service(timeout_sec=10.0) or not node.ik.wait_for_service(timeout_sec=10.0):
            raise SystemExit('no /compute_fk or /compute_ik: is move_group running?')
        ok = check_ik(node, args.n, args.noise, args.seed)
    elif args.cmd == 'joint':
        goal = args.goal or named_states().get(args.named or '')
        if goal is None:
            raise SystemExit(f'give --goal or --named, one of {sorted(named_states())}')
        before = node.current()
        print('from', [round(x, 3) for x in before], '\nto  ', [round(x, 3) for x in goal])
        res, seconds = node.execute(joint_constraints(goal), args.scale)
        ok = report_result(res, seconds)
        time.sleep(0.5)
        err = [abs(a - b) for a, b in zip(node.current(), goal)]
        print('final error per joint [rad]:', ' '.join(f'{e:.4f}' for e in err), f'  max {max(err):.4f}')
    elif args.cmd == 'valid':
        q = args.goal or named_states().get(args.named or '')
        if q is None:
            raise SystemExit(f'give --goal or --named, one of {sorted(named_states())}')
        if not node.validity.wait_for_service(timeout_sec=10.0):
            raise SystemExit('no /check_state_validity: is move_group running?')
        ok, contacts = node.check_state(q, args.base, args.whole)
        x, y, yaw = args.base
        print(f'base ({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.0f} deg), arm {[round(v, 3) for v in q]}, '
              f'{"whole robot" if args.whole else "arm only"}: {"valid" if ok else "IN COLLISION"}')
        for a, b in contacts:
            print(f'  contact: {a} - {b}')
    elif args.cmd == 'look':
        ok = do_look(node, args)
    elif args.cmd == 'pose':
        ok = do_pose(node, args)
    else:
        ok = do_cartesian(node, args)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
