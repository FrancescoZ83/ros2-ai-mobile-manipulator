#!/usr/bin/env python3
"""Send a multi-point trajectory to the FollowJointTrajectory server and print the tracking error."""
import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

N = 6
JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
NAMES = ['pan', 'lift', 'elbow', 'wrist1', 'wrist2', 'wrist3']
SETTLE = 1.0   # [s] observed after the end of the trajectory


def duration(t):
    """float seconds -> builtin_interfaces/Duration."""
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def point(t, q, v, a):
    """JointTrajectoryPoint at time t."""
    return JointTrajectoryPoint(positions=list(q), velocities=list(v), accelerations=list(a),
                                time_from_start=duration(t))


def quintic(a, b, total, dt):
    """Rest-to-rest quintic from a to b, sampled every dt (the move arm_goto makes)."""
    pts, steps = [], max(int(round(total / dt)), 1)
    for k in range(steps + 1):
        t = total * k / steps
        u = t / total
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = (30 * u**2 - 60 * u**3 + 30 * u**4) / total
        dds = (60 * u - 180 * u**2 + 120 * u**3) / total**2
        pts.append(point(t, [x + s * (y - x) for x, y in zip(a, b)],
                         [ds * (y - x) for x, y in zip(a, b)], [dds * (y - x) for x, y in zip(a, b)]))
    return pts


def via(a, b, total):
    """Three points only: rest, the middle at 1.5 times the average speed, rest."""
    mid = [(x + y) / 2 for x, y in zip(a, b)]
    v_mid = [1.5 * (y - x) / total for x, y in zip(a, b)]
    zero = [0.0] * N
    return [point(0.0, a, zero, zero), point(total / 2, mid, v_mid, zero), point(total, b, zero, zero)]


class Tester(Node):
    def __init__(self):
        super().__init__('traj_test')
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_model_controller/follow_joint_trajectory')
        self.create_subscription(Float64MultiArray, '/arm_model_controller/state', self.on_state, 10)
        self.state = None
        self.total = None            # duration of the trajectory being measured
        self.moving, self.settling = [], []
        self.finished_at = None      # controller time t when the trajectory ended

    def on_state(self, msg):
        d = msg.data
        if len(d) != 2 + 5 * N:
            return
        t, big_t = d[0], d[1]
        self.state = d
        err = [d[2 + i] - d[2 + N + i] for i in range(N)]       # q_d - q
        tau = d[2 + 4 * N:2 + 5 * N]
        if self.total is None or abs(big_t - self.total) > 1e-4:
            return
        if t < big_t:
            self.moving.append((err, tau))
        elif t <= big_t + SETTLE:
            self.settling.append((err, tau))

    def report(self):
        print('joint     rms[rad]  max[rad]  mean[rad]  final[rad]  max|tau|[Nm]')
        for i in range(N):
            e = [s[0][i] for s in self.moving]
            rms = math.sqrt(sum(x * x for x in e) / len(e))
            final = sum(s[0][i] for s in self.settling[len(self.settling) // 2:]) / max(len(self.settling) // 2, 1)
            print(f'{NAMES[i]:8s} {rms:9.4f} {max(abs(x) for x in e):9.4f} {sum(e) / len(e):+10.4f} '
                  f'{final:+11.4f} {max(abs(s[1][i]) for s in self.moving):12.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', type=float, nargs=N, required=True, help='target joint configuration [rad]')
    ap.add_argument('--time', type=float, default=4.0, help='duration [s]')
    ap.add_argument('--shape', choices=['quintic', 'via'], default='quintic')
    ap.add_argument('--dt', type=float, default=0.1, help='sampling of the quintic shape [s]')
    args = ap.parse_args()

    rclpy.init()
    node = Tester()
    start = time.time()
    while rclpy.ok() and node.state is None and time.time() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.state is None or not node.client.wait_for_server(timeout_sec=10.0):
        raise SystemExit('no controller state or no action server (is the simulation running?)')
    if node.state[0] < node.state[1]:
        raise SystemExit('the arm is still moving: wait until it is idle')

    a = list(node.state[2:2 + N])          # where the reference is: the goal of the last move
    points = quintic(a, args.goal, args.time, args.dt) if args.shape == 'quintic' else via(a, args.goal, args.time)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory(joint_names=JOINTS, points=points)
    print(f'{args.shape}: {len(points)} points, {args.time:.1f} s')

    node.total = float(points[-1].time_from_start.sec) + points[-1].time_from_start.nanosec * 1e-9
    future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise SystemExit('the goal was rejected')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=3 * args.time + 20.0)
    end = time.time() + SETTLE + 0.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    res = result_future.result()
    if res is None:
        raise SystemExit('no result from the action server')
    print(f'result: error_code {res.result.error_code} {res.result.error_string}'.rstrip())
    if node.moving and node.settling:
        node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
