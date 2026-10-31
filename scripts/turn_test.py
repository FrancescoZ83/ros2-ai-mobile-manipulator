#!/usr/bin/env python3
"""Turn in place in 90 degree steps and compare the odometry yaw with the Gazebo ground truth."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from drive_test import get_pose, profile      # (x, y, yaw) from Gazebo; trapezoidal speed profile

WHEEL_SEPARATION = 0.58                       # [m] value in gazebo_plugins.xacro


def wrap(a):
    """Angle in [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Turn(Node):
    def __init__(self):
        # Simulated time: the profile must match the physics, whatever the real-time factor
        super().__init__('turn_test', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom = None          # (x, y, yaw wrapped)
        self.yaw = 0.0            # odometry yaw, unwrapped over the whole run
        self.prev = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw += 0.0 if self.prev is None else wrap(yaw - self.prev)
        self.prev = yaw
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, self.yaw)

    def send(self, w):
        msg = Twist()
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def spin_for(self, seconds):
        """Spin for some wall-clock time (simulation time is not needed to just wait)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=0.5, help='angular speed [rad/s], negative = clockwise')
    ap.add_argument('--steps', type=int, default=4, help='number of 90 degree steps')
    ap.add_argument('--accel', type=float, default=1.0, help='angular acceleration [rad/s^2]')
    args = ap.parse_args()
    w = abs(args.rate)
    sign = 1.0 if args.rate > 0 else -1.0
    step = math.pi / 2.0
    cruise = step / w - w / args.accel          # time at full speed that makes exactly one step
    if w <= 0 or args.accel <= 0 or cruise < 0:
        raise SystemExit('need rate != 0, accel > 0 and rate^2 <= accel * pi / 2')

    rclpy.init()
    node = Turn()
    start = time.time()
    while rclpy.ok() and (node.odom is None or node.now() <= 0.0) and time.time() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None or node.pub.get_subscription_count() == 0:
        raise SystemExit('no /odom, or nobody listens to /cmd_vel (is the simulation running, and Nav2 stopped?)')

    gt = get_pose()
    gt0, gt_yaw, odom0 = gt, 0.0, node.odom
    print(f'{args.steps} steps of 90 deg at {args.rate:+.2f} rad/s (planned {math.degrees(sign * step):+.0f} deg each)')
    print('step   ground truth [deg]   odometry [deg]   odom / truth   truth centre moved [mm]')
    for k in range(1, args.steps + 1):
        yaw_odom_before, t0, last = node.yaw, node.now(), -1.0
        total = 2.0 * w / args.accel + cruise
        wall0 = time.time()
        while rclpy.ok() and node.now() - t0 < total + 0.5 and time.time() - wall0 < 4.0 * total + 10.0:
            t = node.now() - t0
            if t - last >= 0.02:                  # 50 Hz command rate
                node.send(sign * profile(t, w, args.accel, cruise))
                last = t
            rclpy.spin_once(node, timeout_sec=0.01)
        node.send(0.0)
        node.spin_for(1.0)                        # settle before reading the ground truth
        pose = get_pose()
        d_gt = wrap(pose[2] - gt[2])              # a step is far below 180 degrees: no ambiguity
        gt_yaw += d_gt
        d_odom = node.yaw - yaw_odom_before
        move = 1000.0 * math.hypot(pose[0] - gt[0], pose[1] - gt[1])
        print(f'{k:4d} {math.degrees(d_gt):12.2f} {math.degrees(d_odom):18.2f} {d_odom / d_gt:13.4f} {move:20.1f}')
        gt = pose

    ratio = (node.yaw - odom0[2]) / gt_yaw
    print(f'\ntotal: truth {math.degrees(gt_yaw):+.2f} deg, odometry {math.degrees(node.yaw - odom0[2]):+.2f} deg, '
          f'ratio odom / truth {ratio:.4f}')
    print(f'centre of the robot moved {1000 * math.hypot(gt[0] - gt0[0], gt[1] - gt0[1]):.1f} mm (truth), '
          f'{1000 * math.hypot(node.odom[0] - odom0[0], node.odom[1] - odom0[1]):.1f} mm (odometry)')
    print(f'wheel_separation that would make the ratio 1: {WHEEL_SEPARATION * ratio:.4f} m (now {WHEEL_SEPARATION})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
