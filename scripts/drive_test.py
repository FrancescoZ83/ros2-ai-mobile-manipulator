#!/usr/bin/env python3
"""Drive straight and report pose drift (Gazebo ground truth), arm error and gripper state."""
import argparse
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

N = 6                      # arm joints
WORLD = 'amr_world'
MODEL = 'amr_manipulator'
NAMES = ['pan', 'lift', 'elbow', 'wrist1', 'wrist2', 'wrist3']
GRIP = ['OPEN', 'OPENING', 'CLOSING', 'GRASP', 'EMPTY']


def field(text, key):
    """Value of 'key: number' in a protobuf text block; fields equal to 0 are not printed."""
    m = re.search(rf'\b{key}: (\S+)', text)
    return float(m.group(1)) if m else 0.0


def parse_pose(out, model):
    """(x, y, yaw) of the named model from the text of one /world/<w>/pose/info message."""
    for block in re.findall(r'^pose \{\n(.*?)^\}', out, re.M | re.S):
        name = re.search(r'name: "([^"]+)"', block)
        if not name or name.group(1) != model:
            continue
        pos = re.search(r'position \{(.*?)\}', block, re.S)
        ori = re.search(r'orientation \{(.*?)\}', block, re.S)
        p = pos.group(1) if pos else ''
        o = ori.group(1) if ori else ''
        x, y, z, w = (field(o, k) for k in 'xyzw')
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return field(p, 'x'), field(p, 'y'), yaw
    raise RuntimeError(f'model {model} not found in pose/info')


def get_pose():
    out = subprocess.run(
        ['ign', 'topic', '-e', '-t', f'/world/{WORLD}/pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=15).stdout
    return parse_pose(out, MODEL)


def profile(t, v, a, t_cruise):
    """Trapezoidal speed: ramp up at a, cruise for t_cruise, ramp down at a."""
    t_ramp = v / a
    if t < t_ramp:
        return a * t
    if t < t_ramp + t_cruise:
        return v
    return max(0.0, v - a * (t - t_ramp - t_cruise))


class Drive(Node):
    def __init__(self):
        # Simulated time: the speed profile must match the physics, whatever the real-time factor
        super().__init__('drive_test', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Float64MultiArray, '/arm_model_controller/state', self.on_arm, 10)
        self.create_subscription(
            Float64MultiArray, '/gripper_controller/state', self.on_grip, 10)
        self.recording = False
        self.arm_seen = False
        self.err = [[] for _ in range(N)]
        self.grip = []

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v):
        msg = Twist()
        msg.linear.x = float(v)
        self.pub.publish(msg)

    def on_arm(self, msg):
        d = msg.data
        if len(d) != 2 + 5 * N:
            return
        self.arm_seen = True
        if self.recording:
            for i in range(N):
                self.err[i].append(d[2 + i] - d[2 + N + i])   # reference - measured

    def on_grip(self, msg):
        if self.recording and len(msg.data) == 9:
            self.grip.append(tuple(msg.data))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.3, help='speed [m/s], negative = backwards')
    ap.add_argument('--time', type=float, default=6.0, help='time at full speed [s]')
    ap.add_argument('--accel', type=float, default=0.5, help='ramp acceleration [m/s^2]')
    ap.add_argument('--settle', type=float, default=2.0, help='time at rest before the final pose [s]')
    args = ap.parse_args()
    if args.accel <= 0.0 or args.time < 0.0 or args.speed == 0.0:
        raise SystemExit('need accel > 0, time >= 0 and speed != 0')

    rclpy.init()
    node = Drive()
    start = time.time()
    while rclpy.ok() and time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pub.get_subscription_count() > 0 and node.arm_seen and node.now() > 0.0:
            break
    if node.pub.get_subscription_count() == 0:
        raise SystemExit('nobody listens to /cmd_vel')

    p0 = get_pose()
    v = abs(args.speed)
    sign = 1.0 if args.speed > 0 else -1.0
    t_total = 2.0 * v / args.accel + args.time
    planned = v * (v / args.accel + args.time)
    print(f'driving {args.speed:+.2f} m/s, {args.time:.1f} s at full speed, '
          f'planned distance {planned:.2f} m')

    node.recording = True
    t0 = node.now()
    wall0 = time.time()
    last = -1.0
    while rclpy.ok() and time.time() - wall0 < 4.0 * (t_total + args.settle) + 10.0:
        t = node.now() - t0
        if t > t_total + args.settle:
            break
        if t - last >= 0.02:                      # 50 Hz command rate
            node.send(sign * profile(t, v, args.accel, args.time))
            last = t
        rclpy.spin_once(node, timeout_sec=0.01)
    node.send(0.0)
    node.recording = False
    p1 = get_pose()

    # Displacement in the frame of the initial heading
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    along = dx * c + dy * s
    lateral = -dx * s + dy * c
    dyaw = math.degrees((p1[2] - p0[2] + math.pi) % (2.0 * math.pi) - math.pi)
    print('ground truth (Gazebo):')
    print(f'  along heading {along:+8.3f} m')
    print(f'  lateral       {lateral:+8.3f} m')
    print(f'  yaw change    {dyaw:+8.3f} deg   ({dyaw / max(abs(along), 1e-6):+.3f} deg/m)')

    print('arm error during the run [rad]:')
    print('  joint       rms       max')
    for i in range(N):
        e = node.err[i]
        if e:
            rms = math.sqrt(sum(x * x for x in e) / len(e))
            print(f'  {NAMES[i]:<7} {rms:8.4f}  {max(abs(x) for x in e):8.4f}')
    if node.grip:
        states = sorted({GRIP[int(g[0])] for g in node.grip})
        print(f'gripper: states seen {states}, max |xd| {1000 * max(abs(g[2]) for g in node.grip):.2f} mm, '
              f'last fL/fR {node.grip[-1][4]:+.2f}/{node.grip[-1][5]:+.2f} N')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
