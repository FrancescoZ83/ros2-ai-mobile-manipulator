#!/usr/bin/env python3
"""Print finger positions, inner gap and centring error from /joint_states (diagnostic)."""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEFT = 'gripper_left_finger_joint'
RIGHT = 'gripper_right_finger_joint'
OPEN_GAP = 0.085  # inner-face distance at q = 0: 2 * (0.0485 - 0.012 / 2), from the URDF


class Probe(Node):
    def __init__(self, every):
        super().__init__('grip_probe')
        self.every = every
        self.count = 0
        self.t0 = None
        self.rows = []
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)

    def on_js(self, msg):
        # Ignore messages that do not carry both fingers
        if LEFT not in msg.name or RIGHT not in msg.name:
            return
        self.count += 1
        if self.count % self.every:
            return
        ql = msg.position[msg.name.index(LEFT)]
        qr = msg.position[msg.name.index(RIGHT)]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.rows.append((t - self.t0, ql, qr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=4.0, help='wall-clock seconds to record')
    ap.add_argument('--every', type=int, default=20, help='keep one /joint_states message every N')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.every)
    start = time.time()
    while rclpy.ok() and time.time() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('   t[s]    qL[m]    qR[m]   qL-qR[m]   gap[m]  centre[m]')
    prev = None
    for k, (t, ql, qr) in enumerate(node.rows):
        # Print only rows where a finger moved, plus the last row (compact output)
        moved = prev is None or abs(ql - prev[0]) > 1e-5 or abs(qr - prev[1]) > 1e-5
        if moved or k == len(node.rows) - 1:
            gap = OPEN_GAP - ql - qr    # distance between the inner faces
            centre = 0.5 * (qr - ql)    # y of the gap midpoint in gripper_base_link
            print(f'{t:7.2f} {ql:8.4f} {qr:8.4f} {ql - qr:+10.4f} {gap:8.4f} {centre:+10.4f}')
            prev = (ql, qr)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
