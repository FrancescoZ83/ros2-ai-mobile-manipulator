#!/usr/bin/env python3
"""Send open/close commands to the gripper controller and print its state (diagnostic)."""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

STATES = ['OPEN', 'OPENING', 'CLOSING', 'GRASP', 'EMPTY']
STATE_LEN = 9  # [state, xc, xd, vc, f_left, f_right, width, grip_target, fc]


class Grip(Node):
    def __init__(self):
        super().__init__('grip')
        self.pub = self.create_publisher(Float64MultiArray, '/gripper_controller/command', 10)
        self.rows = []
        self.create_subscription(
            Float64MultiArray, '/gripper_controller/state', self.on_state, 10)

    def on_state(self, msg):
        if len(msg.data) == STATE_LEN:
            self.rows.append((time.time(), *msg.data))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['open', 'close', 'watch'])
    ap.add_argument('--force', type=float, default=0.0, help='grip force [N], 0 = default')
    ap.add_argument('--watch', type=float, default=2.5, help='seconds of state to record')
    ap.add_argument('--every', type=int, default=3, help='print one row every N samples (50 Hz)')
    args = ap.parse_args()

    rclpy.init()
    node = Grip()

    # Wait until the controller listens to commands and publishes its state
    start = time.time()
    while rclpy.ok() and time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pub.get_subscription_count() > 0 and node.rows:
            break
    if not node.rows:
        print('no state received: is gripper_controller active?')
        node.destroy_node()
        rclpy.shutdown()
        return

    t_cmd = time.time()
    if args.action != 'watch':
        msg = Float64MultiArray()
        msg.data = [0.0 if args.action == 'open' else 1.0, args.force]
        node.pub.publish(msg)
    while rclpy.ok() and time.time() - t_cmd < args.watch:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('  t[s]    state     xc[m]    xd[m]  vc[m/s]   fL[N]   fR[N]  width[mm]')
    prev_state = None
    k = 0
    for t, st, xc, xd, vc, fl, fr, width, _target, _fc in node.rows:
        if t < t_cmd - 0.05:
            continue
        # Print every N-th row, and always the rows where the state machine changed state
        if k % args.every == 0 or st != prev_state:
            print(f'{t - t_cmd:6.2f} {STATES[int(st)]:>8} {xc:9.4f} {xd:+8.4f} {vc:+8.3f} '
                  f'{fl:+7.2f} {fr:+7.2f} {width * 1000:9.2f}')
        prev_state = st
        k += 1
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
