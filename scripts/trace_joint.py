#!/usr/bin/env python3
"""Time trace of one joint while a goal is executed (diagnostic, not part of the stack)."""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

N = 6  # number of arm joints


class Trace(Node):
    def __init__(self, joint, goal, duration):
        super().__init__('trace_joint')
        self.joint = joint
        self.goal = goal
        self.duration = duration
        self.sent = False
        self.recording = False
        self.finished = False
        self.big_t = 0.0
        self.rows = []
        self.pub = self.create_publisher(Float64MultiArray, '/arm_model_controller/goal', 10)
        self.create_subscription(
            Float64MultiArray, '/arm_model_controller/state', self.on_state, 10)

    def on_state(self, msg):
        d = msg.data
        if len(d) != 2 + 5 * N:
            return
        t, big_t = d[0], d[1]
        i = self.joint

        if not self.sent:
            # Send the goal only when the controller is idle and the controller is listening
            if t >= big_t and self.pub.get_subscription_count() > 0:
                m = Float64MultiArray()
                m.data = list(self.goal) + [self.duration]
                self.pub.publish(m)
                self.sent = True
            return

        if not self.recording:
            # A new trajectory has T >= requested duration (it may be stretched) and t restarted
            if big_t >= self.duration - 1e-9 and t < big_t:
                self.recording = True
                self.big_t = big_t
            else:
                return

        if t <= self.big_t + 1.5:
            self.rows.append((
                t,
                d[2 + i],              # reference position
                d[2 + N + i],          # measured position
                d[2 + 2 * N + i],      # reference velocity
                d[2 + 3 * N + i],      # measured (noisy, filtered) velocity
                d[2 + 4 * N + i]))     # commanded torque
        else:
            self.finished = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', type=float, nargs=N, required=True)
    ap.add_argument('--time', type=float, default=3.0)
    ap.add_argument('--joint', type=int, default=5, help='joint index 0..5 (5 = wrist3)')
    ap.add_argument('--every', type=int, default=5, help='print one row every N samples')
    args = ap.parse_args()

    rclpy.init()
    node = Trace(args.joint, args.goal, args.time)
    start = time.time()
    while rclpy.ok() and not node.finished and time.time() - start < args.time + 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('   t[s]   ref[rad]     q[rad]   err[rad]  vref[r/s]  vmeas[r/s]   tau[Nm]')
    for k, (t, ref, q, vref, vmeas, tau) in enumerate(node.rows):
        if k % args.every == 0:
            print(f'{t:7.2f} {ref:10.4f} {q:10.4f} {ref - q:+10.4f} '
                  f'{vref:+10.3f} {vmeas:+11.3f} {tau:+9.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
