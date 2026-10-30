#!/usr/bin/env python3
"""Send a joint goal to arm_model_controller and print tracking statistics."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

N = 6
NAMES = ['pan', 'lift', 'elbow', 'wrist1', 'wrist2', 'wrist3']
SETTLE = 2.0  # [s] observed after the end of the trajectory


class GotoTest(Node):
    def __init__(self, goal, duration):
        super().__init__('arm_goto')
        self.goal = goal
        self.duration = duration
        self.pub = self.create_publisher(Float64MultiArray, '/arm_model_controller/goal', 10)
        self.create_subscription(
            Float64MultiArray, '/arm_model_controller/state', self.on_state, 10)
        self.sent = False
        self.started = False
        self.done = False
        self.moving = []    # (error[N], tau[N]) while the trajectory runs
        self.settling = []  # error[N] in the second half of the settling window

    def on_state(self, msg):
        d = msg.data
        if len(d) != 2 + 5 * N:
            return
        t, big_t = d[0], d[1]
        err = [d[2 + i] - d[2 + N + i] for i in range(N)]  # q_d - q
        tau = d[2 + 4 * N:2 + 5 * N]

        # Wait until the controller is idle, then send the goal once
        if not self.sent:
            if t < big_t:
                return
            self.pub.publish(Float64MultiArray(data=self.goal + [self.duration]))
            self.sent = True
            return

        # The new trajectory is recognised by its duration and by t_traj < T
        if not self.started:
            self.started = big_t >= self.duration - 1e-9 and t < big_t
            if not self.started:
                return
            if big_t > self.duration + 1e-9:
                print(f'note: duration stretched from {self.duration} s to {big_t:.3f} s (velocity limit)')

        if t <= big_t:
            self.moving.append((err, tau))
        elif t <= big_t + SETTLE:
            if t >= big_t + SETTLE / 2:
                self.settling.append((err, tau))
        else:
            self.done = True
        

    def report(self):
        print('joint     rms[rad]  max[rad]  mean[rad]  final[rad]  max|tau|[Nm]  tau_std[Nm]')
        for i in range(N):
            e = [s[0][i] for s in self.moving]
            rms = math.sqrt(sum(x * x for x in e) / len(e))
            peak = max(abs(x) for x in e)
            mean = sum(e) / len(e)  # signed mean error during the motion
            final = sum(s[0][i] for s in self.settling) / len(self.settling)
            tmax = max(abs(s[1][i]) for s in self.moving)
            taus = [s[1][i] for s in self.settling]
            mu = sum(taus) / len(taus)
            tau_std = math.sqrt(sum((x - mu) ** 2 for x in taus) / len(taus))  # torque jitter at rest
            print(f'{NAMES[i]:8s} {rms:9.4f} {peak:9.4f} {mean:+10.4f} {final:+11.4f} '
                  f'{tmax:12.2f} {tau_std:12.3f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=float, nargs=N, required=True)
    parser.add_argument('--time', type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = GotoTest(args.goal, args.time)
    start = time.time()
    while rclpy.ok() and not node.done and time.time() - start < 60.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.done and node.moving and node.settling:
        node.report()
    else:
        print('No complete measurement (is the controller active? is the sim running?)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
