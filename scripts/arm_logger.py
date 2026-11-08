#!/usr/bin/env python3
"""Record /arm_model_controller/state to a CSV until it is killed (diagnostic).

Every row: wall time, then the whole vector of the controller: t, T of the trajectory in progress, the reference of the six
joints, their measured positions, and the rest of the state. About 50 rows a second.
    python3 arm_logger.py arm.csv
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Logger(Node):
    def __init__(self, path):
        super().__init__('arm_logger')
        self.f = open(path, 'w')
        self.n = 0
        self.create_subscription(Float64MultiArray, '/arm_model_controller/state', self.on_state, 50)

    def on_state(self, msg):
        self.f.write(f'{time.time():.3f},' + ','.join(f'{v:.5f}' for v in msg.data) + '\n')
        self.n += 1
        if self.n % 50 == 0:
            self.f.flush()


def main():
    rclpy.init()
    node = Logger(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.f.flush()


if __name__ == '__main__':
    main()
