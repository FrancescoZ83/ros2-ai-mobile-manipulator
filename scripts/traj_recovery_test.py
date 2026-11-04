#!/usr/bin/env python3
"""A trajectory that starts where the arm is, not where its last goal is, must run.

MoveIt plans from the measured state. If the arm is held away from the goal of its last move (blocked, or short of it),
the first point of the next trajectory is near the arm and far from that goal. The controller used to refuse it, and
every one after it: the arm could never leave. The arm is made to stay away from its goal by switching off the gravity
compensation (`gravity_scale` 0), then three trajectories are sent through the action server:
  A. from the measured pose, a few milliradians away: must start;
  B. from a pose 0.3 rad from the arm (and from the goal): must be refused (INVALID_GOAL);
  C. after the compensation is back, from the measured pose to the goal of the start: must reach it.
Run with the simulation (sim.launch.py world:=warehouse.sdf), the arm idle. Exit code 0 if all three behave.
"""
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
N = 6
Result = FollowJointTrajectory.Result


class Test(Node):
    def __init__(self):
        super().__init__('traj_recovery_test')
        self.state = None
        self.create_subscription(Float64MultiArray, '/arm_model_controller/state', self.on_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_model_controller/follow_joint_trajectory')

    def on_state(self, msg):
        if len(msg.data) == 2 + 5 * N:
            self.state = list(msg.data)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def measured(self):
        self.spin(0.3)
        return self.state[2 + N:2 + 2 * N]

    def goal_of_last_move(self):
        self.spin(0.3)
        return self.state[2:2 + N]

    def set_param(self, name, value):
        subprocess.run(['ros2', 'param', 'set', '/arm_model_controller', name, str(value)], check=True, capture_output=True)

    def send(self, first, last, seconds):
        """Error code of a two-point trajectory from `first` to `last` (rest to rest) and the final error."""
        traj = JointTrajectory(joint_names=JOINTS)
        for t, q in ((0.0, first), (seconds, last)):
            traj.points.append(JointTrajectoryPoint(positions=list(q), velocities=[0.0] * N, accelerations=[0.0] * N,
                                                    time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self.client.wait_for_server(timeout_sec=10.0)
        handle = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle, timeout_sec=10.0)
        result = handle.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=seconds * 3 + 20.0)
        code = result.result().result.error_code
        self.spin(0.5)
        return code, max(abs(a - b) for a, b in zip(self.measured(), last))


def main():
    rclpy.init()
    node = Test()
    ok = True
    try:
        while node.state is None:
            node.spin(0.2)
        home = node.goal_of_last_move()
        print('arm idle, goal of the last move', [round(v, 3) for v in home])

        node.set_param('gravity_scale', 0.0)          # the arm sags: it stops away from its goal
        node.spin(4.0)
        q = node.measured()
        away = max(abs(a - b) for a, b in zip(q, node.goal_of_last_move()))
        print(f'gravity compensation off: the arm is {away:.3f} rad from its goal (the controller accepts 0.05 from the goal)')
        if away <= 0.05:
            print('FAIL  the arm did not move away from its goal: the test does not apply')
            return 2

        far = list(q)
        far[4] += 0.3
        code, _ = node.send(far, far, 1.0)
        refused = code == Result.INVALID_GOAL
        print(f'{"OK  " if refused else "FAIL"}  B: first point 0.3 rad from the arm and the goal -> error code {code} '
              f'({"refused" if refused else "it was not refused"})')
        ok &= refused

        last = list(q)
        last[5] += 0.005
        code, _ = node.send(q, last, 1.0)
        started = code != Result.INVALID_GOAL
        print(f'{"OK  " if started else "FAIL"}  A: first point at the arm, {away:.3f} rad from the goal -> error code {code} '
              f'({"started" if started else "refused: the arm cannot leave"})')
        ok &= started

        node.set_param('gravity_scale', 1.0)
        node.spin(4.0)
        code, error = node.send(node.measured(), home, 4.0)
        reached = code == Result.SUCCESSFUL and error < 0.03
        print(f'{"OK  " if reached else "FAIL"}  C: back to the goal with the compensation on -> error code {code}, '
              f'{error:.3f} rad from the goal')
        ok &= reached
    finally:
        node.set_param('gravity_scale', 1.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
