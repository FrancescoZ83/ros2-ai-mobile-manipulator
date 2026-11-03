#!/usr/bin/env python3
"""FollowJointTrajectory action server on top of ArmModelController's ~/trajectory topic."""
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
Result = FollowJointTrajectory.Result


def seconds(duration):
    """builtin_interfaces/Duration -> float seconds."""
    return duration.sec + duration.nanosec * 1e-9


class TrajectoryServer(Node):
    def __init__(self):
        super().__init__('follow_joint_trajectory_server')
        self.declare_parameter('controller', 'arm_model_controller')
        self.declare_parameter('joints', JOINTS)
        # The arm stops up to 3 mrad short on some joints and up to 0.5-1 degree on wrist3 (stiction)
        self.declare_parameter('goal_tolerance', 0.03)       # [rad] used when the goal gives none
        self.declare_parameter('settle_time', 0.2)           # [s] wait after the end before the check
        self.declare_parameter('start_timeout', 2.0)         # [s] wall-clock wait for the controller to start
        ctrl = self.get_parameter('controller').value
        self.joints = list(self.get_parameter('joints').value)
        self.n = len(self.joints)

        group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointTrajectory, f'/{ctrl}/trajectory', 10)
        self.create_subscription(Float64MultiArray, f'/{ctrl}/state', self.on_state, 10, callback_group=group)
        self.state = None            # (t, T, q_desired, q_measured) of the last debug sample
        self.busy = False
        self.lock = threading.Lock()
        ActionServer(self, FollowJointTrajectory, f'/{ctrl}/follow_joint_trajectory',
                     execute_callback=self.execute, goal_callback=self.on_goal,
                     cancel_callback=self.on_cancel, callback_group=group)
        self.get_logger().info(f'follow_joint_trajectory ready on /{ctrl}/follow_joint_trajectory')

    def on_state(self, msg):
        d, n = msg.data, self.n
        if len(d) == 2 + 5 * n:
            self.state = (d[0], d[1], list(d[2:2 + n]), list(d[2 + n:2 + 2 * n]))

    def on_goal(self, goal):
        with self.lock:
            if self.busy:
                self.get_logger().warn('goal rejected: another trajectory is running')
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, _):
        # The controller cannot stop a trajectory half way: it follows it to the end
        self.get_logger().warn('cancel rejected: the controller cannot stop a running trajectory')
        return CancelResponse.REJECT

    def wait_for(self, condition, timeout):
        """Poll the controller state (wall-clock timeout) until condition(state) holds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state is not None and condition(self.state):
                return True
            time.sleep(0.01)
        return False

    def fail(self, goal_handle, code, text):
        self.get_logger().error(text)
        result = Result()
        result.error_code = code
        result.error_string = text
        goal_handle.abort()
        return result

    def execute(self, goal_handle):
        try:
            return self.run(goal_handle)
        finally:
            with self.lock:
                self.busy = False

    def run(self, goal_handle):
        goal = goal_handle.request
        traj = goal.trajectory
        if set(traj.joint_names) != set(self.joints) or len(traj.joint_names) != self.n:
            return self.fail(goal_handle, Result.INVALID_JOINTS, f'joints must be {self.joints}')
        if len(traj.points) < 2:
            return self.fail(goal_handle, Result.INVALID_GOAL, 'a trajectory needs at least 2 points')
        duration = seconds(traj.points[-1].time_from_start)
        if self.state is None:
            return self.fail(goal_handle, Result.INVALID_GOAL, 'no state from the controller yet')

        self.pub.publish(traj)
        # Started = the controller reports this duration and is not idle (t < T)
        started = self.wait_for(lambda s: abs(s[1] - duration) < 1e-4 and s[0] < s[1],
                                self.get_parameter('start_timeout').value)
        if not started:
            return self.fail(goal_handle, Result.INVALID_GOAL,
                             'the controller did not start the trajectory (busy, or its first point is '
                             'far from where the arm is, or a value is out of limits: see its log)')

        # Follow it to the end, keeping the largest tracking error of every joint
        worst = [0.0] * self.n
        end = time.monotonic() + 3.0 * duration + 10.0
        while time.monotonic() < end:
            s = self.state
            worst = [max(w, abs(d - q)) for w, d, q in zip(worst, s[2], s[3])]
            if s[0] >= s[1]:
                break
            time.sleep(0.01)
        else:
            return self.fail(goal_handle, Result.INVALID_GOAL, 'timeout waiting for the end of the trajectory')

        time.sleep(self.get_parameter('settle_time').value)
        q = self.state[3]
        last = dict(zip(traj.joint_names, traj.points[-1].positions))
        default = self.get_parameter('goal_tolerance').value
        tol = {t.name: t.position for t in goal.goal_tolerance if t.position > 0.0}
        for name, q_now in zip(self.joints, q):
            if abs(q_now - last[name]) > tol.get(name, default):
                return self.fail(goal_handle, Result.GOAL_TOLERANCE_VIOLATED,
                                 f'{name} ended {abs(q_now - last[name]):.4f} rad from the goal '
                                 f'(tolerance {tol.get(name, default):.4f})')
        path = {t.name: t.position for t in goal.path_tolerance if t.position > 0.0}
        for name, w in zip(self.joints, worst):
            if name in path and w > path[name]:
                return self.fail(goal_handle, Result.PATH_TOLERANCE_VIOLATED,
                                 f'{name} was {w:.4f} rad from the path (tolerance {path[name]:.4f})')
        goal_handle.succeed()
        result = Result()
        result.error_code = Result.SUCCESSFUL
        return result


def main():
    rclpy.init()
    node = TrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
