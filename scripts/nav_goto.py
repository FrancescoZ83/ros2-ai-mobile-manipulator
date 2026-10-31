#!/usr/bin/env python3
"""Send goals to Nav2 and report time, path length and the final error against the Gazebo ground truth."""
import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

from drive_test import get_pose      # (x, y, yaw) of the robot in the Gazebo world

# Goals (x, y, yaw) in the map frame, which is the world frame. Doors from semantic_map.yaml:
# P1 (-3, 0), P2 (-3, -3), P3 (3, 0), P4 (3, 3). Stations stand 0.2 m or more from the benches.
SUITES = {
    'doors': [
        ('P1 into receiving', -3.8, 0.0, math.pi),
        ('receiving south', -4.2, -3.0, math.pi),
        ('P2 into south aisle', -1.0, -3.0, 0.0),
        ('P2 out to receiving', -3.8, 0.0, math.pi),
        ('receiving to packing (P1, P3)', 3.8, 0.0, 0.0),
        ('P4 into north aisle', 0.5, 3.0, math.pi),
    ],
}


def make_pose(x, y, yaw):
    """PoseStamped in the map frame; a zero stamp means 'latest transform'."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def wrap(a):
    """Angle in [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def map_pose(buf):
    """(x, y, yaw) of base_footprint in the map frame, as Nav2 believes it, or None."""
    try:
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
    except Exception:
        return None
    q = t.rotation
    return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def run_goal(nav, buf, name, x, y, yaw, timeout):
    """Drive to one goal and return a dict of measurements."""
    nav.goToPose(make_pose(x, y, yaw))
    start, last, length, recoveries = time.monotonic(), None, 0.0, 0
    nav_time = 0.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None:
            p = fb.current_pose.pose.position
            if last is not None:
                length += math.hypot(p.x - last[0], p.y - last[1])
            last = (p.x, p.y)
            recoveries = fb.number_of_recoveries
            nav_time = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        if time.monotonic() - start > timeout:
            nav.cancelTask()
            break
    result = nav.getResult() if time.monotonic() - start <= timeout else TaskResult.CANCELED
    # Let the robot settle, then compare Gazebo's pose with the goal and with Nav2's own estimate
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)
    gx, gy, gyaw = get_pose()
    est = map_pose(buf)
    return {
        'name': name, 'result': result, 'time': nav_time, 'length': length, 'recoveries': recoveries,
        'err_xy': math.hypot(gx - x, gy - y), 'err_yaw': math.degrees(abs(wrap(gyaw - yaw))),
        # Error in the frame of the goal: along its heading, and across it (positive = to its left)
        'along': (gx - x) * math.cos(yaw) + (gy - y) * math.sin(yaw),
        'across': -(gx - x) * math.sin(yaw) + (gy - y) * math.cos(yaw),
        'loc_xy': None if est is None else math.hypot(gx - est[0], gy - est[1]),
    }


def stat(values):
    """'mean / max / std' of a list of numbers."""
    if not values:
        return '-'
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return f'{statistics.mean(values):.3f} / {max(values):.3f} / {std:.3f}'


def run_goals(goals, timeout, repeat=1):
    """Send the goals (name, x, y, yaw) in order, `repeat` times; print the report; True if all were reached."""
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    buf = Buffer()
    TransformListener(buf, nav)       # AMCL sets the initial pose from the parameters: nothing to publish here

    rows = []
    for name, x, y, yaw in goals * repeat:
        print(f'-> {name}: ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f} deg)', flush=True)
        row = run_goal(nav, buf, name, x, y, yaw, timeout)
        rows.append(row)
        print(f"   {row['result'].name}: {row['time']:.1f} s of simulation, path {row['length']:.2f} m, "
              f"{row['recoveries']} recoveries, error {row['err_xy']:.3f} m / {row['err_yaw']:.1f} deg", flush=True)

    ok = [r for r in rows if r['result'] == TaskResult.SUCCEEDED]
    print('\ngoal                           result     time[s]  path[m]  rec  err[m]  along[m] across[m]  yaw[deg]  loc[m]')
    for r in rows:
        loc = '-' if r['loc_xy'] is None else f"{r['loc_xy']:.3f}"
        print(f"{r['name']:30s} {r['result'].name:10s} {r['time']:7.1f} {r['length']:8.2f} {r['recoveries']:4d} "
              f"{r['err_xy']:7.3f} {r['along']:+9.3f} {r['across']:+9.3f} {r['err_yaw']:9.1f} {loc:>7s}")
    if repeat > 1:
        print(f'\nper goal over {repeat} runs:   mean time[s]  max time[s]  recoveries  reached')
        for name, *_ in goals:
            mine = [r for r in rows if r['name'] == name]
            times = [r['time'] for r in mine]
            reached = sum(r['result'] == TaskResult.SUCCEEDED for r in mine)
            print(f'{name:30s} {statistics.mean(times):12.1f} {max(times):12.1f} '
                  f"{sum(r['recoveries'] for r in mine):11d} {reached:4d}/{len(mine)}")
    print(f'\nreached {len(ok)} of {len(rows)}')
    print('final position error vs ground truth [m], mean / max / std :', stat([r['err_xy'] for r in ok]))
    print('  along the goal heading [m], mean (signed) / max |.| / std  :', f"{statistics.mean(r['along'] for r in ok):+.3f} / "
          f"{max(abs(r['along']) for r in ok):.3f} / {statistics.pstdev(r['along'] for r in ok):.3f}" if ok else '-')
    print('  across it, positive to the left [m], same                  :', f"{statistics.mean(r['across'] for r in ok):+.3f} / "
          f"{max(abs(r['across']) for r in ok):.3f} / {statistics.pstdev(r['across'] for r in ok):.3f}" if ok else '-')
    print('final yaw error vs ground truth [deg], mean / max / std    :', stat([r['err_yaw'] for r in ok]))
    print('Nav2 estimate vs ground truth [m], mean / max / std        :', stat([r['loc_xy'] for r in rows if r['loc_xy'] is not None]))
    nav.destroy_node()
    rclpy.shutdown()
    return len(ok) == len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', choices=sorted(SUITES), help='run a named sequence of goals')
    ap.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'YAW'), help='one goal in the map frame [m, m, rad]')
    ap.add_argument('--timeout', type=float, default=150.0, help='wall-clock seconds allowed per goal')
    ap.add_argument('--repeat', type=int, default=1, help='run the whole list of goals this many times')
    args = ap.parse_args()
    if bool(args.suite) == bool(args.goal):
        ap.error('give exactly one of --suite and --goal')
    if args.repeat < 1:
        ap.error('--repeat must be at least 1')
    goals = SUITES[args.suite] if args.suite else [('goal', *args.goal)]
    sys.exit(0 if run_goals(goals, args.timeout, args.repeat) else 1)


if __name__ == '__main__':
    main()
