#!/usr/bin/env python3
"""Create and remove articles in the running simulation, through Gazebo's services. The articles are static."""
import argparse
import re
import subprocess
import time

WORLD = 'amr_world'


class WorldError(RuntimeError):
    """The world does not do what was asked of it: whoever is generating data should restart it."""


def call(service, request_type, request, timeout_ms=5000):
    """Call a service of the world with a protobuf text request; True if it answered 'data: true'.

    The answer is not the truth: once the world has been busy for a while a call that did its work is often answered
    with a timeout. Whoever needs to know what is in the world reads it with `models`."""
    try:
        out = subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', request_type,
                              '--reptype', 'ignition.msgs.Boolean', '--timeout', str(timeout_ms), '--req', request],
                             capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return False
    return 'data: true' in out.stdout


def models(prefix, attempts=3):
    """The names of the models of the world that start with prefix, read from the scene of the world."""
    for _ in range(attempts):
        try:
            out = subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/scene/info', '--reqtype', 'ignition.msgs.Empty',
                                  '--reptype', 'ignition.msgs.Scene', '--timeout', '5000', '--req', ''],
                                 capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            continue
        if 'name:' in out.stdout:
            return set(re.findall(rf'name: "({re.escape(prefix)}[^"]*)"', out.stdout))
    raise WorldError('the scene of the world cannot be read')


def real_time_factor():
    """The real time factor of the world now, or None if it cannot be read."""
    try:
        out = subprocess.run(['ign', 'topic', '-e', '-t', f'/world/{WORLD}/stats', '-n', '1'],
                             capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        return None
    found = re.search(r'real_time_factor: ([0-9.eE+-]+)', out)
    return float(found.group(1)) if found else None


def create(name, model, x, y, z, yaw=0.0, static=True):
    """A copy of model://<model> called name, standing at (x, y, z) with the given yaw.

    Static by default: it cannot be pushed or lifted, and nothing makes it fall. static=False makes a dynamic body."""
    sdf = (f"<sdf version='1.8'><include><uri>model://{model}</uri><name>{name}</name><static>{'true' if static else 'false'}</static>"
           f"<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose></include></sdf>")
    return call('create', 'ignition.msgs.EntityFactory', f'sdf: "{sdf}"')


def set_pose(name, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Move the model called name to a pose (position and quaternion in the world)."""
    return call('set_pose', 'ignition.msgs.Pose', f'name: "{name}" position {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}} '
                f'orientation {{x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f}}}', timeout_ms=3000)


def remove(name):
    """Remove the model called name."""
    return call('remove', 'ignition.msgs.Entity', f'name: "{name}" type: MODEL')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('create')
    p.add_argument('name')
    p.add_argument('model')
    p.add_argument('x', type=float)
    p.add_argument('y', type=float)
    p.add_argument('z', type=float)
    p.add_argument('--yaw', type=float, default=0.0)
    sub.add_parser('remove').add_argument('name')
    args = ap.parse_args()
    t0 = time.monotonic()
    ok = create(args.name, args.model, args.x, args.y, args.z, args.yaw) if args.cmd == 'create' else remove(args.name)
    print(f'{args.cmd} {args.name}: {"ok" if ok else "FAILED"} in {time.monotonic() - t0:.2f} s')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
