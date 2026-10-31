#!/usr/bin/env python3
"""Check a running stocked warehouse: average real time factor and place of the dynamic articles."""
import argparse
import math
import re
import subprocess
import time

import yaml


def ign_echo(topic):
    """One message of a Gazebo topic, as text."""
    return subprocess.run(['ign', 'topic', '-e', '-t', topic, '-n', '1'],
                          capture_output=True, text=True, timeout=20).stdout


def clock(text, name):
    """Seconds in the sim_time or real_time block of a world stats message (zero fields are omitted)."""
    block = re.search(name + r'\s*\{([^}]*)\}', text).group(1)
    sec = re.search(r'\bsec:\s*(\d+)', block)
    nsec = re.search(r'\bnsec:\s*(\d+)', block)
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9


def positions(text):
    """name -> (x, y, z) of every entity in a pose/info message (zero fields are omitted)."""
    out = {}
    for chunk in text.split('pose {')[1:]:
        name = re.search(r'name:\s*"([^"]+)"', chunk)
        pos = re.search(r'position\s*\{([^}]*)\}', chunk)
        if not name or not pos:
            continue
        xyz = [re.search(rf'\b{axis}:\s*(-?[\d.eE+-]+)', pos.group(1)) for axis in 'xyz']
        out[name.group(1)] = tuple(float(m.group(1)) if m else 0.0 for m in xyz)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='semantic_map.yaml of the running world')
    ap.add_argument('--world', default='amr_world')
    ap.add_argument('--window', type=float, default=30.0, help='seconds over which the RTF is averaged')
    ap.add_argument('--tol', type=float, default=0.01, help='allowed displacement of an article [m]')
    args = ap.parse_args()

    slots = yaml.safe_load(open(args.map))['slots']
    stats = f'/world/{args.world}/stats'

    # Real time factor = simulated time / wall time between two samples
    a = ign_echo(stats)
    sim0, real0 = clock(a, 'sim_time'), clock(a, 'real_time')
    time.sleep(args.window)
    b = ign_echo(stats)
    sim1, real1 = clock(b, 'sim_time'), clock(b, 'real_time')
    print(f'real time factor over {real1 - real0:.1f} s: {(sim1 - sim0) / (real1 - real0):.3f} '
          f'(simulation time now {sim1:.1f} s)')

    # Dynamic articles must still be where the semantic map says
    now = positions(ign_echo(f'/world/{args.world}/pose/info'))
    wanted = {s['model']: s for s in slots.values() if s['article'] and s['dynamic']}
    missing, moved, worst = [], [], []
    for model, s in wanted.items():
        if model not in now:
            missing.append(model)
            continue
        x, y, z = now[model]
        dxy = math.hypot(x - s['pose'][0], y - s['pose'][1])
        dz = z - s['pose'][2]
        worst.append((max(dxy, abs(dz)), model, dxy, dz))
        if dxy > args.tol or abs(dz) > args.tol:
            moved.append(model)
    print(f'dynamic articles: {len(wanted)}, found {len(wanted) - len(missing)}, '
          f'moved more than {args.tol * 1000:.0f} mm: {len(moved)}')
    if missing:
        print('  not in pose/info:', ', '.join(missing[:5]), '...' if len(missing) > 5 else '')
    for _, model, dxy, dz in sorted(worst, reverse=True)[:5]:
        print(f'  {model:18s} dxy {dxy * 1000:6.1f} mm   dz {dz * 1000:+6.1f} mm')


if __name__ == '__main__':
    main()
