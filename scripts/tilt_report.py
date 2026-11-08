#!/usr/bin/env python3
"""When and how an article tilts, from the JSON of pick_oracle.py --trace-tilt (see tilt_trace.py).

For every attempt with a trace: the final tilt, the time at which the tilt first goes over 5 degrees counted from the moment the
arm starts to move (the retreat), the lean along the approach and across it, and how far the fingers had moved then.
"""
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tilt_trace import parse, tilt  # noqa: E402

APPROACH = (0.0, -1.0)      # the robot looks at the north face of the unit from the aisle: into the shelf is -y


def analyse(path, article, step=None):
    """Dict with the series of the article and the time the retreat starts (the fingers move 2 mm from where they were)."""
    s = parse(path, rf'{re.escape(article)}|gripper_(left|right)_finger_link')
    t, pos, quat = s[article]
    total, along, across = tilt(quat, APPROACH)
    ft, fpos, _ = s['gripper_left_finger_link']
    moved = np.linalg.norm(fpos - fpos[np.searchsorted(ft, ft[0] + 1.5)], axis=1)     # from where the fingers are after the closing
    after = np.nonzero(ft > ft[0] + 1.5)[0]
    start = ft[after[np.nonzero(moved[after] > 0.002)[0][0]]] if (moved[after] > 0.002).any() else math.nan
    return {'t': t - start, 'tilt': total, 'along': along, 'across': across, 'z': pos[:, 2], 'finger': fpos, 'start': start}


if __name__ == '__main__':
    import math
    rows = json.load(open(sys.argv[1]))
    for r in rows:
        if 'trace_file' not in r or not os.path.exists(r['trace_file']):
            continue
        a = analyse(r['trace_file'], r['article'])
        t, tot = a['t'], a['tilt']
        over = np.nonzero(tot > 5.0)[0]
        first = f'{t[over[0]]:+.2f} s' if len(over) else 'never'
        late = tot[t > 0.0]
        print(f'{r.get("attempt", r.get("trial"))}: {r["slot"]} {r["outcome"]:8s} final {tot[-1]:5.1f} deg (along {a["along"][-1]:+6.1f}, across {a["across"][-1]:+6.1f}) '
              f'over 5 deg at {first} from the start of the retreat; tilt at the closing {tot[(t > -1.0) & (t < 0.0)].max() if ((t > -1.0) & (t < 0.0)).any() else float("nan"):.1f}, '
              f'peak {tot.max():.1f}')
