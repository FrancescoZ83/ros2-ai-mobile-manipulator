#!/usr/bin/env python3
"""Counts from the JSON of pick_oracle.py or pick_series.sh: outcomes, by level and by class.

An attempt is `held` when the gripper reports GRASP and the article is within 40 mm of the TCP after the retreat (what
pick_oracle.py calls success) and `clean` when it is within 10 mm of it: a firm grasp. The slot name gives the level
(`S1.L2.n4`: level 2).
"""
import collections
import json
import sys


def level(row):
    return row['slot'].split('.')[1]


def clean(row):
    return row['outcome'] == 'success' and row.get('article_to_tcp_mm', 1e9) < 10.0


def table(rows, key):
    groups = collections.defaultdict(list)
    for r in rows:
        groups[key(r)].append(r)
    for name in sorted(groups):
        g = groups[name]
        print(f'  {name:14s} {len(g):3d} attempts: {sum(r["outcome"] == "success" for r in g):3d} held, {sum(clean(r) for r in g):3d} clean')


def main():
    rows = json.load(open(sys.argv[1]))
    n = len(rows)
    held = sum(r['outcome'] == 'success' for r in rows)
    print(f'{n} attempts: {held} held ({100 * held / n:.0f} %), {sum(clean(r) for r in rows)} clean (article within 10 mm of the TCP)')
    print('outcomes:', dict(collections.Counter(r['outcome'] for r in rows)))
    print('by level:')
    table(rows, level)
    print('by class:')
    table(rows, lambda r: r['class'])
    bad = [r for r in rows if r['outcome'] != 'success']
    if bad:
        print('not held:')
        for r in bad:
            print(f'  attempt {r.get("attempt", r.get("trial"))}: {r["class"]:13s} {r["slot"]:10s} {r["outcome"]:16s} pregrasp {r.get("pregrasp")} approach {r.get("approach")} '
                  f'grip {r.get("grip_state")} to_tcp {r.get("article_to_tcp_mm")} mm lifted {r.get("lifted_mm")} mm')


if __name__ == '__main__':
    main()
