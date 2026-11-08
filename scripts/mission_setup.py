#!/usr/bin/env python3
"""Stock the shelves of a unit for a mission and say which article is ordered (simulation set-up, not part of the robot).

Prints one JSON line {"slot", "article_class", "article"}: the slot of the unit drawn from the seed (the same as pick_oracle.sh
SLOT=draw) and the class of the article that the stock put there. The stock is made again until that slot has an article.
"""
import argparse
import json
import os
import random

import numpy as np
import rclpy
import yaml

import semantic_goals as sg
from dataset_common import Stock


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--unit', default='S1')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--slot', help='the first ordered slot (default: drawn from the seed)')
    ap.add_argument('--orders', type=int, default=1, help='how many articles are ordered: the slots are drawn from the seed, one after the other, all different')
    ap.add_argument('--classes', nargs='+', help='the stock is made again until the ordered slot has an article of one of these classes')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()
    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    rclpy.init()
    stock = Stock(sem, classes, random.Random(args.seed))
    stock.clear_original()
    slots = [args.slot or sg.draw_slot(sem, args.unit, args.seed)]
    k = 1
    while len(slots) < args.orders:                           # the next slots: drawn from other seeds, not a slot already ordered
        nxt = sg.draw_slot(sem, args.unit, args.seed + 1000 * k)
        k += 1
        if nxt not in slots:
            slots.append(nxt)
    face_slots = {n: s for n, s in sem['slots'].items() if s['shelf'] == args.unit and s['pickable']}
    centre_x = float(np.mean([s['pose'][0] for s in face_slots.values()]))
    for _ in range(40):
        records, _ = stock.refill(args.unit, centre_x, 0.9)
        found = [next((r for r in records if r['slot'] == slot and (not args.classes or r['class'] in args.classes)), None) for slot in slots]
        if all(found):
            break
    else:
        raise SystemExit(f'no article in all of {slots} after 40 fillings')
    json.dump({r['slot']: r['class'] for r in records}, open('/tmp/mission_stock.json', 'w'))      # what the simulator really put in each slot (the skill server's adapter reads it)
    orders = [{'slot': r['slot'], 'article_class': r['class'], 'article': r['name']} for r in found]
    print(json.dumps(dict(orders[0], orders=orders)), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
