#!/usr/bin/env python3
"""Try localization.localize and localize_images with made-up detections: a shelf face of five slots 0.2 m apart (no ROS, no simulation).

The shelf is the north face of S1 (normal +y, so the shift along the face is in x). For every shift of the believed map in
the range, and for several fillings of the unit (a slot is empty with the probability --empty), the articles that the camera
would see (the ones within 0.26 m of the centre of the view, with a noise of 5 mm) are given to the function: is the shift
found, and the article of the target slot?

  localize        the inventory (the class in each slot) is given, as the stock of the simulator or a warehouse system gives it.
  localize_images the inventory is not given: only the slots and the class of the article to pick; the class of the other
                  slots is read from the image (--images). Its inventory is compared with the true filling.
"""
import argparse
import random
import sys

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from localization import localize, localize_images  # noqa: E402

XS = [-2.2, -2.0, -1.8, -1.6, -1.4]
NORMAL = (0.0, 1.0)


def trial(rng, filling, target, shift, view=0.26):
    """filling: the label of each slot or None. Returns the slots, the inventory (occupied slots), the detections and, for each,
    the index of its slot."""
    slots = [{'slot': f'n{i + 1}', 'x': XS[i], 'y': -1.05} for i in range(5)]
    inventory = [dict(slots[i], label=filling[i]) for i in range(5) if filling[i] is not None]
    centre = XS[target]
    detections, who = [], []
    for i in range(5):
        if filling[i] is None or abs(XS[i] - centre) > view:      # empty, or outside the image (aimed at where the believed map puts the slot)
            continue
        detections.append({'label': filling[i], 'x': XS[i] + shift + rng.gauss(0, 0.005), 'y': -1.05 + 0.009 + rng.gauss(0, 0.005)})
        who.append(i)
    return slots, inventory, detections, who


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', action='store_true', help='localize_images (the inventory is read) instead of localize (it is given)')
    ap.add_argument('--range', type=float, default=0.2, help='the shift of the believed map is uniform within +-this [m]')
    ap.add_argument('--empty', type=float, default=0.0, help='probability that a slot is empty')
    ap.add_argument('--trials', type=int, default=4000)
    args = ap.parse_args()
    rng = random.Random(1)
    right = wrong = failed = ambiguous = 0
    inv_slots = inv_right = 0
    bad = []
    done = 0
    while done < args.trials:
        filling = [None if rng.random() < args.empty else rng.randint(1, 6) for _ in range(5)]
        target = rng.randrange(5)
        if filling[target] is None:
            continue                                                 # there is nothing to pick in an empty slot
        done += 1
        shift = rng.uniform(-args.range, args.range)
        slots, inventory, detections, who = trial(rng, filling, target, shift)
        if args.images:
            res = localize_images(detections, slots, f'n{target + 1}', filling[target], NORMAL)
        else:
            res = localize(detections, inventory, f'n{target + 1}', NORMAL)
        if 'failed' in res:
            failed += 1
            continue
        found = who[res['target']] == target and abs(res['shift'][0] - shift) < 0.02
        right += found
        wrong += not found
        ambiguous += res['ambiguous']
        if args.images and found:
            for s, label in res['inventory'].items():
                inv_slots += 1
                inv_right += filling[int(s[1:]) - 1] == label
        if not found:
            bad.append((filling, target, round(shift, 3), who[res['target']], round(res['shift'][0], 3), res['score'], res['ambiguous']))
    n = right + wrong + failed
    print(f'{"localize_images" if args.images else "localize"} range +-{args.range} empty {args.empty}: {n} trials: {right} right, '
          f'{wrong} wrong, {failed} without a result; {ambiguous} flagged ambiguous', end='')
    print(f'; inventory read: {inv_right} of {inv_slots} slots right' if args.images else '')
    for b in bad[:6]:
        print('  wrong:', b)
    sys.exit(0 if wrong == 0 else 1)


if __name__ == '__main__':
    main()
