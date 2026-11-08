"""The shift of the believed map along a shelf, from what the camera sees: which of the articles in the image is the one to pick.

AMCL fixes the distance to a flat shelf face and the heading, not the position along it (up to 17 cm off in the
simulation, see the README, "Pick with Nav2"). The camera looks at the articles of the unit instead. Every article in
the image gives an estimate of its position in the believed map; the inventory says which class stands in which slot,
and where the slot is in the map. A shift `e` of the believed map that puts the articles seen on the slots of the same
class is a candidate; the one that explains the most articles is taken, and with it the article of the slot to pick.

Only numpy: nothing here needs ROS or the simulation (localize_check.py tries it with made-up detections).
"""
import math

import numpy as np

NORMAL_TOL = 0.045      # [m] across the face (the direction the LiDAR fixes): the shift is at most this (measured: 0.027, plus
                        # the 1.4 cm of play of the article in its slot)
MAX_SHIFT = 0.25        # [m] along the face: farther than this is not a localization error, it is another article
MATCH_RADIUS = 0.03     # [m] a detection belongs to a slot when it is this close to it once the shift is applied


def match(detections, inventory, shift):
    """{slot index in `inventory`: detection index} for the shift `shift` (x, y): each detection goes to the nearest slot
    of its class within MATCH_RADIUS, and a slot keeps only the nearest detection."""
    best = {}
    for i, d in enumerate(detections):
        near, near_d = None, MATCH_RADIUS
        for k, item in enumerate(inventory):
            if item['label'] != d['label']:
                continue
            dist = math.hypot(d['x'] - item['x'] - shift[0], d['y'] - item['y'] - shift[1])
            if dist <= near_d:
                near, near_d = k, dist
        if near is not None and (near not in best or near_d < best[near][1]):
            best[near] = (i, near_d)
    return {k: v[0] for k, v in best.items()}


def localize(detections, inventory, target, normal):
    """The shift of the believed map and the detection of the slot `target`.

    detections: dicts with label, x, y (position in the believed map, from the stereo and the shape of the class).
    inventory: dicts with slot, label, x, y (the slots of the face, the class that stands in each, the nominal position).
    target: the name of the slot to pick. normal: unit vector (x, y) out of the face.
    Returns {'shift': (x, y), 'tangent': along the face, 'score': articles explained, 'target': index in `detections`,
    'matched': {slot: index}, 'ambiguous': another shift explained as many articles with another article for the target}
    or {'failed': reason}."""
    n = np.asarray(normal, dtype=float)
    t = np.array([-n[1], n[0]])
    guesses = []
    for d in detections:
        for item in inventory:
            if item['label'] == d['label']:
                e = np.array([d['x'] - item['x'], d['y'] - item['y']])
                if abs(e @ n) <= NORMAL_TOL and abs(e @ t) <= MAX_SHIFT:
                    guesses.append(e)
    if not guesses:
        return {'failed': f'{len(detections)} articles seen, none fits a slot of its class within the limits of the shift'}
    found = []
    for e in guesses:
        pairs = match(detections, inventory, e)
        for _ in range(2):                              # the mean of the residuals of those that match, then again
            e = np.mean([[detections[i]['x'] - inventory[k]['x'], detections[i]['y'] - inventory[k]['y']]
                         for k, i in pairs.items()], axis=0)
            pairs = match(detections, inventory, e)
        names = {inventory[k]['slot']: i for k, i in pairs.items()}
        found.append({'shift': e, 'tangent': float(e @ t), 'score': len(pairs), 'matched': names,
                      'target': names.get(target)})
    # most articles explained; then the smallest shift (AMCL's error is small more often than large)
    found.sort(key=lambda f: (-f['score'], abs(f['tangent'])))
    best = found[0]
    if best['target'] is None:
        return {'failed': f'the best shift explains {best["score"]} articles and not the slot {target}',
                'shift': best['shift'].tolist()}
    ambiguous = any(f['score'] == best['score'] and np.linalg.norm(f['shift'] - best['shift']) > MATCH_RADIUS
                    and f['target'] is not None and f['target'] != best['target'] for f in found[1:])
    return {'shift': best['shift'].tolist(), 'tangent': best['tangent'], 'score': best['score'],
            'target': best['target'], 'matched': best['matched'], 'ambiguous': bool(ambiguous)}


def match_grid(detections, slots, shift):
    """Like `match`, without the class: {slot index in `slots`: detection index}, each detection to the nearest slot within
    MATCH_RADIUS, and a slot keeps only the nearest detection."""
    best = {}
    for i, d in enumerate(detections):
        near, near_d = None, MATCH_RADIUS
        for k, item in enumerate(slots):
            dist = math.hypot(d['x'] - item['x'] - shift[0], d['y'] - item['y'] - shift[1])
            if dist <= near_d:
                near, near_d = k, dist
        if near is not None and (near not in best or near_d < best[near][1]):
            best[near] = (i, near_d)
    return {k: v[0] for k, v in best.items()}


HALF_PITCH = 0.10       # [m] without the classes a shift is told from the same shift plus one pitch (0.2 m) only by the slots at the ends of the view:
                        # the method assumes the shift is below half a pitch


def localize_images(detections, slots, target, expected_label, normal, max_shift=HALF_PITCH):
    """The shift of the believed map, the detection of the slot `target` and the inventory, all from the image: the class that
    stands in each slot is not given, it is read.

    detections: dicts with label, x, y (as for `localize`). slots: dicts with slot, x, y, the slots of the face and level
    (the nominal position, nothing about what stands there). target: the slot to pick; expected_label: the class it should hold
    (the task says what to pick, the rest of the stock is not needed) or None. normal: unit vector (x, y) out of the face.
    A shift is a candidate when it puts a detection on a slot (within the limits of `localize`); the one that puts the most
    detections on slots, with the target slot taken by a detection of the expected class, is chosen, then the smallest shift.
    The articles are spaced as the slots are, so a shift is told from the same shift plus one pitch only by the articles at
    the ends of the view: shifts beyond `max_shift` (half a pitch) are not looked for, and within it the smallest decides.
    Returns the same as `localize` and 'inventory': {slot: label} of the slots that a detection took, or {'failed': reason}."""
    n = np.asarray(normal, dtype=float)
    t = np.array([-n[1], n[0]])
    guesses = []
    for d in detections:
        for item in slots:
            e = np.array([d['x'] - item['x'], d['y'] - item['y']])
            if abs(e @ n) <= NORMAL_TOL and abs(e @ t) <= max_shift:
                guesses.append(e)
    if not guesses:
        return {'failed': f'{len(detections)} articles seen, none fits a slot within the limits of the shift'}
    found = []
    for e in guesses:
        pairs = match_grid(detections, slots, e)
        for _ in range(2):
            if not pairs:
                break
            e = np.mean([[detections[i]['x'] - slots[k]['x'], detections[i]['y'] - slots[k]['y']] for k, i in pairs.items()], axis=0)
            pairs = match_grid(detections, slots, e)
        if not pairs:
            continue
        names = {slots[k]['slot']: i for k, i in pairs.items()}
        found.append({'shift': e, 'tangent': float(e @ t), 'score': len(pairs), 'matched': names, 'target': names.get(target)})
    if not found:
        return {'failed': 'no shift puts a detection on a slot'}
    ok = [f for f in found if f['target'] is not None and (expected_label is None or detections[f['target']]['label'] == expected_label)]
    if not ok:
        best = max(found, key=lambda f: f['score'])
        return {'failed': f'no shift puts an article of the expected class on the slot {target}', 'shift': best['shift'].tolist()}
    ok.sort(key=lambda f: (-f['score'], abs(f['tangent'])))
    best = ok[0]
    ambiguous = any(f['score'] == best['score'] and np.linalg.norm(f['shift'] - best['shift']) > MATCH_RADIUS
                    and f['target'] != best['target'] for f in ok[1:])
    return {'shift': best['shift'].tolist(), 'tangent': best['tangent'], 'score': best['score'], 'target': best['target'],
            'matched': best['matched'], 'ambiguous': bool(ambiguous),
            'inventory': {s: detections[i]['label'] for s, i in best['matched'].items()}}
