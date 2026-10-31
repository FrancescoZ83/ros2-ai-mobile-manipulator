"""Stock of the warehouse: slots on the shelves, random filling, <include> elements, semantic map."""
import math
import random

import yaml

FACE_SIGN = {'north': 1.0, 'south': -1.0}   # sign of the y offset of each long face of a shelf
DROP_GAP = 0.002                            # dynamic articles start this high above the surface [m]


class Flow(dict):
    """A mapping that YAML writes on a single line."""


yaml.SafeDumper.add_representer(
    Flow, lambda dumper, data: dumper.represent_mapping('tag:yaml.org,2002:map', data, flow_style=True))


def shelf_units(layout):
    """(unit name, row name, centre x, centre y) of every shelf unit, e.g. ('S1', 'S', -1.8, -1.3)."""
    w = layout['shelf']['unit'][0]
    return [(f"{row['name']}{i + 1}", row['name'], row['x_start'] + w * (i + 0.5), row['y'])
            for row in layout['shelf']['rows'] for i in range(row['units'])]


def slot_grid(layout):
    """All slots of all shelves. The id reads shelf.level.face+index, e.g. S1.L2.n3."""
    shelf, stock = layout['shelf'], layout['stock']
    w = shelf['unit'][0]
    n = int((w - 2 * shelf['upright_size']) // stock['slot_pitch'])   # slots per face and level
    slots = []
    for name, row, cx, cy in shelf_units(layout):
        active = stock['active_faces'][row]
        for level, z in enumerate(shelf['levels'], start=1):
            for face, sign in FACE_SIGN.items():
                for k in range(n):
                    sid = f'{name}.L{level}.{face[0]}{k + 1}'
                    slots.append({
                        'id': sid, 'model': 'art_' + sid.replace('.', '_'),
                        'shelf': name, 'level': level, 'face': face,
                        'x': cx + (k - (n - 1) / 2.0) * stock['slot_pitch'],
                        'y': cy + sign * stock['face_inset'],
                        'z': z, 'yaw': 0.0,
                        'pickable': face == active, 'dynamic': face == active, 'article': None,
                    })
    return slots


def fill_slots(slots, article_names, layout, fill=None, seed=None):
    """Put a random article in a random subset of the slots (modifies the slots in place)."""
    stock = layout['stock']
    fill = stock['fill'] if fill is None else fill
    rng = random.Random(stock['seed'] if seed is None else seed)
    jitter, yaw_max = stock['position_jitter'], math.radians(stock['yaw_jitter_deg'])
    for s in slots:
        if rng.random() < fill:
            s['article'] = rng.choice(article_names)
            s['x'] += rng.uniform(-jitter, jitter)
            s['y'] += rng.uniform(-jitter, jitter)
            s['yaw'] = rng.uniform(-yaw_max, yaw_max)
    # Every class must be available to the robot, otherwise some orders cannot be served
    pickable = {s['article'] for s in slots if s['article'] and s['pickable']}
    missing = sorted(set(article_names) - pickable)
    if missing:
        raise ValueError(f'no pickable article of class {missing}: raise stock.fill or change the seed')
    return slots


def include_xml(slot):
    """<include> of one article. Articles on the background faces are static."""
    z = slot['z'] + (DROP_GAP if slot['dynamic'] else 0.0)
    static = '' if slot['dynamic'] else '\n      <static>true</static>'
    return (f"    <include>\n"
            f"      <name>{slot['model']}</name>\n"
            f"      <uri>model://{slot['article']}</uri>\n"
            f"      <pose>{slot['x']:.4f} {slot['y']:.4f} {z:.4f} 0 0 {slot['yaw']:.4f}</pose>{static}\n"
            f"    </include>")


def zone_of(layout, x):
    for z in layout['zones']:
        if z['x'][0] <= x <= z['x'][1]:
            return z['name']
    raise ValueError(f'x = {x} is outside every zone')


def semantic_map(layout, slots):
    """Everything the robot has to know about the building, as plain data for YAML."""
    r = lambda v: round(float(v), 4)
    sm = {'frame': 'world',
          'zones': {z['name']: {'purpose': z['purpose'], 'x': z['x'], 'y': [-layout['building']['size'][1] / 2,
                                                                       layout['building']['size'][1] / 2]}
                    for z in layout['zones']}}
    doors, n = {}, 0
    for part in layout['partitions']:
        for c, w in part['doors']:
            n += 1
            doors[f'P{n}'] = {'pose': [r(part['x']), r(c)], 'width': r(w),
                              'connects': [zone_of(layout, part['x'] - 0.01), zone_of(layout, part['x'] + 0.01)]}
    sm['doors'] = doors
    w, d = layout['shelf']['unit']
    sm['shelves'] = {name: {'pose': [r(cx), r(cy)], 'size': [r(w), r(d)], 'zone': zone_of(layout, cx),
                            'levels': list(layout['shelf']['levels'])}
                     for name, _, cx, cy in shelf_units(layout)}
    heights = {t['name']: t['height'] for t in layout['tables']}
    sm['tables'] = {t['name']: {'pose': [r(t['centre'][0]), r(t['centre'][1])], 'size': t['size'],
                                'height': t['height'], 'zone': zone_of(layout, t['centre'][0])}
                    for t in layout['tables']}
    sm['bins'] = {b['name']: {'pose': [r(b['centre'][0]), r(b['centre'][1]), r(heights[b['table']])],
                              'size': b['size'], 'table': b['table'],
                              'zone': zone_of(layout, b['centre'][0])}
                  for b in layout['bins']}
    per_class = {}
    for s in slots:
        if s['article']:
            per_class[s['article']] = per_class.get(s['article'], 0) + 1
    sm['stock'] = {'fill': layout['stock']['fill'], 'seed': layout['stock']['seed'],
                   'slots': len(slots), 'articles': sum(per_class.values()),
                   'per_class': dict(sorted(per_class.items()))}
    sm['slots'] = {s['id']: Flow(shelf=s['shelf'], level=s['level'], face=s['face'],
                                 pose=[r(s['x']), r(s['y']), r(s['z'])], yaw=r(s['yaw']),
                                 article=s['article'], pickable=s['pickable'], dynamic=s['dynamic'],
                                 model=s['model'] if s['article'] else None)
                   for s in slots}
    return sm
