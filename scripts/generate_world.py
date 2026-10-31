#!/usr/bin/env python3
"""Generate the warehouse world: copy a template world and add everything described in layout.yaml."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from warehouse_stock import fill_slots, include_xml, semantic_map, shelf_units, slot_grid

LIDAR_PLANE_Z = 0.275   # world height of the LiDAR scan plane [m]
TABLE_TOP_THICKNESS = 0.04
TABLE_INSET = 0.05      # the cabinet under the top is smaller than the top by this much per side
BIN_WALL = 0.01
TABLE_TOP_COLOR = [0.55, 0.45, 0.32, 1.0]
TABLE_BODY_COLOR = [0.55, 0.58, 0.62, 1.0]
BIN_COLOR = [0.62, 0.72, 0.84, 1.0]
COLUMN_COLOR = [0.48, 0.50, 0.52, 1.0]


def free_intervals(lo, hi, doors):
    """Parts of [lo, hi] that are not doorways. doors is a list of (centre, width)."""
    gaps = sorted((c - w / 2.0, c + w / 2.0) for c, w in doors)
    pos, out = lo, []
    for g0, g1 in gaps:
        # A door must lie inside the wall and must not overlap the previous one
        if g0 < pos - 1e-9 or g1 > hi + 1e-9:
            raise ValueError(f'door [{g0:.2f}, {g1:.2f}] overlaps another door or leaves [{lo}, {hi}]')
        if g0 > pos + 1e-9:
            out.append((pos, g0))
        pos = g1
    if hi > pos + 1e-9:
        out.append((pos, hi))
    return out


def box_link(name, centre, size, color):
    """SDF text of one link made of a single box (collision + visual)."""
    cx, cy, cz = centre
    sx, sy, sz = size
    r, g, b, a = color
    return f'''      <link name="{name}">
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>'''


def wall_links(layout):
    """One link per wall segment: four outer walls, then the partitions split around their doors."""
    hx, hy = (s / 2.0 for s in layout['building']['size'])
    t, h = layout['wall']['thickness'], layout['wall']['height']
    color = layout['wall']['color']
    z = h / 2.0
    links = [
        # North and south walls span the corners too, so west and east walls need no extra length
        box_link('wall_north', (0.0, hy + t / 2.0, z), (2.0 * hx + 2.0 * t, t, h), color),
        box_link('wall_south', (0.0, -hy - t / 2.0, z), (2.0 * hx + 2.0 * t, t, h), color),
        box_link('wall_west', (-hx - t / 2.0, 0.0, z), (t, 2.0 * hy, h), color),
        box_link('wall_east', (hx + t / 2.0, 0.0, z), (t, 2.0 * hy, h), color),
    ]
    for part in layout['partitions']:
        doors = [(c, w) for c, w in part['doors']]
        for i, (y0, y1) in enumerate(free_intervals(-hy, hy, doors)):
            links.append(box_link(f"wall_{part['name']}_{i}", (part['x'], (y0 + y1) / 2.0, z),
                                  (t, y1 - y0, h), color))
    return links


def walls_model(layout):
    links = wall_links(layout)
    text = '    <model name="walls">\n      <static>true</static>\n' + '\n'.join(links) + '\n    </model>'
    return text, len(links)


def static_model(name, xy, parts):
    """One static model with a single link 'body' that holds many boxes.

    xy is the model position on the floor; parts is a list of (part name, centre, size, color)
    with the centre relative to the model origin (x, y, 0).
    """
    elements = []
    for pname, (cx, cy, cz), (sx, sy, sz), (r, g, b, a) in parts:
        geom = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
        pose = f'<pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>'
        elements.append(f'''        <collision name="{pname}_collision">
          {pose}
          {geom}
        </collision>
        <visual name="{pname}_visual">
          {pose}
          {geom}
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>''')
    return (f'    <model name="{name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{xy[0]:.4f} {xy[1]:.4f} 0 0 0 0</pose>\n'
            f'      <link name="body">\n' + '\n'.join(elements) + '\n      </link>\n    </model>')


def shelf_parts(shelf):
    """Boxes of one shelf unit, relative to the centre of its footprint."""
    w, d = shelf['unit']
    levels = shelf['levels']
    bt, u, uh = shelf['board_thickness'], shelf['upright_size'], shelf['upright_height']
    if levels[0] < LIDAR_PLANE_Z + 0.02:
        raise ValueError(f'plinth {levels[0]} m is too low: the LiDAR plane is at {LIDAR_PLANE_Z} m')
    if sorted(levels) != list(levels) or uh <= levels[-1]:
        raise ValueError('levels must be increasing and the uprights must rise above the last level')
    col = shelf['colors']
    # Level 1: solid plinth over the whole footprint, its top surface is the first shelf surface
    parts = [('plinth', (0.0, 0.0, levels[0] / 2.0), (w, d, levels[0]), col['plinth'])]
    # Other levels: boards whose top surface is at the level height
    for i, z in enumerate(levels[1:], start=2):
        parts.append((f'board_{i}', (0.0, 0.0, z - bt / 2.0), (w, d, bt), col['board']))
    # Four corner uprights standing on the plinth
    z0 = levels[0]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append((f'upright_{"e" if sx > 0 else "w"}{"n" if sy > 0 else "s"}',
                          (sx * (w / 2.0 - u / 2.0), sy * (d / 2.0 - u / 2.0), (z0 + uh) / 2.0),
                          (u, u, uh - z0), col['upright']))
    return parts


def shelf_models(layout):
    """One static model per shelf unit, named shelf_<row><index> (shelf_S1, shelf_S2, ..., shelf_N1, ...)."""
    parts = shelf_parts(layout['shelf'])
    return [static_model(f'shelf_{name}', (cx, cy), parts) for name, _, cx, cy in shelf_units(layout)]


def table_models(layout):
    """Cabinet (solid, seen by the LiDAR) plus an overhanging top. Returns (models, height by name)."""
    models, heights = [], {}
    for t in layout['tables']:
        sx, sy = t['size']
        h = t['height']
        body_h = h - TABLE_TOP_THICKNESS
        parts = [
            ('body', (0.0, 0.0, body_h / 2.0), (sx - 2 * TABLE_INSET, sy - 2 * TABLE_INSET, body_h),
             TABLE_BODY_COLOR),
            ('top', (0.0, 0.0, h - TABLE_TOP_THICKNESS / 2.0), (sx, sy, TABLE_TOP_THICKNESS), TABLE_TOP_COLOR),
        ]
        models.append(static_model(t['name'], tuple(t['centre']), parts))
        heights[t['name']] = h
    return models, heights


def bin_models(layout, table_heights):
    """Open box: a floor and four thin walls, standing on a table."""
    models = []
    for b in layout['bins']:
        sx, sy, sz = b['size']
        if b['table'] not in table_heights:
            raise ValueError(f"bin {b['name']} stands on an unknown table '{b['table']}'")
        z0 = table_heights[b['table']]
        wt = BIN_WALL
        parts = [
            ('floor', (0.0, 0.0, z0 + wt / 2.0), (sx, sy, wt), BIN_COLOR),
            ('wall_n', (0.0, sy / 2.0 - wt / 2.0, z0 + sz / 2.0), (sx, wt, sz), BIN_COLOR),
            ('wall_s', (0.0, -sy / 2.0 + wt / 2.0, z0 + sz / 2.0), (sx, wt, sz), BIN_COLOR),
            ('wall_e', (sx / 2.0 - wt / 2.0, 0.0, z0 + sz / 2.0), (wt, sy, sz), BIN_COLOR),
            ('wall_w', (-sx / 2.0 + wt / 2.0, 0.0, z0 + sz / 2.0), (wt, sy, sz), BIN_COLOR),
        ]
        models.append(static_model(b['name'], tuple(b['centre']), parts))
    return models


def column_models(layout):
    h = layout['wall']['height']
    return [static_model(c['name'], tuple(c['centre']),
                         [('shaft', (0.0, 0.0, h / 2.0), (c['size'], c['size'], h), COLUMN_COLOR)])
            for c in layout['columns']]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', required=True, help='layout.yaml')
    ap.add_argument('--template', required=True, help='world file that provides physics, plugins, light, ground')
    ap.add_argument('--out', required=True, help='world file to write')
    ap.add_argument('--articles', help='articles.yaml: if given, the shelves are stocked with these articles')
    ap.add_argument('--semantic-out', help='semantic map to write (needs --articles)')
    ap.add_argument('--fill', type=float, help='override stock.fill of the layout')
    ap.add_argument('--seed', type=int, help='override stock.seed of the layout')
    ap.add_argument('--static-articles', action='store_true',
                    help='make every article static: a cheap scene in which nothing can be picked')
    args = ap.parse_args()
    if args.semantic_out and not args.articles:
        raise SystemExit('--semantic-out needs --articles')

    layout = yaml.safe_load(Path(args.layout).read_text())
    template = Path(args.template).read_text()
    if template.count('</world>') != 1:
        raise SystemExit('the template must contain exactly one </world>')

    walls, n_walls = walls_model(layout)
    shelves = shelf_models(layout)
    tables, heights = table_models(layout)
    bins = bin_models(layout, heights)
    columns = column_models(layout)

    includes, slots = [], []
    if args.articles:
        names = [a['name'] for a in yaml.safe_load(Path(args.articles).read_text())['articles']]
        try:
            slots = fill_slots(slot_grid(layout), names, layout, fill=args.fill, seed=args.seed)
        except ValueError as err:
            raise SystemExit(str(err))
        if args.static_articles:
            for s in slots:
                s['dynamic'] = False
        includes = [include_xml(s) for s in slots if s['article']]

    body = '\n'.join([walls] + shelves + tables + bins + columns + includes) + '\n'
    world = template.replace('</world>', '\n    <!-- Generated from layout.yaml: do not edit by hand -->\n'
                             + body + '\n  </world>')
    root = ET.fromstring(world)          # raises if the result is not well-formed XML
    names = [m.get('name') for m in root.iter('model')] + [i.find('name').text for i in root.iter('include')]
    if len(names) != len(set(names)):
        raise SystemExit(f'duplicate model names: {sorted(n for n in set(names) if names.count(n) > 1)}')
    Path(args.out).write_text(world)
    print(f'wrote {args.out}: {n_walls} wall segments, {len(shelves)} shelf units, '
          f'{len(tables)} tables, {len(bins)} bins, {len(columns)} columns')

    if args.articles:
        n_art = sum(1 for s in slots if s['article'])
        n_dyn = sum(1 for s in slots if s['article'] and s['dynamic'])
        print(f'stock: {len(slots)} slots, {n_art} articles ({n_dyn} dynamic, {n_art - n_dyn} static)')
    if args.semantic_out:
        text = yaml.safe_dump(semantic_map(layout, slots), sort_keys=False, default_flow_style=None, width=200)
        Path(args.semantic_out).write_text('# Generated from the layout: do not edit by hand\n' + text)
        print(f'wrote {args.semantic_out}')


if __name__ == '__main__':
    main()
