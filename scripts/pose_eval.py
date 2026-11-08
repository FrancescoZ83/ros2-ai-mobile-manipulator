#!/usr/bin/env python3
"""How well can the pose of an article be found from the images? Measured against the true pose of the simulation.

Needs `ros2 launch amr_manipulator_gazebo dataset.launch.py`, the stereo node (`ros2 run amr_manipulator_perception
stereo_depth_node.py`) and a trained network (`runs/<name>.pt`); `pose_eval.sh` starts them. As in make_dataset_free.py, the
shelves of a row are filled at random and the camera looks at them from poses like those of the arm. For every article
of the picking face that is visible in the image, the pose (x, y of the centre in the map, yaw of a box) is estimated in
three ways, to tell where the error comes from:
  A. depth of the simulator, true mask (that depth has a noise of its own, 10 mm: the fit is exact on exact images, see
     check_geometry.py, so this is the fit on noisy points);
  B. stereo depth, true mask: the fit plus the stereo error;
  C. stereo depth, mask of the network: the whole chain.
The articles are matched to the pixels by projecting their true centre into the image (only the evaluation uses it).
"""
import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np
import rclpy
import torch
import yaml
from scipy import ndimage
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', 'amr_manipulator_perception')
sys.path.insert(0, os.path.join(ROOT, 'segmentation'))
sys.path.insert(0, os.path.join(ROOT, 'pose'))

import geometry as geo                                            # noqa: E402
import make_dataset_free as free                                  # noqa: E402
import semantic_goals as sg                                       # noqa: E402
import stocker                                                    # noqa: E402
from dataset_common import Capture, Stock                         # noqa: E402
from model import UNet                                            # noqa: E402
from rclpy.qos import qos_profile_sensor_data                     # noqa: E402

MIN_PIXELS = 120            # an article with fewer pixels in its mask is not tried
MIN_POINTS = 30


class PoseCapture(Capture):
    """The left image and labels of Capture, and the stereo and the true depth of the same view."""

    def __init__(self):
        super().__init__()
        self.depths = {}
        for kind, topic in (('stereo', '/stereo/depth'), ('truth', '/stereo/left/depth_image')):
            self.create_subscription(Image, topic, lambda m, k=kind: self.on_depth(k, m), qos_profile_sensor_data)
        self.trigger = self.create_client(Trigger, '/stereo_depth/compute')

    def on_depth(self, kind, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.depths[(kind, stamp)] = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float64)
        for old in sorted(self.depths, key=lambda k: k[1])[:-10]:
            del self.depths[old]

    def depth(self, kind, stamp, timeout=15.0):
        """Depth image of `kind` ('truth': same stamp as the image; 'stereo': computed on demand, stamp not before it)."""
        if kind == 'stereo':
            rclpy.spin_until_future_complete(self, self.trigger.call_async(Trigger.Request()), timeout_sec=10.0)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for (k, s), d in self.depths.items():
                if k == kind and (s == stamp if kind == 'truth' else s >= stamp):
                    return d
        return None


def predict(model, image, device):
    x = torch.from_numpy(image.astype(np.float32) / 255.0).permute(2, 0, 1)[None].to(device)
    with torch.no_grad():
        return model(x).argmax(1)[0].cpu().numpy()


def component_at(mask, u, v, reach=6):
    """The connected component of `mask` that holds pixel (u, v), or the nearest one within `reach` pixels."""
    comps, n = ndimage.label(mask)
    if n == 0:
        return None
    h, w = mask.shape
    u0, v0 = int(round(u)), int(round(v))
    best, best_d = None, reach + 1
    for dv in range(-reach, reach + 1):
        for du in range(-reach, reach + 1):
            uu, vv = u0 + du, v0 + dv
            if 0 <= uu < w and 0 <= vv < h and comps[vv, uu] > 0 and math.hypot(du, dv) < best_d:
                best, best_d = comps[vv, uu], math.hypot(du, dv)
    return None if best is None else comps == best


def view_rows(records, face_slots, by_name, normal, cam_pos, cam_quat, sources, intr, ids):
    """One row per article of the picking face whose centre is in the image: the true pose and the estimates.

    records: the articles created (class, slot, pose_map); face_slots: the slots of the picking face (the articles of the
    other face are hidden in the image); sources: {variant: (depth image, label image)}; the pose of the optical frame of
    the camera in the map is (cam_pos, cam_quat). Each article is matched with its pixels by projecting its true centre."""
    fx, fy, cx, cy = intr
    rot = geo.quat_to_matrix(cam_quat)
    rows = []
    for rec in records:
        if rec['slot'] not in face_slots:
            continue
        cls = by_name[rec['class']]
        height = cls['size'][2] if cls['shape'] == 'box' else cls['size'][1]
        ax, ay, az, ayaw = rec['pose_map']
        p = rot.T @ (np.array([ax, ay, az + height / 2.0]) - cam_pos)
        if p[2] < 0.1:
            continue
        u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
        if not (0 <= u < 320 and 0 <= v < 240):
            continue
        row = dict(ids, **{'class': rec['class'], 'shape': cls['shape'], 'truth': [ax, ay, ayaw],
                           'dist': float(np.linalg.norm(p)), 'normal': list(normal),
                           'nominal': [face_slots[rec['slot']]['pose'][0], face_slots[rec['slot']]['pose'][1]]})
        for variant, (depth, labs) in sources.items():
            mask = component_at(labs == rec['label'], u, v)
            if mask is None or mask.sum() < MIN_PIXELS:
                row[variant] = None
                continue
            est = geo.estimate_pose(depth, mask, intr, cam_pos, cam_quat, cls['shape'], cls['size'], MIN_POINTS)
            row[variant] = None if est is None else list(est) + [int(mask.sum())]
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='base', help='network: segmentation/runs/<name>.pt')
    ap.add_argument('--out', required=True, help='JSON file with every estimate')
    ap.add_argument('--arrangements', type=int, default=30)
    ap.add_argument('--shots', type=int, default=5)
    ap.add_argument('--dist', type=float, nargs=2, default=[0.23, 0.53])
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--dump', help='save the images and depths of the first view in this .npz')
    ap.add_argument('--only-class', help='fill the shelves with this class only (more estimates of one class)')
    ap.add_argument('--map', default=os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
    ap.add_argument('--articles', default=os.path.join(sg.GAZEBO, 'config', 'articles.yaml'))
    args = ap.parse_args()

    sem = sg.load(args.map)
    classes = yaml.safe_load(open(args.articles))['articles']
    by_name = {c['name']: c for c in classes}
    rng = random.Random(args.seed)
    picking = {}
    for name, slot in sem['slots'].items():
        if slot['pickable']:
            picking.setdefault(slot['shelf'][0], []).append((name, slot))
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    ck = torch.load(os.path.join(ROOT, 'segmentation', 'runs', args.name + '.pt'), map_location=device, weights_only=True)
    model = UNet(3, 7, 16).to(device)
    model.load_state_dict(ck['model'])
    model.eval()
    intr = geo.intrinsics()

    rclpy.init()
    capture = PoseCapture()
    stock = Stock(sem, [c for c in classes if not args.only_class or c['name'] == args.only_class], rng)
    results, shots, t0 = [], 0, time.monotonic()
    try:
        if free.CAMERA not in stocker.models(free.CAMERA):
            raise SystemExit(f'there is no {free.CAMERA} in the world')
        stock.clear_original()
        for k in range(args.arrangements):
            row = rng.choice(sorted(picking))
            centre = rng.choice(picking[row])[1]['pose'][0]
            records, _ = stock.refill(row, centre, 0.7)
            targets = [(n, s) for n, s in picking[row] if abs(s['pose'][0] - centre) <= 0.3]
            for shot in range(args.shots):
                slot_name, slot = rng.choice(targets)
                x, y, z, roll, pitch, yaw = free.camera_pose(slot, rng.uniform(*args.dist), rng)
                q = free.quaternion(roll, pitch, yaw)
                if not free.move_camera(x, y, z, q):
                    continue
                grabbed = capture.grab(settle=1.0 if shot == 0 else 0.3)
                if grabbed is None:
                    continue
                image, labels, stamp = grabbed
                truth, stereo = capture.depth('truth', stamp), capture.depth('stereo', stamp)
                if truth is None or stereo is None:
                    continue
                cam_pos, cam_quat = geo.optical_pose_from_link((x, y, z), q)
                if args.dump and shots == 0:
                    np.savez(args.dump, image=image, labels=labels, truth=truth, stereo=stereo, cam_pos=cam_pos,
                             cam_quat=cam_quat, records=json.dumps(records))
                net_labels = predict(model, image, device)
                shots += 1
                results += view_rows(records, dict(picking[row]), by_name, sg.NORMALS[slot['face']], cam_pos, cam_quat,
                                     {'A': (truth, labels), 'B': (stereo, labels), 'C': (stereo, net_labels)}, intr,
                                     {'arrangement': k, 'shot': shot})
                json.dump(results, open(args.out, 'w'))
            print(f'arrangement {k + 1}/{args.arrangements}: {shots} views, {len(results)} articles tried, '
                  f'{time.monotonic() - t0:.0f} s', flush=True)
    finally:
        try:
            stock.clear_current()
        except stocker.WorldError as err:
            print('the articles could not be removed:', err)
        json.dump(results, open(args.out, 'w'))
        capture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
