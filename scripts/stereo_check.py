#!/usr/bin/env python3
"""Compare the stereo depth with the true depth of the left camera (Gazebo), pixel by pixel."""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

BINS = [(0.10, 0.20), (0.20, 0.30), (0.30, 0.50), (0.50, 1.00), (1.00, 3.00)]
TEXTURE = 6.0         # [grey levels] local deviation above which a pixel counts as textured


class Check(Node):
    def __init__(self):
        super().__init__('stereo_check')
        self.bridge = CvBridge()
        self.frames = {}                     # stamp -> {'stereo': depth, 'truth': depth, 'left': image}
        for key, topic, enc in (('stereo', '/stereo/depth', 'passthrough'), ('truth', '/stereo/left/depth_image', 'passthrough'),
                                ('left', '/stereo/left/image', 'rgb8')):
            self.create_subscription(Image, topic, lambda m, k=key, e=enc: self.on_image(k, e, m), qos_profile_sensor_data)

    def on_image(self, key, encoding, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.frames.setdefault(stamp, {})[key] = self.bridge.imgmsg_to_cv2(msg, encoding)

    def complete(self):
        return [f for f in self.frames.values() if len(f) == 3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=5, help='number of frame triplets to analyse')
    ap.add_argument('--timeout', type=float, default=60.0, help='wall-clock seconds to wait')
    ap.add_argument('--save', help='directory for PNGs of the last triplet (left image, true depth, stereo depth, error)')
    args = ap.parse_args()

    rclpy.init()
    node = Check()
    # The stereo node computes on demand: ask for one depth image per frame (a node running at a rate just ignores it)
    trigger = node.create_client(Trigger, '/stereo_depth/compute')
    on_demand = trigger.wait_for_service(timeout_sec=3.0)
    end = time.monotonic() + args.timeout
    while rclpy.ok() and len(node.complete()) < args.frames and time.monotonic() < end:
        if on_demand:
            rclpy.spin_until_future_complete(node, trigger.call_async(Trigger.Request()), timeout_sec=10.0)
            for _ in range(8):          # about 0.8 s: the cameras give a new pair every 0.1 s of simulation
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
    done = node.complete()
    if not done:
        raise SystemExit('no complete frame: are the simulation (stereo:=true) and stereo_depth_node running?')
    done = done[:args.frames]

    truth = np.concatenate([f['truth'][np.isfinite(f['truth'])] for f in done])
    print(f'{len(done)} frames of {done[0]["truth"].shape[1]}x{done[0]["truth"].shape[0]}; true depth {truth.min():.2f} .. {truth.max():.2f} m')
    stacks = {k: np.stack([f[k] for f in done]) for k in ('truth', 'stereo')}
    t, s = stacks['truth'].astype(np.float64), stacks['stereo'].astype(np.float64)
    # Texture: local standard deviation of the grey left image above the noise of the sensor (about 2.5 grey levels)
    textured = []
    for f in done:
        g = cv2.cvtColor(f['left'], cv2.COLOR_RGB2GRAY).astype(np.float32)
        textured.append(np.sqrt(np.maximum(cv2.blur(g * g, (5, 5)) - cv2.blur(g, (5, 5)) ** 2, 0)) > TEXTURE)
    textured = np.stack(textured)
    for label, mask in (('with texture', textured), ('without texture', ~textured)):
        print(f'\npixels {label} (local deviation {">" if label == "with texture" else "<="} {TEXTURE:.0f} grey levels):')
        print('true depth    pixels   with stereo   bias[mm]   |error| median / 95th [mm]   relative |error| median [%]')
        for lo, hi in BINS:
            inside = mask & np.isfinite(t) & (t >= lo) & (t < hi)
            got = inside & np.isfinite(s)
            n, m = int(inside.sum()), int(got.sum())
            if n == 0:
                continue
            if m == 0:
                print(f'{lo:.2f}-{hi:.2f} m {n:8d}   {0:10.0f} %')
                continue
            err = s[got] - t[got]
            print(f'{lo:.2f}-{hi:.2f} m {n:8d}   {100.0 * m / n:10.0f} %   {1000 * err.mean():+8.1f}   '
                  f'{1000 * np.median(np.abs(err)):7.1f} / {1000 * np.percentile(np.abs(err), 95):7.1f}   '
                  f'{100 * np.median(np.abs(err) / t[got]):8.2f}')
    inside = np.isfinite(t) & (t < 1.0)
    for label, mask in (('with texture', textured), ('without texture', ~textured)):
        sel = inside & mask
        print(f'below 1 m, {label}: stereo depth on {100.0 * (sel & np.isfinite(s)).sum() / max(sel.sum(), 1):.0f} % of {int(sel.sum())} pixels')

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        last = done[-1]
        def color(d, lo=0.1, hi=1.2):
            v = np.nan_to_num(np.clip((d - lo) / (hi - lo), 0, 1), nan=0.0)
            img = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_JET)
            img[~np.isfinite(d)] = 0
            return img
        err = np.abs(last['stereo'] - last['truth'])
        err_img = cv2.applyColorMap((np.nan_to_num(np.clip(err / 0.05, 0, 1), nan=0.0) * 255).astype(np.uint8), cv2.COLORMAP_HOT)
        err_img[~np.isfinite(err)] = 0
        both = np.hstack([cv2.cvtColor(last['left'], cv2.COLOR_RGB2BGR), color(last['truth']), color(last['stereo']), err_img])
        cv2.imwrite(os.path.join(args.save, 'stereo_check.png'), cv2.resize(both, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
        print('saved', os.path.join(args.save, 'stereo_check.png'), '(left image | true depth | stereo depth | error, white = 5 cm)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
