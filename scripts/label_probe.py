#!/usr/bin/env python3
"""Look at the labels of the segmentation camera against the left image: encoding, labels present, overlay."""
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

CLASSES = {0: 'background', 1: 'cereal_box', 2: 'juice_carton', 3: 'soup_can', 4: 'water_bottle', 5: 'parcel', 6: 'jam_jar'}


class Probe(Node):
    def __init__(self):
        super().__init__('label_probe')
        self.bridge = CvBridge()
        self.frames = {}
        for key, topic in (('image', '/stereo/left/image'), ('labels', '/segmentation/labels_map')):
            self.create_subscription(Image, topic, lambda m, k=key: self.on_image(k, m), qos_profile_sensor_data)

    def on_image(self, key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        entry = self.frames.setdefault(stamp, {})
        entry[key] = (self.bridge.imgmsg_to_cv2(msg, 'passthrough'), msg.encoding, msg.header.frame_id)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--save', help='directory for the overlay and the label image')
    ap.add_argument('--name', default='label_probe.png', help='file name of the overlay')
    args = ap.parse_args()
    rclpy.init()
    node = Probe()
    end = time.monotonic() + args.timeout
    pair = None
    while rclpy.ok() and pair is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        pair = next((f for f in node.frames.values() if len(f) == 2), None)
    if pair is None:
        raise SystemExit('no image and labels with the same stamp: is the simulation running with stereo:=true dataset:=true?')
    image, _, _ = pair['image']
    labels, encoding, frame = pair['labels']
    print(f'labels: encoding {encoding}, shape {labels.shape}, dtype {labels.dtype}, frame {frame}; image {image.shape}')
    ids = labels if labels.ndim == 2 else labels[:, :, 0]
    values, counts = np.unique(ids, return_counts=True)
    for v, c in zip(values, counts):
        print(f'  label {int(v):3d} {CLASSES.get(int(v), "?"):13s} {c:7d} pixels ({100.0 * c / ids.size:5.1f} %)')
    if args.save:
        os.makedirs(args.save, exist_ok=True)
        colors = np.array([[0, 0, 0], [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180]], np.uint8)
        painted = colors[np.clip(ids.astype(np.int64), 0, len(colors) - 1)]
        overlay = np.where((ids > 0)[..., None], (0.5 * image + 0.5 * painted).astype(np.uint8), image)
        both = np.hstack([cv2.cvtColor(image, cv2.COLOR_RGB2BGR), cv2.cvtColor(painted, cv2.COLOR_RGB2BGR), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)])
        cv2.imwrite(os.path.join(args.save, args.name), cv2.resize(both, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
        print('saved', os.path.join(args.save, args.name), '(image | labels | overlay)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
