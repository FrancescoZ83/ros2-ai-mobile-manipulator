"""What the two dataset generators share: reading the cameras, filling the shelves, splitting, saving, statistics."""
import concurrent.futures as cf
import glob
import hashlib
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

import stocker

SPLITS = (('train', 70), ('val', 15), ('test', 15))       # percent of the groups (look poses or arrangements)
YAW_JITTER = math.radians(15)                              # the same as the stock of the warehouse
XY_JITTER = 0.01                                           # [m]


class Capture(Node):
    """The left image and the labels of the segmentation camera, and the pose of the camera in the map."""

    def __init__(self):
        super().__init__('dataset_capture', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()
        self.frames = {}
        for key, topic in (('image', '/stereo/left/image'), ('labels', '/segmentation/labels_map')):
            self.create_subscription(Image, topic, lambda m, k=key: self.on_image(k, m), qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

    def on_image(self, key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.frames.setdefault(stamp, {})[key] = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        for old in sorted(self.frames)[:-20]:
            del self.frames[old]

    def grab(self, settle=1.0, timeout=20.0):
        """(image, labels, stamp) of a frame taken after the scene has settled, or None.

        This node spins only when asked, so the frames queued while it did not are old: it first spins for `settle`
        seconds (they are read and dropped), takes the newest stamp seen as the reference, and accepts only a frame
        with a later stamp, which was rendered after the articles had been created."""
        end = time.monotonic() + settle
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        reference = max(self.frames) if self.frames else (0, 0)
        reference = reference[0] + reference[1] * 1e-9
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for stamp in sorted(self.frames, reverse=True):
                frame = self.frames[stamp]
                if len(frame) == 2 and stamp[0] + stamp[1] * 1e-9 > reference + 0.05:
                    return frame['image'], frame['labels'][:, :, 0], stamp
        return None

    def camera_pose(self, stamp, timeout=10.0):
        """Position and quaternion (x, y, z, w) of the optical frame of the left camera in the map, at the time of a frame."""
        when = rclpy.time.Time(seconds=stamp[0], nanoseconds=stamp[1], clock_type=rclpy.clock.ClockType.ROS_TIME)
        end = time.monotonic() + timeout       # this node spins only when asked: let the transforms come in
        while time.monotonic() < end and not self.tf_buffer.can_transform('map', 'stereo_left_optical_frame', when):
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self.tf_buffer.lookup_transform('map', 'stereo_left_optical_frame', when).transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])


class Stock:
    """The articles of the shelves: removes the original ones, then fills the shelves at random.

    What is in the world is read from its scene, never taken from the answers of the services (see stocker.call):
    after every change the scene is checked and what is wrong is done again, and if it cannot be put right a
    WorldError is raised, because every image taken in a world that is not the one described by its metadata is wrong."""

    PREFIX = 'ds_'             # the names of the articles created here

    def __init__(self, sem, classes, rng):
        self.sem, self.classes, self.rng = sem, classes, rng
        self.count = 0

    def run(self, jobs):
        with cf.ThreadPoolExecutor(8) as pool:
            return list(pool.map(lambda job: job[0](*job[1:]), jobs))

    def remove_present(self, prefix, only=None, tries=6):
        """Remove the models that start with prefix (and, if `only`, are in it) until the scene shows none of them."""
        for _ in range(tries):
            present = stocker.models(prefix)
            if only is not None:
                present &= only
            if not present:
                return
            self.run([(stocker.remove, n) for n in sorted(present)])
            time.sleep(0.5)
        raise stocker.WorldError(f'{len(present)} models of {prefix}* are still in the world')

    def clear_original(self):
        names = {s['model'] for s in self.sem['slots'].values() if s['model']}
        self.remove_present('art_', only=names)
        print(f'removed the {len(names)} articles of the warehouse (checked in the scene)', flush=True)

    def clear_current(self):
        self.remove_present(self.PREFIX)

    def verify(self, records):
        """The world holds exactly the articles of `records`."""
        present = stocker.models(self.PREFIX)
        wanted = {r['name'] for r in records}
        if present != wanted:
            raise stocker.WorldError(f'the world has {len(present - wanted)} articles too many and {len(wanted - present)} missing')

    def refill(self, unit, x_cam, half_width):
        """Remove the last filling and fill the slots of the row of `unit` near x_cam; the records of the new articles."""
        self.clear_current()
        fill = self.rng.uniform(0.2, 0.9)
        records, jobs = [], []
        for name, slot in self.sem['slots'].items():
            if slot['shelf'][0] != unit[0] or abs(slot['pose'][0] - x_cam) > half_width or self.rng.random() > fill:
                continue
            cls = self.rng.choice(self.classes)
            self.count += 1
            x = slot['pose'][0] + self.rng.uniform(-XY_JITTER, XY_JITTER)
            y = slot['pose'][1] + self.rng.uniform(-XY_JITTER, XY_JITTER)
            yaw = self.rng.uniform(-YAW_JITTER, YAW_JITTER)
            article = f"{self.PREFIX}{self.count}_{name.replace('.', '_')}"
            jobs.append((stocker.create, article, cls['name'], x, y, slot['pose'][2], yaw))
            records.append({'name': article, 'class': cls['name'], 'label': cls['label'], 'slot': name,
                            'pose_map': [round(x, 4), round(y, 4), slot['pose'][2], round(yaw, 4)]})
        self.run(jobs)
        for _ in range(3):                  # create again what the scene does not show
            time.sleep(0.5)
            missing = {r['name'] for r in records} - stocker.models(self.PREFIX)
            if not missing:
                break
            self.run([job for job in jobs if job[1] in missing])
        self.verify(records)
        return records, fill


def split_of(group_id):
    """train, val or test, decided by the group (look pose or arrangement): all the images of a group go to the same split."""
    bucket = int(hashlib.md5(group_id.encode()).hexdigest(), 16) % 100
    edge = 0
    for name, percent in SPLITS:
        edge += percent
        if bucket < edge:
            return name


def pixel_counts(labels):
    values, counts = np.unique(labels, return_counts=True)
    return {int(v): int(c) for v, c in zip(values, counts)}


def save(out, split, index, image, labels, meta):
    base = os.path.join(out, split)
    for sub in ('images', 'labels', 'meta'):
        os.makedirs(os.path.join(base, sub), exist_ok=True)
    cv2.imwrite(os.path.join(base, 'images', f'{index:06d}.png'), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.imwrite(os.path.join(base, 'labels', f'{index:06d}.png'), labels.astype(np.uint8))
    with open(os.path.join(base, 'meta', f'{index:06d}.json'), 'w') as f:
        json.dump(meta, f)


def next_index(out):
    """The number after the highest image number already in the dataset `out` (0 if it is empty)."""
    stems = [int(os.path.basename(f)[:-4]) for f in glob.glob(os.path.join(out, '*', 'images', '*.png'))]
    return max(stems) + 1 if stems else 0


class Stats:
    """Images, pixels per label and images per label, split by split."""

    def __init__(self):
        self.splits = {s: {'images': 0, 'groups': set(), 'pixels': {}, 'present': {}} for s, _ in SPLITS}

    def add(self, split, group_id, pixels):
        st = self.splits[split]
        st['images'] += 1
        st['groups'].add(group_id)
        for label, count in pixels.items():
            st['pixels'][label] = st['pixels'].get(label, 0) + count
            st['present'][label] = st['present'].get(label, 0) + 1

    def write(self, out, seed, first_index, failures, seconds):
        """stats_seed<seed>.json in `out`; the number of images written by this run."""
        summary = {s: {'images': v['images'], 'groups': len(v['groups']), 'pixels': v['pixels'], 'images_with_class': v['present']}
                   for s, v in self.splits.items()}
        images = sum(v['images'] for v in self.splits.values())
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, f'stats_seed{seed}.json'), 'w') as f:
            json.dump({'seed': seed, 'first_index': first_index, 'images': images, 'failures': failures,
                       'seconds': round(seconds), 'splits': summary}, f, indent=1)
        return images
