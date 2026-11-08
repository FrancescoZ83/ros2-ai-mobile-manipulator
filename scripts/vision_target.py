"""The pose of the article to pick, from the images of the camera on the arm: network, stereo depth, shape model.

The arm is at a look pose in front of the slot. The left image goes to the network; the mask of the class of the article
that is nearest to where the slot is projected is back-projected with the stereo depth, and the shape of the class is
fitted to the points (amr_manipulator_perception/pose/geometry.py). What the prior (the nominal pose of the slot) does
is to choose which of the articles of that class in the image is the one; the pose is only from the images and from the
transforms of the arm. The camera is taken to the world with the pose of the robot from the simulator, and the caller
takes the estimate back to the robot with the same pose, so that the error of a localization does not enter.
"""
import math
import os
import sys

import numpy as np
import torch
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pose_eval import PoseCapture, ROOT, component_at, geo, predict   # noqa: E402
from model import UNet                                                # noqa: E402

MIN_FRACTION = 0.3        # fraction of the pixels of the mask with a stereo depth: less, and there are too few points
PRIOR_RADIUS = 0.03       # [m] the article is in its slot within 1.4 cm (the jitter of the stock): an estimate farther than this
                          # from the nominal slot is a wrong depth of the stereo. Measured on the soup can (172 estimates:
                          # 15 wrong, none left) and on all the classes (258: 10 wrong, none left), better than a limit on
                          # the fraction of the mask with a depth, which does not tell the good from the wrong.
MIN_PIXELS = 120


class Vision:
    def __init__(self, network='base', prior_radius=PRIOR_RADIUS):
        self.prior_radius = prior_radius      # [m] see PRIOR_RADIUS; with AMCL the localization error adds to it
        self.capture = PoseCapture()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        ck = torch.load(os.path.join(ROOT, 'segmentation', 'runs', network + '.pt'), map_location=self.device, weights_only=True)
        self.model = UNet(3, 7, 16).to(self.device)
        self.model.load_state_dict(ck['model'])
        self.model.eval()
        self.intr = geo.intrinsics()

    def detect(self, classes_by_label, robot):
        """Every article in the image, with its pose in the map the caller believes (see localization.py).

        classes_by_label: {label: entry of articles.yaml}. robot: (x, y, z, yaw) of the robot in that map. A mask that
        touches the border of the image is cut and gives a wrong centre: it is counted and not used. Returns
        (detections, info): the detections are dicts with label, x, y, yaw, points, mask_pixels, fraction; info says
        how many masks were found, cut, too small or without enough depth, or {'failed': reason}."""
        grabbed = self.capture.grab(settle=1.0)
        if grabbed is None:
            return [], {'failed': 'no image'}
        image, _, stamp = grabbed
        stereo = self.capture.depth('stereo', stamp)
        if stereo is None:
            return [], {'failed': 'no stereo depth'}
        labels = predict(self.model, image, self.device)
        pos_b, quat_b = self.capture.camera_pose(stamp, frame='base_footprint')
        x, y, z, yaw = robot
        c, s = math.cos(yaw), math.sin(yaw)
        r_robot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = r_robot @ np.asarray(pos_b) + np.array([x, y, z])
        cam_quat = geo.matrix_to_quat(r_robot @ geo.quat_to_matrix(quat_b))
        h, w = labels.shape
        detections, info = [], {'masks': 0, 'cut': 0, 'small': 0, 'no_depth': 0}
        for label, cls in classes_by_label.items():
            comps, n = ndimage.label(labels == label)
            for k in range(1, n + 1):
                mask = comps == k
                pixels = int(mask.sum())
                if pixels < MIN_PIXELS:
                    info['small'] += 1
                    continue
                info['masks'] += 1
                vs, us = np.nonzero(mask)
                if us.min() == 0 or vs.min() == 0 or us.max() == w - 1 or vs.max() == h - 1:
                    info['cut'] += 1
                    continue
                est = geo.estimate_pose(stereo, mask, self.intr, cam_pos, cam_quat, cls['shape'], cls['size'])
                if est is None or est[3] / pixels < MIN_FRACTION:
                    info['no_depth'] += 1
                    continue
                detections.append({'label': label, 'class': cls['name'], 'x': est[0], 'y': est[1], 'yaw': est[2],
                                   'points': est[3], 'mask_pixels': pixels, 'fraction': round(float(est[3] / pixels), 2)})
        return detections, info

    def estimate(self, cls, expected_centre, robot):
        """Pose of the article of class `cls` (an entry of articles.yaml) expected at `expected_centre` (world), seen now.

        robot: (x, y, z, yaw) of the robot in the world. Returns a dict with x, y (world), yaw, and what the estimate
        is made of, or {'failed': reason}."""
        grabbed = self.capture.grab(settle=1.0)
        if grabbed is None:
            return {'failed': 'no image'}
        image, _, stamp = grabbed
        stereo = self.capture.depth('stereo', stamp)
        if stereo is None:
            return {'failed': 'no stereo depth'}
        labels = predict(self.model, image, self.device)
        pos_b, quat_b = self.capture.camera_pose(stamp, frame='base_footprint')
        x, y, z, yaw = robot
        c, s = math.cos(yaw), math.sin(yaw)
        r_robot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = r_robot @ np.asarray(pos_b) + np.array([x, y, z])
        cam_quat = geo.matrix_to_quat(r_robot @ geo.quat_to_matrix(quat_b))
        p = geo.quat_to_matrix(cam_quat).T @ (np.asarray(expected_centre) - cam_pos)
        if p[2] < 0.1:
            return {'failed': 'the slot is behind the camera'}
        fx, fy, cx, cy = self.intr
        u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
        mask = component_at(labels == cls['label'], u, v, reach=20)
        if mask is None or mask.sum() < MIN_PIXELS:
            return {'failed': 'no mask of the class near the slot', 'pixels_of_class': int((labels == cls['label']).sum())}
        est = geo.estimate_pose(stereo, mask, self.intr, cam_pos, cam_quat, cls['shape'], cls['size'])
        if est is None:
            return {'failed': 'too few points with a depth', 'mask_pixels': int(mask.sum())}
        fraction = est[3] / mask.sum()
        prior = math.hypot(est[0] - expected_centre[0], est[1] - expected_centre[1])
        out = {'x': est[0], 'y': est[1], 'yaw': est[2], 'points': est[3], 'mask_pixels': int(mask.sum()),
               'fraction': round(float(fraction), 2), 'camera_distance': round(float(np.linalg.norm(p)), 3),
               'prior_distance': round(float(prior), 3)}
        if prior > self.prior_radius:
            out['failed'] = f'the estimate is {100 * prior:.1f} cm from the nominal slot'
        elif fraction < MIN_FRACTION:
            out['failed'] = f'only {100 * fraction:.0f} percent of the mask has a depth'
        return out
