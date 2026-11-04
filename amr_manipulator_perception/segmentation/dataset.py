"""Loading of the segmentation dataset: the left image and the class of every pixel.

The whole split is read into memory once (about 0.3 MB per sample), so training does not wait for the disk.
Augmentation is for the training split only: the validation and test splits are always the images as they are.
"""
import glob
import json
import os

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

CLASSES = ['background', 'cereal_box', 'juice_carton', 'soup_can', 'water_bottle', 'parcel', 'jam_jar']
NUM_CLASSES = len(CLASSES)


def load_split(root, split):
    """(images, labels): uint8 arrays N x H x W x 3 (RGB) and N x H x W (class id), sorted by file name."""
    names = sorted(glob.glob(os.path.join(root, split, 'images', '*.png')))
    if not names:
        raise FileNotFoundError(f'no images in {os.path.join(root, split, "images")}')
    images = np.stack([cv2.cvtColor(cv2.imread(n), cv2.COLOR_BGR2RGB) for n in names])
    labels = np.stack([cv2.imread(n.replace('images', 'labels'), cv2.IMREAD_UNCHANGED) for n in names])
    return images, labels


def pixel_frequencies(root, split='train'):
    """Fraction of the pixels of each class in a split, from the statistics written by the generator."""
    counts = np.zeros(NUM_CLASSES)
    for path in glob.glob(os.path.join(root, 'stats_seed*.json')):
        for label, count in json.load(open(path))['splits'][split]['pixels'].items():
            counts[int(label)] += count
    return counts / counts.sum()


def augment(image, label, rng):
    """A random change of a training sample. image: float32 H x W x 3 in [0, 1]; label: uint8 H x W.

    A shift moves image and label together (nearest neighbour for the label, so that no new class appears); the
    other changes touch only the colours: contrast, brightness, a little blur, a little more noise than the camera's."""
    h, w = label.shape
    shift = np.float32([[1, 0, rng.integers(-16, 17)], [0, 1, rng.integers(-12, 13)]])
    image = cv2.warpAffine(image, shift, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    label = cv2.warpAffine(label, shift, (w, h), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REPLICATE)
    image = (image - 0.5) * rng.uniform(0.8, 1.2) + 0.5 + rng.uniform(-0.1, 0.1)
    if rng.random() < 0.3:
        image = cv2.GaussianBlur(image, (3, 3), 0)
    image = image + rng.normal(0.0, rng.uniform(0.0, 0.02), image.shape)
    return np.clip(image, 0.0, 1.0).astype(np.float32), label


class SegmentationDataset(Dataset):
    """Samples (image, label): float32 tensor 3 x H x W in [0, 1] and int64 tensor H x W with the class ids."""

    def __init__(self, root, split, augmented=False):
        self.images, self.labels = load_split(root, split)
        self.augmented = augmented

    def __len__(self):
        return len(self.images)

    def __getitem__(self, i):
        image = self.images[i].astype(np.float32) / 255.0
        label = self.labels[i]
        if self.augmented:
            image, label = augment(image, label, np.random.default_rng())
        return torch.from_numpy(image).permute(2, 0, 1).contiguous(), torch.from_numpy(label.astype(np.int64))
