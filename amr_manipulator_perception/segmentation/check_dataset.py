#!/usr/bin/env python3
"""Look at the data the network will see: sizes, class frequencies, and a sheet of augmented training samples."""
import argparse
import time

import cv2
import numpy as np
from torch.utils.data import DataLoader

from dataset import CLASSES, NUM_CLASSES, SegmentationDataset, pixel_frequencies

COLORS = np.array([[0, 0, 0], [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180]], np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', help='directory of the dataset')
    ap.add_argument('--out', default='check_dataset.png')
    args = ap.parse_args()

    t0 = time.monotonic()
    sets = {s: SegmentationDataset(args.root, s, augmented=(s == 'train')) for s in ('train', 'val', 'test')}
    print(f'loaded in {time.monotonic() - t0:.1f} s')
    for name, ds in sets.items():
        print(f'{name}: {len(ds)} samples, images {ds.images.shape} {ds.images.dtype}, labels {ds.labels.shape} {ds.labels.dtype}, '
              f'ids {sorted(np.unique(ds.labels).tolist())}, {(ds.images.nbytes + ds.labels.nbytes) / 2**20:.0f} MiB')
    freq = pixel_frequencies(args.root)
    print('pixel frequency in train:', {CLASSES[c]: round(float(freq[c]), 4) for c in range(NUM_CLASSES)})

    images, labels = next(iter(DataLoader(sets['train'], batch_size=8, shuffle=False)))
    print('one batch:', tuple(images.shape), images.dtype, f'range {images.min():.2f}..{images.max():.2f}', tuple(labels.shape), labels.dtype)
    rows = []
    for k in range(6):
        raw = sets['train'].images[k]                 # the same sample, before the augmentation
        aug = (images[k].permute(1, 2, 0).numpy() * 255).astype(np.uint8)
        paint = COLORS[labels[k].numpy()]
        rows.append(np.hstack([raw, aug, paint]))
    cv2.imwrite(args.out, cv2.cvtColor(np.vstack([np.hstack(rows[i:i + 2]) for i in range(0, 6, 2)]), cv2.COLOR_RGB2BGR))
    print('sheet (original | augmented | augmented labels):', args.out)


if __name__ == '__main__':
    main()
