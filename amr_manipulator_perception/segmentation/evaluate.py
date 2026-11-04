#!/usr/bin/env python3
"""Measure a trained network on a split: IoU per class, IoU by camera distance, and a contact sheet of predictions.

    python3 evaluate.py --name base                                  # the test split of the free-camera dataset
    python3 evaluate.py --name base --root ../../dataset_robot --split all --sheet /tmp/robot.png
`--split all` takes every image of the root, whatever its split (the robot set is small and is only a test).
The distance of the camera to the slot is in the metadata of every image: the far shots are the hard ones, because the
articles are only a few pixels wide. The bins are thirds of the distances found, unless --bins gives the edges.
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np
import torch

import metrics
from dataset import NUM_CLASSES, SegmentationDataset
from model import UNet

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..', 'dataset')
PALETTE = np.array([[0, 0, 0], [230, 60, 60], [240, 190, 40], [60, 200, 90], [60, 140, 240], [170, 90, 220], [40, 210, 210]], np.uint8)


def load(root, split):
    """[(dataset, metadata list)] of a split, or of all the splits that exist."""
    names = [s for s in ('train', 'val', 'test') if os.path.isdir(os.path.join(root, s, 'images'))] if split == 'all' else [split]
    out = []
    for name in names:
        data = SegmentationDataset(root, name, augmented=False)
        metas = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(root, name, 'meta', '*.json')))]
        assert len(metas) == len(data), f'{name}: metadata and images do not match'
        out.append((data, metas))
    return out


def paint(image, labels):
    """The image (uint8 RGB) with the classes painted over it."""
    out = image.copy()
    mask = labels > 0
    out[mask] = (0.45 * image[mask] + 0.55 * PALETTE[labels[mask]]).astype(np.uint8)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True, help='the network runs/<name>.pt')
    ap.add_argument('--split', default='test', help='train, val, test or all')
    ap.add_argument('--root', default=ROOT)
    ap.add_argument('--bins', type=float, nargs='+', help='edges of the distance bins [m] (default: thirds)')
    ap.add_argument('--sheet', help='write a PNG with the worst images: image | true labels | predicted labels')
    ap.add_argument('--sheet-rows', type=int, default=8)
    args = ap.parse_args()

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    checkpoint = torch.load(os.path.join(HERE, 'runs', args.name + '.pt'), map_location=device, weights_only=True)
    model = UNet(in_ch=3, num_classes=NUM_CLASSES, base=16).to(device)
    model.load_state_dict(checkpoint['model'])
    model.eval()
    print(f'network {args.name}: epoch {checkpoint["epoch"]}, validation mean IoU {checkpoint["val_miou"]:.3f}')

    sets = load(args.root, args.split)
    count = sum(len(data) for data, _ in sets)
    dists = np.array([m['dist'] for _, metas in sets for m in metas])
    edges = args.bins or list(np.quantile(dists, [0, 1 / 3, 2 / 3, 1]))
    total = torch.zeros(NUM_CLASSES, NUM_CLASSES, dtype=torch.long)
    by_bin = [torch.zeros(NUM_CLASSES, NUM_CLASSES, dtype=torch.long) for _ in range(len(edges) - 1)]
    counts = [0] * (len(edges) - 1)
    cases = []                                    # (mean IoU of the image, image, labels, prediction)
    with torch.no_grad():
        for data, metas in sets:
            for i in range(len(data)):
                x, y = data[i]
                scores = model(x[None].to(device))
                matrix = metrics.confusion_matrix(scores, y[None].to(device), NUM_CLASSES)
                total += matrix
                d = metas[i]['dist']
                for b in range(len(edges) - 1):
                    if edges[b] <= d < edges[b + 1] or (b == len(edges) - 2 and d == edges[-1]):
                        by_bin[b] += matrix
                        counts[b] += 1
                if args.sheet:
                    cases.append((metrics.mean_iou(matrix), data.images[i], data.labels[i], scores.argmax(1)[0].cpu().numpy().astype(np.uint8)))
    print(f'\n{args.root} {args.split}: {count} images')
    print(metrics.report(total))
    print('\nby distance of the camera to the slot:')
    for b in range(len(edges) - 1):
        if counts[b]:
            print(f'  {edges[b]:.2f}-{edges[b + 1]:.2f} m  {counts[b]:4d} images  mean IoU {metrics.mean_iou(by_bin[b]):.3f}')
    if args.sheet:
        cases.sort(key=lambda c: c[0])            # the worst first
        rows = [np.hstack([image, paint(image, labels), paint(image, pred)]) for _, image, labels, pred in cases[:args.sheet_rows]]
        print('\nworst images, mean IoU:', ' '.join(f'{c[0]:.2f}' for c in cases[:args.sheet_rows]))
        cv2.imwrite(args.sheet, cv2.cvtColor(np.vstack(rows), cv2.COLOR_RGB2BGR))
        print(f'sheet (image | true | predicted): {args.sheet}')


if __name__ == '__main__':
    main()
