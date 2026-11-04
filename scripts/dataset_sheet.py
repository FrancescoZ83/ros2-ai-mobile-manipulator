#!/usr/bin/env python3
"""A contact sheet of a split of the dataset: images with their labels painted over them."""
import argparse
import glob
import json
import os
import random

import cv2
import numpy as np

COLORS = np.array([[0, 0, 0], [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180]], np.uint8)
NAMES = ['background', 'cereal_box', 'juice_carton', 'soup_can', 'water_bottle', 'parcel', 'jam_jar']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('split_dir', help='for example dataset/train')
    ap.add_argument('--n', type=int, default=12, help='number of samples')
    ap.add_argument('--columns', type=int, default=3)
    ap.add_argument('--out', default='sheet.png')
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    names = sorted(glob.glob(os.path.join(args.split_dir, 'images', '*.png')))
    if not names:
        raise SystemExit(f'no images in {args.split_dir}/images')
    picked = random.Random(args.seed).sample(names, min(args.n, len(names)))
    tiles = []
    for path in picked:
        stem = os.path.splitext(os.path.basename(path))[0]
        image = cv2.imread(path)
        labels = cv2.imread(os.path.join(args.split_dir, 'labels', stem + '.png'), cv2.IMREAD_GRAYSCALE)
        painted = COLORS[np.clip(labels, 0, len(COLORS) - 1)][:, :, ::-1]
        overlay = np.where((labels > 0)[..., None], (0.5 * image + 0.5 * painted).astype(np.uint8), image)
        tile = np.hstack([image, overlay])
        meta = json.load(open(os.path.join(args.split_dir, 'meta', stem + '.json')))
        cv2.putText(tile, f"{stem} {meta['group']} d{meta['dist']}", (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        tiles.append(tile)
    while len(tiles) % args.columns:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + args.columns]) for i in range(0, len(tiles), args.columns)]
    cv2.imwrite(args.out, np.vstack(rows))
    print('saved', args.out, '; classes:', ', '.join(f'{i}={n}' for i, n in enumerate(NAMES)))


if __name__ == '__main__':
    main()
