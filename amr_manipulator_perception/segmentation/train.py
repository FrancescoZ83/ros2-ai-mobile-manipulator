#!/usr/bin/env python3
"""Train the U-Net on the training split, measure it on the validation split after every epoch, keep the best.

The best epoch is the one with the highest mean IoU on the validation set (not the lowest loss, and not the accuracy
of the pixels: see metrics.py). The test split is not used here: it is for the final measure, once.

    python3 train.py --name base                       # plain cross entropy
    python3 train.py --name sqrt --class-weights sqrt  # rare classes count more in the loss
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

import metrics
from dataset import NUM_CLASSES, SegmentationDataset, pixel_frequencies
from model import UNet

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..', 'dataset')


def class_weights(kind, root):
    """Weight of every class in the loss: None, 1 / f, or 1 / sqrt(f) (f: fraction of the pixels), scaled to mean 1."""
    if kind == 'none':
        return None
    f = pixel_frequencies(root, 'train')
    w = 1.0 / f if kind == 'inv' else 1.0 / np.sqrt(f)
    return torch.tensor(w / w.mean(), dtype=torch.float32)


def evaluate(model, loader, criterion, device):
    """(mean loss, confusion matrix) on a loader, with the network in evaluation mode."""
    model.eval()                          # BatchNorm uses its running averages
    matrix, total, count = torch.zeros(NUM_CLASSES, NUM_CLASSES, dtype=torch.long), 0.0, 0
    with torch.no_grad():                 # no gradients: less memory, faster
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            scores = model(x)
            total += criterion(scores, y).item() * len(x)
            count += len(x)
            matrix += metrics.confusion_matrix(scores, y, NUM_CLASSES)
    return total / count, matrix


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True, help='the run: files are written in runs/<name>.*')
    ap.add_argument('--root', default=ROOT)
    ap.add_argument('--epochs', type=int, default=30)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--class-weights', choices=['none', 'sqrt', 'inv'], default='none')
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    os.makedirs(os.path.join(HERE, 'runs'), exist_ok=True)
    stem = os.path.join(HERE, 'runs', args.name)

    train_set = SegmentationDataset(args.root, 'train', augmented=True)
    val_set = SegmentationDataset(args.root, 'val', augmented=False)
    # The batches go from the workers to the main process through /dev/shm, which is 64 MB in the container: a batch of 8
    # is 12 MB (images in float32, labels in int64), so only one batch per worker is prepared in advance.
    train_loader = DataLoader(train_set, batch_size=args.batch, shuffle=True, num_workers=args.workers, drop_last=True,
                              prefetch_factor=1)
    val_loader = DataLoader(val_set, batch_size=args.batch, shuffle=False, num_workers=2, prefetch_factor=1)
    print(f'train {len(train_set)} images, val {len(val_set)} images, {device}', flush=True)

    weights = class_weights(args.class_weights, args.root)
    print('class weights:', None if weights is None else [round(float(w), 2) for w in weights], flush=True)
    model = UNet(in_ch=3, num_classes=NUM_CLASSES, base=16).to(device)
    criterion = nn.CrossEntropyLoss(weight=None if weights is None else weights.to(device))
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    # The learning rate goes down smoothly to ~0 at the last epoch: big steps first, small ones to settle.
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    history, best, t0 = [], -1.0, time.monotonic()
    for epoch in range(1, args.epochs + 1):
        model.train()
        total, count = 0.0, 0
        for x, y in train_loader:
            x, y = x.to(device), y.to(device)
            optimizer.zero_grad()
            loss = criterion(model(x), y)
            loss.backward()
            optimizer.step()
            total += loss.item() * len(x)
            count += len(x)
        scheduler.step()
        val_loss, matrix = evaluate(model, val_loader, criterion, device)
        iou = metrics.iou_per_class(matrix)
        miou = metrics.mean_iou(matrix)
        row = {'epoch': epoch, 'train_loss': total / count, 'val_loss': val_loss, 'val_miou': miou,
               'val_iou': [None if v != v else round(v, 4) for v in iou.tolist()]}
        history.append(row)
        mark = ''
        if miou > best:
            best = miou
            torch.save({'model': model.state_dict(), 'epoch': epoch, 'val_miou': miou, 'args': vars(args)}, stem + '.pt')
            mark = '  *'
        print(f'epoch {epoch:3d}  train {row["train_loss"]:.4f}  val {val_loss:.4f}  mIoU {miou:.3f}  '
              f'[{" ".join(f"{v:.2f}" for v in iou.tolist())}]  {time.monotonic() - t0:.0f} s{mark}', flush=True)
        with open(stem + '.json', 'w') as f:
            json.dump(history, f, indent=1)
    print(f'best validation mean IoU {best:.3f}; network in {stem}.pt', flush=True)


if __name__ == '__main__':
    main()
