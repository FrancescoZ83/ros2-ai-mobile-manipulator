#!/usr/bin/env python3
"""Sanity test: can the network memorise 8 fixed images?

If the data, the network, the loss and the update of the weights are all right, the loss goes from about ln(7) = 1.95
to nearly zero and the IoU on these same images to nearly one. If not, there is a mistake to find before any real training.
"""
import argparse
import os

import torch
import torch.nn as nn

import metrics
from dataset import NUM_CLASSES, SegmentationDataset
from model import UNet

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'dataset')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=ROOT)
    ap.add_argument('--images', type=int, default=8)
    ap.add_argument('--steps', type=int, default=300)
    ap.add_argument('--lr', type=float, default=1e-3)
    args = ap.parse_args()

    torch.manual_seed(0)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # The first images of the training set, without augmentation: they must be the same at every step.
    data = SegmentationDataset(args.root, 'train', augmented=False)
    samples = [data[i] for i in range(args.images)]
    x = torch.stack([image for image, _ in samples]).to(device)           # B, 3, H, W  float32 in [0, 1]
    y = torch.stack([label for _, label in samples]).to(device)           # B, H, W     int64 class ids
    print(f'{args.images} images {tuple(x.shape)}, labels {tuple(y.shape)}, on {device}')

    model = UNet(in_ch=3, num_classes=NUM_CLASSES, base=16).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    model.train()
    for step in range(args.steps + 1):
        optimizer.zero_grad()                  # the gradients are summed by PyTorch: clear those of the last step
        scores = model(x)                      # B, 7, H, W
        loss = criterion(scores, y)            # softmax + log of the probability of the right class, mean over pixels
        if step % 20 == 0:
            print(f'step {step:4d}  loss {loss.item():.4f}', flush=True)
        if step == args.steps:
            break
        loss.backward()                        # how much each weight contributed to the error
        optimizer.step()                       # move every weight so that the error gets smaller

    model.eval()                               # BatchNorm uses its running averages, not the statistics of the batch
    with torch.no_grad():
        matrix = metrics.confusion_matrix(model(x), y, NUM_CLASSES)
    print(metrics.report(matrix))


if __name__ == '__main__':
    main()
