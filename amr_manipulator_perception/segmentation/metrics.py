"""Metrics of the segmentation: the confusion matrix and the intersection over union (IoU) of every class.

Pixel accuracy is not used: a network that answers "background" everywhere is right on 91 percent of the pixels.
The IoU of a class is  (pixels of the class found where they are) / (pixels predicted as the class, plus pixels that
are the class, minus the ones counted twice); it is 1 for a perfect mask and 0 when the class is never found.
"""
import torch

CLASS_NAMES = ('background', 'cereal_box', 'juice_carton', 'soup_can', 'water_bottle', 'parcel', 'jam_jar')


def confusion_matrix(scores, labels, num_classes=7):
    """Matrix [true class, predicted class] of pixel counts.

    scores: B x C x H x W (output of the network); labels: B x H x W (int64 class ids)."""
    predicted = scores.argmax(dim=1)
    pairs = labels.flatten() * num_classes + predicted.flatten()
    return torch.bincount(pairs, minlength=num_classes ** 2).reshape(num_classes, num_classes).cpu()


def iou_per_class(matrix):
    """IoU of every class from a (possibly summed over many batches) confusion matrix; nan for a class that is
    neither in the labels nor in the predictions."""
    matrix = matrix.double()
    hits = matrix.diag()
    union = matrix.sum(dim=0) + matrix.sum(dim=1) - hits
    return torch.where(union > 0, hits / union.clamp(min=1), torch.full_like(hits, float('nan')))


def mean_iou(matrix):
    """Mean of the IoU of the classes that exist (nan ignored)."""
    iou = iou_per_class(matrix)
    return iou[~torch.isnan(iou)].mean().item()


def report(matrix):
    """One line per class with its IoU, and the mean."""
    iou = iou_per_class(matrix)
    lines = [f'{name:14s} {value:.3f}' for name, value in zip(CLASS_NAMES, iou.tolist())]
    return '\n'.join(lines + [f'{"mean IoU":14s} {mean_iou(matrix):.3f}'])
