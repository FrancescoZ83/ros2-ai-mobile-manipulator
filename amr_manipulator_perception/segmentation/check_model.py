#!/usr/bin/env python3
"""Checks of the pieces of model.py, one by one, so that every block can be tested as soon as it is written.

Expected in model.py (the sizes are what the checks verify):
  ConvBlock(in_ch, out_ch)              two 3x3 convolutions that keep the size: B,in_ch,H,W -> B,out_ch,H,W
  Down(in_ch, out_ch)                   halves the size and changes the channels: B,in_ch,H,W -> B,out_ch,H/2,W/2
  Up(in_ch, out_ch)                     forward(x, skip): x has in_ch channels, skip has out_ch channels and twice the
                                        size of x; the result has out_ch channels and the size of skip
  UNet(in_ch=3, num_classes=7, base=16) B,3,H,W -> B,num_classes,H,W (scores of the classes, no softmax)
"""
import sys

import torch

import model

RESULTS = []


def check(name, fn):
    try:
        message = fn()
        RESULTS.append(True)
        print(f'OK    {name}' + (f': {message}' if message else ''))
    except AttributeError as err:
        RESULTS.append(False)
        print(f'--    {name}: not written yet ({err})')
    except Exception as err:                    # a failed check must not hide the next ones
        RESULTS.append(False)
        print(f'FAIL  {name}: {type(err).__name__}: {err}')


def shape(t):
    return tuple(t.shape)


def check_convblock():
    y = model.ConvBlock(3, 16)(torch.zeros(2, 3, 240, 320))
    assert shape(y) == (2, 16, 240, 320), f'got {shape(y)}, expected (2, 16, 240, 320)'


def check_down():
    y = model.Down(16, 32)(torch.zeros(2, 16, 240, 320))
    assert shape(y) == (2, 32, 120, 160), f'got {shape(y)}, expected (2, 32, 120, 160)'


def check_up():
    y = model.Up(256, 128)(torch.zeros(2, 256, 15, 20), torch.zeros(2, 128, 30, 40))
    assert shape(y) == (2, 128, 30, 40), f'got {shape(y)}, expected (2, 128, 30, 40)'


def check_unet():
    net = model.UNet()
    x = torch.randn(2, 3, 240, 320)
    y = net(x)
    assert shape(y) == (2, 7, 240, 320), f'got {shape(y)}, expected (2, 7, 240, 320)'
    y.mean().backward()
    missing = [n for n, p in net.named_parameters() if p.grad is None]
    assert not missing, f'no gradient for {missing[:3]}: a block is not connected to the output'
    n = sum(p.numel() for p in net.parameters())
    assert 1.7e6 < n < 2.2e6, f'{n} parameters, about 1.9 million are expected with base=16 and four levels'
    return f'{n / 1e6:.2f} million parameters'


def main():
    for name, fn in (('ConvBlock', check_convblock), ('Down', check_down), ('Up', check_up), ('UNet', check_unet)):
        check(name, fn)
    sys.exit(0 if all(RESULTS) else 1)


if __name__ == '__main__':
    main()
