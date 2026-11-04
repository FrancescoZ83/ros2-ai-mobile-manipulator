import torch
import torch.nn as nn


class ConvBlock(nn.Module):
    """Two 3x3 convolutions, each followed by BatchNorm and ReLU.

    Shape: B, in_ch, H, W -> B, out_ch, H, W (the size does not change).
    """

    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1, bias=False),    # BatchNorm cancels a bias
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class Down(nn.Module):
    """Halves height and width, then changes the number of channels.

    Shape: B, in_ch, H, W -> B, out_ch, H/2, W/2.
    """

    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.MaxPool2d(2),
            ConvBlock(in_ch, out_ch),
        )

    def forward(self, x):
        return self.block(x)


class Up(nn.Module):
    """Doubles the size of x, concatenates skip, then mixes with a ConvBlock.

    Shape: x is B, in_ch, H, W; skip is B, out_ch, 2H, 2W; result is B, out_ch, 2H, 2W.
    """

    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.up = nn.ConvTranspose2d(in_ch, out_ch, kernel_size=2, stride=2)
        self.block = ConvBlock(2 * out_ch, out_ch)

    def forward(self, x, skip):
        x = self.up(x)                      # B, out_ch, 2H, 2W
        x = torch.cat([skip, x], dim=1)     # B, 2 * out_ch, 2H, 2W
        return self.block(x)                # B, out_ch, 2H, 2W


class UNet(nn.Module):
    """U-shaped network: B, in_ch, H, W -> B, num_classes, H, W (class scores, no softmax)."""

    def __init__(self, in_ch=3, num_classes=7, base=16):
        super().__init__()
        # Going down: each level doubles the channels
        self.inc = ConvBlock(in_ch, base)
        self.down1 = Down(base, base * 2)
        self.down2 = Down(base * 2, base * 4)
        self.down3 = Down(base * 4, base * 8)
        self.down4 = Down(base * 8, base * 16)
        # Going up: each level halves the channels
        self.up1 = Up(base * 16, base * 8)
        self.up2 = Up(base * 8, base * 4)
        self.up3 = Up(base * 4, base * 2)
        self.up4 = Up(base * 2, base)
        # 1x1 convolution: one score per class, for every pixel
        self.outc = nn.Conv2d(base, num_classes, kernel_size=1)

    def forward(self, x):
        x1 = self.inc(x)        # base,      H,    W
        x2 = self.down1(x1)     # base * 2,  H/2,  W/2
        x3 = self.down2(x2)     # base * 4,  H/4,  W/4
        x4 = self.down3(x3)     # base * 8,  H/8,  W/8
        x5 = self.down4(x4)     # base * 16, H/16, W/16
        x = self.up1(x5, x4)    # base * 8,  H/8,  W/8
        x = self.up2(x, x3)     # base * 4,  H/4,  W/4
        x = self.up3(x, x2)     # base * 2,  H/2,  W/2
        x = self.up4(x, x1)     # base,      H,    W
        return self.outc(x)     # num_classes, H, W
