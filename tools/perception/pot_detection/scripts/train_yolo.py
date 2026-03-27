#!/usr/bin/env python3
import os
from ultralytics import YOLO, settings

settings.update({"wandb": True})

pwd = os.path.dirname(os.path.abspath(__file__))


def train(
    data_cfg: str = os.path.join(pwd, "..", "data.yaml"),
    project:  str = os.path.join(pwd, "..", "runs", "train"),
    name:     str = "yolo11_pots",
):
    model = YOLO("yolo11n.pt")
    model.train(
        data          = data_cfg,
        epochs        = 100,
        imgsz         = (640, 640),
        batch         = -1,
        patience      = 30,
        lr0           = 0.01,
        lrf           = 0.1,
        dropout       = 0.2,
        augment       = True,
        hsv_h         = 0.015,
        hsv_s         = 0.7,
        hsv_v         = 0.4,
        degrees       = 15.0,
        translate     = 0.1,
        scale         = 0.5,
        shear         = 2.0,
        perspective   = 0.0002,
        flipud        = 0.2,
        fliplr        = 0.5,
        mosaic        = 1.0,
        mixup         = 0.0,
        copy_paste    = 0.0,
        weight_decay  = 0.0005,
        warmup_epochs = 3.0,
        cos_lr        = True,
        project       = project,
        name          = name,
        exist_ok      = True,
        plots         = True,
        iou           = 0.3,
        conf          = 0.1,
        nms           = True,
    )


if __name__ == "__main__":
    train()
