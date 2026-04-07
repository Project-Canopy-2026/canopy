#!/usr/bin/env python3
import os
import torch
from ultralytics import YOLO
import ultralytics.nn
from ultralytics import settings
settings.update({"wandb": True})
from torch.nn.modules.container import Sequential
pwd = "/content/"
# pwd = "/home/shuyi/code/canopy_local/seedling_detection_training/yolo_v8"
MODEL = "yolo11s.pt"
OUTPUT_DIR = "runs/train"


def train_yolov11(
    data_cfg:  str = "/content/0402/data.yaml",
    project:   str = "/content/run/train_0402",
    name:      str = "train_100_11s",
):
    model = YOLO("yolo11s.pt")
    model.train(
        data        = data_cfg,
        epochs      = 100,
        imgsz       = (640, 640),
        batch       = -1,
        # device      = "cpu",
        patience    = 30,
        lr0         = 0.01,
        lrf         = 0.1,
        dropout     = 0.2,  # Increase dropout to prevent overfitting
        augment     = True,
        hsv_h       = 0.0,  # Hue augmentation
        hsv_s       = 0.0,    # Saturation augmentation
        hsv_v       = 0.0,    # Value augmentation
        degrees     = 5.0,    # Random rotation
        translate   = 0.1,    # Random translation
        scale       = 0.5,    # Random scaling
        shear       = 0.0,    # No shear (fixed camera, uniform pot type)
        perspective = 0.0002, # Perspective transformation
        flipud      = 0.2,    # Vertical flip probability
        fliplr      = 0.5,    # Horizontal flip probability
        mosaic      = 0.0,    # Disable mosaic (no mixing different images)
        mixup       = 0.0,    # Add mixup augmentation
        copy_paste  = 0.0,    # Add copy-paste augmentation
        # Regularization parameters
        weight_decay = 0.0005, # L2 regularization
        warmup_epochs = 3.0,   # Gradual learning rate warmup
        cos_lr      = True,    # Cosine learning rate scheduler
        project     = project,
        name        = name,
        exist_ok    = True,
        # resume      = True,
        plots       = True,
        iou         = 0.3,
        conf        = 0.1,
        nms         = True,
    )
if __name__ == "__main__":
    # Change to the dataset directory
    # os.chdir(os.path.join(pwd, "datasets", DATASET))
    # print(os.getcwd())
    train_yolov11()
