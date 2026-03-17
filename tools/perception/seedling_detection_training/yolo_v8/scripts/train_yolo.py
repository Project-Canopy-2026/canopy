#!/usr/bin/env python3
import os
import torch
from ultralytics import YOLO
import ultralytics.nn
from torch.nn.modules.container import Sequential
pwd = os.path.dirname(os.path.abspath(__file__))
# pwd = "/home/shuyi/code/canopy_local/seedling_detection_training/yolo_v8"
MODEL = "best.pt"
DATASET = "stems_outdoors"
DATASET = "/~/data/seedling_stem/stems_highbay_annotated"
OUTPUT_DIR = "runs/train"
data_config = os.path.join(DATASET, "data.yaml")
def train_yolov8(
    model_cfg: str = os.path.join(pwd, "..", "models", MODEL),
    data_cfg:  str = os.path.join(DATASET, "data.yaml"),
    project:   str = os.path.join(pwd, "..", OUTPUT_DIR),
    name:      str = "yolov8_stems_highbay",
):
    print(model_cfg)
    model = YOLO(model=model_cfg)
    # model = YOLO(os.path.join(pwd, "runs/train/yolov8_large_plant_diseases/weights/best.pt"))
    model.train(
        data        = data_cfg,
        epochs      = 200,
        # imgsz       = (640, 640),
        imgsz       = (480, 848), # for highbay_annotated dataset
        # imgsz       = (216, 409), # For Ximea camera (demosaic)
        # imgsz       = (1088, 2048), # For Ximea camera
        batch       = -1,
        patience    = 200,
        lr0         = 0.01,
        lrf         = 0.01,
        dropout     = 0.2,  # Increase dropout to prevent overfitting
        augment     = True,
        hsv_h       = 0.0,  # Hue augmentation
        hsv_s       = 0.0,    # Saturation augmentation
        hsv_v       = 0.0,    # Value augmentation
        degrees     = 15.0,   # Random rotation
        translate   = 0.1,    # Random translation
        scale       = 0.5,    # Random scaling
        shear       = 2.0,    # Random shear
        perspective = 0.0002, # Perspective transformation
        flipud      = 0.0,    # Vertical flip probability
        fliplr      = 0.5,    # Horizontal flip probability
        mosaic      = 1.0,    # Keep mosaic augmentation
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
        nms         = True
    )
if __name__ == "__main__":
    # Change to the dataset directory
    # os.chdir(os.path.join(pwd, "datasets", DATASET))
    # print(os.getcwd())
    train_yolov8()
    