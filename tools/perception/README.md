# Perception Tools

## Pot Detection

YOLO11-based object detection model for detecting pots in RGB images.

**Scripts** (`pot_detection/scripts/`):

| Script | Description |
|---|---|
| `train_yolo.py` | Train YOLO11 model on the pot dataset |
| `validate_yolo.py` | Evaluate model performance on the test set |
| `predict_yolo.py` | Run inference on a folder of images |
| `split_dataset.py` | Split raw images into train/val/test sets |

**Downloads:**
- [Model weights (best.pt)](https://drive.google.com/drive/folders/1GRYXLVdfBjWJHr1uYBA2uzi9RSa4rN-w)
- [Pot dataset](https://drive.google.com/drive/folders/1VN9E7omTHzNywc57ymVGsC3t9seLZJpl)

Place downloaded weights under `pot_detection/models/`.

### Usage

**Train:**
```bash
python pot_detection/scripts/train_yolo.py
```

**Validate:**
```bash
python pot_detection/scripts/validate_yolo.py
```

**Predict** (images must be in `<image_dir>/rgb/`, results saved to `<image_dir>/pot_detections/`):
```bash
python pot_detection/scripts/predict_yolo.py <image_dir>
```

### Model Performance (test set, 34 images)

| Metric | Score |
|---|---|
| Precision | 0.989 |
| Recall | 0.910 |
| mAP50 | 0.971 |
| mAP50-95 | 0.917 |

---

## Camera Data Collection

RealSense RGB image publisher/subscriber scripts under `camera_data_collection/`.

---

## VLP-16 Logger Tools

ROS2 package for publishing and subscribing to Velodyne VLP-16 LiDAR data. See `vlp16_logger_tools/`.
