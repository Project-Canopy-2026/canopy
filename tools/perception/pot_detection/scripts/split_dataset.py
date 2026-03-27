import os
import shutil
import random

# --- Configuration ---
SOURCE_IMAGES = "/home/shuyi/data/annotated_data/Pot detection.yolov11/train/images"   # exported images from Label Studio annotation result
SOURCE_LABELS = "/home/shuyi/data/annotated_data/Pot detection.yolov11/train/labels"   # exported labels from Label Studio annotation result
OUTPUT_DIR    = "/home/shuyi/data/pot_dataset"               # final split dataset

TRAIN_RATIO = 0.7
VAL_RATIO   = 0.2
TEST_RATIO  = 0.1  # must sum to 1.0

SEED = 42

# --- Setup output directories ---
for split in ("train", "val", "test"):
    os.makedirs(os.path.join(OUTPUT_DIR, "images", split), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "labels", split), exist_ok=True)

# --- Collect all labeled samples (images that have a matching label file) ---
label_files = [f for f in os.listdir(SOURCE_LABELS) if f.endswith(".txt")]

paired = []
for lf in label_files:
    stem = os.path.splitext(lf)[0]
    for ext in (".jpg", ".jpeg", ".png"):
        img_path = os.path.join(SOURCE_IMAGES, stem + ext)
        if os.path.exists(img_path):
            paired.append((img_path, os.path.join(SOURCE_LABELS, lf)))
            break

print(f"Found {len(paired)} labeled images")

# --- Shuffle and split ---
random.seed(SEED)
random.shuffle(paired)

n = len(paired)
n_train = int(n * TRAIN_RATIO)
n_val   = int(n * VAL_RATIO)

splits = {
    "train": paired[:n_train],
    "val":   paired[n_train:n_train + n_val],
    "test":  paired[n_train + n_val:],
}

# --- Copy files ---
for split, items in splits.items():
    for img_path, lbl_path in items:
        shutil.copy2(img_path, os.path.join(OUTPUT_DIR, "images", split, os.path.basename(img_path)))
        shutil.copy2(lbl_path, os.path.join(OUTPUT_DIR, "labels", split, os.path.basename(lbl_path)))
    print(f"  {split}: {len(items)} images")

print(f"\nDataset saved to {OUTPUT_DIR}")
