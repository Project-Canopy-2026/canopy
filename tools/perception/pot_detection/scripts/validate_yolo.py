from ultralytics import YOLO

# Load a model
# model = YOLO("yolo11n.pt")  # load an official model
# model = YOLO("/home/shuyi/code/canopy/tools/perception/pot_detection/models/0325_yolo11n/best.pt")  # load a custom model
model = YOLO("/home/shuyi/Downloads/yolo_runs_backup_0402_with_yolo11s/content/run/train_0402/train_100_11s/weights/best.pt")  # load a custom model

# Validate the model
metrics = model.val(data="/home/shuyi/data/pot_dataset/0402/data.yaml", split="test")
metrics.box.map  # map50-95
metrics.box.map50  # map50
metrics.box.map75  # map75
metrics.box.maps  # a list containing mAP50-95 for each category