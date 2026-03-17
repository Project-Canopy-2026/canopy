from ultralytics import YOLO
import os

pwd = os.path.dirname(os.path.abspath(__file__))
MODEL = "best.pt"
model_cfg = os.path.join(pwd, "..", "models", MODEL)

# dir_custom_model = "runs/train/yolov8_stems_testt"
project_dir = os.path.join(pwd, "..", "runs", "results")


# Define path to directory containing images and videos for inference
# source = "/media/fyandun/data/projects/reforestation/stem_seedling/all_images/"
source = "/home/shuyi/data/stem_20260517/rgb"
model_cfg = os.path.join(pwd, "..", "models", "best.pt")
# Load a pretrained YOLO11n model
model = YOLO(model_cfg)

# TODO: might want to reduce conf to 0.05, 
model.predict(source, save=True, imgsz=640, conf=0.5, project=project_dir, device="cpu") # conf - Sets the minimum confidence threshold for detections
# # Run inference on the source
# results = model(source, stream=True)  # generator of Results objects

# # Process results list
# for result in results:
#     boxes = result.boxes  # Boxes object for bounding box outputs
#     masks = result.masks  # Masks object for segmentation masks outputs
#     keypoints = result.keypoints  # Keypoints object for pose outputs
#     probs = result.probs  # Probs object for classification outputs
#     obb = result.obb  # Oriented boxes object for OBB outputs
#     result.show()  # display to screen
#     result.save(filename="result.jpg")  # save to disk
