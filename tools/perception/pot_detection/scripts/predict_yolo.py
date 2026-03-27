from ultralytics import YOLO
import os
import argparse

pwd = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(pwd, "..", "models", "best.pt")

def predict(image_dir: str):
    rgb_dir    = os.path.join(image_dir, "rgb")
    output_dir = os.path.join(image_dir, "pot_detections")

    model = YOLO(MODEL_PATH)
    model.predict(
        source  = rgb_dir,
        save    = True,
        imgsz   = 640,
        conf    = 0.5, # confidence threshold
        project = output_dir,
        device  = "cpu",
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("image_dir", help="Root image directory containing 'rgb/' subfolder")
    args = parser.parse_args()
    predict(args.image_dir)
