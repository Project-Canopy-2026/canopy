from ultralytics import YOLO
import os
import argparse

pwd = os.path.dirname(os.path.abspath(__file__))
model_name = "0402_yolo11s"
MODEL_PATH = os.path.join(pwd, "..", "models", model_name, "best.pt")

def predict(image_dir: str):
    rgb_dir    = os.path.join(image_dir)
    output_dir = os.path.join(image_dir, "pot_detections", model_name)

    model = YOLO(MODEL_PATH)
    model.predict(
        source  = rgb_dir,
        save    = True,
        imgsz   = 640,
        conf    = 0.56, # confidence threshold
        project = output_dir,
        device  = "cpu",
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("image_dir", help="Root image directory containing 'rgb/' subfolder")
    args = parser.parse_args()
    predict(args.image_dir)
