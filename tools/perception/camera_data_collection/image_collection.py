# Example code for collecting rgb image data using the realsense camera
import pyrealsense2 as rs
import cv2
import numpy as np
from datetime import datetime  
import os

# BEFORE running this code, define a save directory
save_dir = ...
os.makedirs(save_dir, exist_ok=True)

# Initialize RealSense streaming pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        # get one color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("Warning: color frame is empty, skipping...")
            continue

        # visualize the image
        img = np.asanyarray(color_frame.get_data())
        cv2.imshow("RGB", img)

        # press 's' to save an image, press 'q' to exit the program
        key = cv2.waitKey(1)
        if key == ord('s'):
            # Get current time stamp, in the format of 20260129_223012_123（date_time_ms)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = os.path.join(save_dir, f"img_{timestamp}.jpg")
            cv2.imwrite(filename, img)
            print(f"Saved {filename}")
        elif key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()