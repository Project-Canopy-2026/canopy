import pyrealsense2 as rs
import cv2
import numpy as np
from datetime import datetime
import os

# Define a save directory
save_dir = "data"
os.makedirs(save_dir, exist_ok=True)

# Initialize RealSense streaming pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # enable depth
pipeline.start(config)

# Create a colorizer to visualize depth
colorizer = rs.colorizer()

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("Warning: missing frame, skipping...")
            continue

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Visualize RGB and depth
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        cv2.imshow("RGB", color_image)
        cv2.imshow("Depth", depth_colormap)

        key = cv2.waitKey(1)
        if key == ord('s'):
            # Timestamp for filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

            # Save RGB image
            rgb_filename = os.path.join(save_dir, f"rgb_{timestamp}.jpg")
            cv2.imwrite(rgb_filename, color_image)

            # Save depth image (16-bit PNG)
            depth_filename = os.path.join(save_dir, f"depth_{timestamp}.png")
            cv2.imwrite(depth_filename, depth_image)

            print(f"Saved RGB: {rgb_filename}, Depth: {depth_filename}")

        elif key == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()