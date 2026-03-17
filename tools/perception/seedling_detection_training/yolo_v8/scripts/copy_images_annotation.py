import os
import shutil

# --- Configuration ---
txt_folder = "/media/fyandun/data/projects/reforestation/stem_seedling/outdoors_combined/all_stems_annotated/train/labels"              # folder with text files like image_1.txt
src_folder = "/media/fyandun/data/projects/reforestation/stem_seedling/outdoors_2/camera_camera_color_image_rect_raw"         # folder where .png files live
dst_folder = "/media/fyandun/data/projects/reforestation/stem_seedling/outdoors_combined/all_stems_annotated/train/images"         # destination folder

# Make sure destination exists
os.makedirs(dst_folder, exist_ok=True)

# Loop through all .txt files in txt_folder
for txt_file in os.listdir(txt_folder):
    if txt_file.endswith(".txt"):
        # Replace .txt with .png
        image_name = os.path.splitext(txt_file)[0] + ".png"

        src_path = os.path.join(src_folder, image_name)
        dst_path = os.path.join(dst_folder, image_name)

        if os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)
            print(f"Copied {image_name} → {dst_folder}")
        else:
            print(f"⚠️ Image not found: {src_path}")

