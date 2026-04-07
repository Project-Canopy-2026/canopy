import os
import shutil
import cv2
import argparse

# --- Configuration ---
BLUR_THRESHOLD = 100.0   # Laplacian variance below this → blurry
DRY_RUN = True           # set False to actually delete files

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}


def laplacian_variance(img_path: str) -> float:
    """Return the variance of the Laplacian — higher means sharper."""
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return -1.0
    return cv2.Laplacian(img, cv2.CV_64F).var()


def collect_raw(directory: str) -> list[str]:
    """Recursively collect all image paths under a flat raw directory."""
    paths = []
    for root, _, files in os.walk(directory):
        for fname in sorted(files):
            if os.path.splitext(fname)[1].lower() in IMAGE_EXTENSIONS:
                paths.append(os.path.join(root, fname))
    return paths


def collect_yolo(dataset_dir: str) -> list[str]:
    """Collect all image paths under images/{train,val,test} layout."""
    paths = []
    images_root = os.path.join(dataset_dir, "images")
    for split in sorted(os.listdir(images_root)):
        split_dir = os.path.join(images_root, split)
        if not os.path.isdir(split_dir):
            continue
        for fname in sorted(os.listdir(split_dir)):
            if os.path.splitext(fname)[1].lower() in IMAGE_EXTENSIONS:
                paths.append(os.path.join(split_dir, fname))
    return paths


def label_path_for(img_path: str, dataset_dir: str) -> str | None:
    """Return the paired label .txt path for a YOLO dataset image, or None."""
    # images/<split>/foo.jpg  →  labels/<split>/foo.txt
    rel = os.path.relpath(img_path, dataset_dir)
    parts = rel.split(os.sep)
    if len(parts) < 3 or parts[0] != "images":
        return None
    stem = os.path.splitext(parts[-1])[0]
    lbl = os.path.join(dataset_dir, "labels", parts[1], stem + ".txt")
    return lbl if os.path.exists(lbl) else None


def main(directory: str, threshold: float, dry_run: bool, blurry_dir: str | None = None):
    if blurry_dir is None:
        blurry_dir = directory.rstrip(os.sep) + "_blurry"

    # Auto-detect mode
    is_yolo = os.path.isdir(os.path.join(directory, "images"))
    mode = "YOLO dataset" if is_yolo else "raw"
    print(f"Directory : {directory}")
    print(f"Mode      : {mode}")
    print(f"Threshold : {threshold}")
    print(f"Blurry dir: {blurry_dir}")
    print(f"Dry run   : {dry_run}\n")

    image_paths = collect_yolo(directory) if is_yolo else collect_raw(directory)
    print(f"Scanning {len(image_paths)} images...")

    blurry: list[tuple[str, float]] = []
    for img_path in image_paths:
        score = laplacian_variance(img_path)
        if score < threshold:
            blurry.append((img_path, score))

    print(f"Found {len(blurry)} blurry images\n")

    moved_imgs = 0
    moved_lbls = 0

    for img_path, score in blurry:
        rel = os.path.relpath(img_path, directory)
        lbl_path = label_path_for(img_path, directory) if is_yolo else None
        lbl_note = " + label" if lbl_path else ""
        print(f"  [score={score:7.1f}] {rel}{lbl_note}")

        if not dry_run:
            # Mirror the relative path under the blurry output directory
            dst_img = os.path.join(blurry_dir, rel)
            os.makedirs(os.path.dirname(dst_img), exist_ok=True)
            shutil.move(img_path, dst_img)
            moved_imgs += 1

            if lbl_path:
                lbl_rel = os.path.relpath(lbl_path, directory)
                dst_lbl = os.path.join(blurry_dir, lbl_rel)
                os.makedirs(os.path.dirname(dst_lbl), exist_ok=True)
                shutil.move(lbl_path, dst_lbl)
                moved_lbls += 1

    if dry_run:
        print("\nDry run — no files moved. Pass --delete to move them.")
    else:
        msg = f"\nMoved {moved_imgs} images"
        if is_yolo:
            msg += f" and {moved_lbls} label files"
        print(msg + f" → {blurry_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Remove blurry images from a raw or YOLO-format dataset.\n\n"
                    "Auto-detects mode:\n"
                    "  YOLO layout  — directory contains an images/ subdirectory\n"
                    "  Raw layout   — any other flat/nested image directory",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "directory",
        help="Dataset root (YOLO) or image directory (raw). "
             "Examples:\n  /home/shuyi/data/pot_dataset/0331\n"
             "  /home/shuyi/data/pot_dataset_raw/0402/rgb",
    )
    parser.add_argument(
        "--threshold", type=float, default=BLUR_THRESHOLD,
        help="Laplacian variance threshold (default: %(default)s). "
             "Lower = only remove very blurry images.",
    )
    parser.add_argument(
        "--blurry-dir",
        help="Directory to move blurry files into (default: <directory>_blurry).",
    )
    parser.add_argument(
        "--delete", action="store_true",
        help="Actually move files. Without this flag the script is a dry run.",
    )
    args = parser.parse_args()

    main(args.directory, args.threshold, dry_run=not args.delete, blurry_dir=args.blurry_dir)
