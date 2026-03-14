"""
Standalone Forest Planner - decouples the ROS2 node to test on Windows

This script extracts the core forest planning algorithm from the ROS node
so it can be tested on Windows without ROS dependencies.

Usage:
    python forest_planner_standalone.py

Requirements:
    1. Install the required packages:
    pip install numpy scipy opencv-python scikit-image matplotlib tqdm
    2. Run from the repository root:
    cd ~/steward
    python src/planning/forest_planning/forest_planning/forest_planner_standalone.py
"""

import numpy as np
from time import time
from tqdm import trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from skimage.draw import disk
import cv2
from pathlib import Path


class BoundCell:
    DO_NOT_PLANT = 0
    PLANT_HERE = 1
    PASS_ONLY = 2


def create_forest_plan(
    bounds_path: str,
    imagery_path: str = None,
    heightmap_path: str = None,
    plan_resolution: float = 0.2,
    minimum_spacing: float = 1.0,
    do_plotting: bool = True,
) -> np.ndarray:
    """
    Generate a forest planting plan.

    Args:
        bounds_path: Path to the planting bounds image (grayscale)
        imagery_path: Path to satellite imagery (optional, for visualization)
        heightmap_path: Path to height map image (optional, for visualization)
        plan_resolution: Side length of cells within the plan, in meters
        minimum_spacing: Minimum spacing between seedlings, in meters
        do_plotting: Whether to show matplotlib plots

    Returns:
        Tuple of:
        - plan: numpy array representing the forest plan (1 = plant here, 0 = don't plant)
        - seedling_coords_meters: list of (x, y) tuples with seedling positions in meters
    """
    RES = plan_resolution
    MINIMUM_SPACING_METERS = minimum_spacing
    MINIMUM_SPACING_PX = np.floor(MINIMUM_SPACING_METERS / RES)

    print(f"Generating Forest Plan with {MINIMUM_SPACING_METERS} meter spacing")
    print(f"Resolution: {RES} m/px, Minimum spacing: {MINIMUM_SPACING_PX} px")
    start_time = time()

    # Read bounds map from disk (grayscale)
    bounds = cv2.imread(bounds_path, 0)  # "0" means grayscale mode
    if bounds is None:
        raise FileNotFoundError(f"Could not read bounds image: {bounds_path}")

    # Optionally load imagery and heightmap for visualization
    imagery = None
    heightmap = None
    if imagery_path:
        imagery = cv2.imread(imagery_path)
        if imagery is not None:
            imagery = cv2.cvtColor(imagery, cv2.COLOR_BGR2RGB)  # Convert for matplotlib
    if heightmap_path:
        heightmap = cv2.imread(heightmap_path)
        if heightmap is not None:
            heightmap = cv2.cvtColor(heightmap, cv2.COLOR_BGR2RGB)

    # Map bounds to classification scheme:
    # 0 = don't enter, 1 = plant here, 2 = enter but don't plant
    bounds_classified = bounds.copy()
    bounds_classified[bounds > 250] = BoundCell.PLANT_HERE
    bounds_classified[bounds > 100] = BoundCell.PASS_ONLY
    bounds_classified = bounds_classified.astype(np.uint8)

    if bounds_classified.shape[0] != bounds_classified.shape[1]:
        print("Warning: Planting bounds map is not square.")

    GRID_SIZE = bounds_classified.shape[0]  # px
    print(f"Grid size: {GRID_SIZE} x {GRID_SIZE} px")

    # Create the plan array
    plan = np.zeros_like(bounds_classified)

    # Store seedling coordinates (row, col) in pixels
    seedling_coords = []

    rng = np.random.default_rng()

    # Heuristic for max iterations - algorithm asymptotically approaches max density
    MAX_ITERS = int(GRID_SIZE**2 / 10)
    print(f"Running {MAX_ITERS} iterations...")

    for i in trange(MAX_ITERS):
        # Pick a random pixel
        random_idx = rng.uniform(low=0, high=GRID_SIZE, size=2)
        random_idx = np.floor(random_idx).astype(int)

        # Check if within planting bounds
        if bounds_classified[random_idx[0], random_idx[1]] != BoundCell.PLANT_HERE:
            continue

        # Check if too close to another seedling
        nearby_cells = disk(
            center=random_idx, radius=MINIMUM_SPACING_PX, shape=plan.shape
        )
        seedling_nearby = np.sum(plan[nearby_cells]) > 0

        if seedling_nearby:
            continue

        # Mark this cell as planted
        plan[random_idx[0], random_idx[1]] = 1
        seedling_coords.append((random_idx[0], random_idx[1]))

    current_seedling_count = len(seedling_coords)
    elapsed = time() - start_time
    print(f"Forest Plan generated with {current_seedling_count} seedlings in {elapsed:.2f}s")

    # Convert pixel coords to meters (origin at top-left)
    seedling_coords_meters = [(col * RES, row * RES) for row, col in seedling_coords]

    # Calculate density statistics
    plantable_area_px = np.sum(bounds_classified == BoundCell.PLANT_HERE)
    plantable_area_m2 = plantable_area_px * (RES ** 2)
    density = current_seedling_count / plantable_area_m2 if plantable_area_m2 > 0 else 0
    print(f"Plantable area: {plantable_area_m2:.1f} m², Density: {density:.2f} seedlings/m²")

    # Calculate extent for coordinate display (in meters)
    map_width_m = GRID_SIZE * RES
    map_height_m = GRID_SIZE * RES
    extent = [0, map_width_m, map_height_m, 0]  # [left, right, bottom, top]

    if do_plotting:
        fig, axs = plt.subplots(ncols=2, nrows=2, figsize=(12.0, 12.0), layout="constrained")

        # Planting bounds with coordinates
        axs[0, 0].set_title("Planting bounds")
        axs[0, 0].imshow(bounds_classified, cmap='viridis', extent=extent)
        axs[0, 0].set_xlabel("X (meters)")
        axs[0, 0].set_ylabel("Y (meters)")

        if heightmap is not None:
            axs[1, 0].imshow(heightmap, extent=extent)
            axs[1, 0].set(title="Height map")
            axs[1, 0].set_xlabel("X (meters)")
            axs[1, 0].set_ylabel("Y (meters)")
        else:
            axs[1, 0].text(0.5, 0.5, "No heightmap", ha='center', va='center', transform=axs[1, 0].transAxes)
            axs[1, 0].set(title="Height map (not loaded)")

        if imagery is not None:
            axs[1, 1].imshow(imagery, extent=extent)
            axs[1, 1].set(title="Satellite imagery")
            axs[1, 1].set_xlabel("X (meters)")
            axs[1, 1].set_ylabel("Y (meters)")
        else:
            axs[1, 1].text(0.5, 0.5, "No imagery", ha='center', va='center', transform=axs[1, 1].transAxes)
            axs[1, 1].set(title="Satellite imagery (not loaded)")

        # Show plan with seedling locations using scatter plot for visibility
        axs[0, 1].imshow(bounds_classified, cmap='Greys', alpha=0.3, extent=extent)
        if seedling_coords_meters:
            xs, ys = zip(*seedling_coords_meters)
            axs[0, 1].scatter(xs, ys, c='green', s=15, marker='o', alpha=0.8, edgecolors='darkgreen', linewidths=0.5)
        print(f"Displaying {current_seedling_count} seedlings in plot.")
        axs[0, 1].set(title=f"Forest plan ({current_seedling_count} seedlings)")
        axs[0, 1].set_xlabel("X (meters)")
        axs[0, 1].set_ylabel("Y (meters)")
        axs[0, 1].set_xlim(0, map_width_m)
        axs[0, 1].set_ylim(map_height_m, 0)  # Invert Y to match image coordinates

        plt.suptitle(f"Forest Planner Output\nSpacing: {minimum_spacing}m, Resolution: {plan_resolution}m/px, Area: {map_width_m:.0f}x{map_height_m:.0f}m")
        plt.show()

    return plan, seedling_coords_meters


def main():
    # Default paths relative to repository root
    # Adjust these paths based on where you run the script from
    repo_root = Path(__file__).parent.parent.parent.parent.parent  # Navigate up to steward/

    bounds_path = repo_root / "data" / "maps" / "flagstaff" / "planting-bounds.jpg"
    imagery_path = repo_root / "data" / "maps" / "flagstaff" / "imagery.jpg"
    heightmap_path = repo_root / "data" / "maps" / "flagstaff" / "heightmap.jpg"

    print(f"Looking for data in: {repo_root / 'data'}")
    print(f"Bounds path: {bounds_path} (exists: {bounds_path.exists()})")

    if not bounds_path.exists():
        # Try current working directory
        print("Trying current working directory...")
        bounds_path = Path("data/maps/flagstaff/planting-bounds.jpg")
        imagery_path = Path("data/maps/flagstaff/imagery.jpg")
        heightmap_path = Path("data/maps/flagstaff/heightmap.jpg")

    if not bounds_path.exists():
        print("\nError: Could not find planting bounds image.")
        print("Please run this script from the repository root directory,")
        print("or modify the paths in the script.")
        print("\nExpected structure:")
        print("  steward/")
        print("    data/maps/flagstaff/")
        print("      planting-bounds.jpg")
        print("      imagery.jpg")
        print("      heightmap.jpg")
        return

    plan_resolution = 0.2  # meters per pixel
    plan, seedling_coords_meters = create_forest_plan(
        bounds_path=str(bounds_path),
        imagery_path=str(imagery_path) if imagery_path.exists() else None,
        heightmap_path=str(heightmap_path) if heightmap_path.exists() else None,
        plan_resolution=plan_resolution,
        minimum_spacing=1.0,  # meters between seedlings
        do_plotting=True,
    )

    # Create a better output image with visible tree markers
    output_img = cv2.imread(str(bounds_path))
    if output_img is None:
        output_img = np.zeros((plan.shape[0], plan.shape[1], 3), dtype=np.uint8)

    # Draw circles at each seedling location
    marker_radius = max(3, int(0.5 / plan_resolution))  # At least 3px, or 0.5m radius
    for x_m, y_m in seedling_coords_meters:
        col = int(x_m / plan_resolution)
        row = int(y_m / plan_resolution)
        cv2.circle(output_img, (col, row), marker_radius, (0, 255, 0), -1)  # Green filled circle
        cv2.circle(output_img, (col, row), marker_radius, (0, 100, 0), 1)   # Dark green border

    # Add coordinate grid
    grid_spacing_m = 10  # Grid lines every 10 meters
    grid_spacing_px = int(grid_spacing_m / plan_resolution)
    height, width = output_img.shape[:2]

    for x in range(0, width, grid_spacing_px):
        cv2.line(output_img, (x, 0), (x, height), (128, 128, 128), 1)
        # Add coordinate label
        label = f"{x * plan_resolution:.0f}m"
        cv2.putText(output_img, label, (x + 2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    for y in range(0, height, grid_spacing_px):
        cv2.line(output_img, (0, y), (width, y), (128, 128, 128), 1)
        label = f"{y * plan_resolution:.0f}m"
        cv2.putText(output_img, label, (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Save the annotated output image (in same folder as script)
    script_dir = Path(__file__).parent / "output"
    output_path = script_dir / "forest_plan_output.png"
    print(f"Saving plan image to: {output_path}")
    cv2.imwrite(str(output_path), output_img)
    print(f"Plan image saved to: {output_path}")

    # Also save coordinates to CSV for easy access
    csv_path = script_dir / "forest_plan_coordinates.csv"
    with open(csv_path, 'w') as f:
        f.write("tree_id,x_meters,y_meters\n")
        for i, (x, y) in enumerate(seedling_coords_meters):
            f.write(f"{i},{x:.3f},{y:.3f}\n")
    print(f"Coordinates saved to: {csv_path}")


if __name__ == "__main__":
    main()
