"""
Standalone Route Planner - No ROS Required

This script extracts the core route planning (TSP) algorithm from the ROS node
so it can be tested on Windows without ROS dependencies.

Usage:
    python route_planner_standalone.py

Requirements:
    pip install numpy scipy matplotlib tqdm fast-tsp

Note: fast-tsp is optional. If not installed, Ant Colony Optimization will be used.
"""

import numpy as np
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from pathlib import Path
import csv

# Try to import fast_tsp, fall back to Ant Colony if not available
try:
    import fast_tsp
    FAST_TSP_AVAILABLE = True
except ImportError:
    FAST_TSP_AVAILABLE = False
    print("fast-tsp not installed. Will use Ant Colony Optimization instead.")
    print("Install with: pip install fast-tsp")


class AntColony:
    """
    Ant Colony Optimization for TSP.
    From: https://github.com/Akavall/AntColonyOptimization
    """
    def __init__(self, distances, n_ants, n_best, n_iterations, decay, alpha=1, beta=1):
        """
        Args:
            distances: Square matrix of distances. Diagonal assumed to be np.inf.
            n_ants: Number of ants running per iteration
            n_best: Number of best ants who deposit pheromone
            n_iterations: Number of iterations
            decay: Rate at which pheromone decays (0.95 = slow decay, 0.5 = fast decay)
            alpha: Exponent on pheromone, higher = more weight on pheromone
            beta: Exponent on distance, higher = more weight on distance
        """
        self.distances = distances
        self.pheromone = np.ones(self.distances.shape) / len(distances)
        self.all_inds = range(len(distances))
        self.n_ants = n_ants
        self.n_best = n_best
        self.n_iterations = n_iterations
        self.decay = decay
        self.alpha = alpha
        self.beta = beta

    def run(self):
        shortest_path = None
        all_time_shortest_path = ("placeholder", np.inf)
        for i in trange(self.n_iterations, desc="Ant Colony Optimization"):
            all_paths = self.gen_all_paths()
            self.spread_pheromone(all_paths, self.n_best, shortest_path=shortest_path)
            shortest_path = min(all_paths, key=lambda x: x[1])
            if shortest_path[1] < all_time_shortest_path[1]:
                all_time_shortest_path = shortest_path
            self.pheromone = self.pheromone * self.decay
        return all_time_shortest_path

    def spread_pheromone(self, all_paths, n_best, shortest_path):
        sorted_paths = sorted(all_paths, key=lambda x: x[1])
        for path, dist in sorted_paths[:n_best]:
            for move in path:
                self.pheromone[move] += 1.0 / self.distances[move]

    def gen_path_dist(self, path):
        total_dist = 0
        for ele in path:
            total_dist += self.distances[ele]
        return total_dist

    def gen_all_paths(self):
        all_paths = []
        for i in range(self.n_ants):
            path = self.gen_path(0)
            all_paths.append((path, self.gen_path_dist(path)))
        return all_paths

    def gen_path(self, start):
        path = []
        visited = set()
        visited.add(start)
        prev = start
        for i in range(len(self.distances) - 1):
            move = self.pick_move(self.pheromone[prev], self.distances[prev], visited)
            path.append((prev, move))
            prev = move
            visited.add(move)
        path.append((prev, start))  # Return to start
        return path

    def pick_move(self, pheromone, dist, visited):
        pheromone = np.copy(pheromone)
        pheromone[list(visited)] = 0
        row = pheromone**self.alpha * ((1.0 / dist) ** self.beta)
        norm_row = row / row.sum()
        move = np.random.choice(self.all_inds, 1, p=norm_row)[0]
        return move


def calculate_route(points: np.ndarray, use_fast_tsp: bool = True) -> tuple[np.ndarray, float]:
    """
    Calculate optimal route through all points using TSP solver.

    Args:
        points: Nx2 array of (x, y) coordinates
        use_fast_tsp: Use fast_tsp library if available (much faster)

    Returns:
        Tuple of (route_indices, total_distance)
    """
    print(f"Calculating route through {len(points)} points...")
    start_time = time()

    # Calculate pairwise distances
    distances = pdist(points)

    if use_fast_tsp and FAST_TSP_AVAILABLE:
        print("Using Fast-TSP solver...")
        distances_matrix = squareform(distances.astype(np.uint16), force="tomatrix", checks=False)
        route = np.asarray(fast_tsp.find_tour(distances_matrix.astype(int)))
    else:
        print("Using Ant Colony Optimization...")
        print(FAST_TSP_AVAILABLE)
        distances_matrix = squareform(distances, force="tomatrix", checks=False)
        np.fill_diagonal(distances_matrix, np.inf)  # Required by AntColony

        # ACO parameters - adjust for speed vs quality tradeoff
        n_points = len(points)
        ant_colony = AntColony(
            distances_matrix,
            n_ants=max(1, n_points // 10),  # Scale ants with problem size
            n_best=max(1, n_points // 20),
            n_iterations=min(500, max(100, n_points * 2)),  # More iterations for larger problems
            decay=0.95,
            alpha=1,
            beta=2,  # Favor shorter edges
        )
        result = ant_colony.run()
        route = np.asarray(result[0])[:, 0]

    elapsed = time() - start_time
    print(f"Route calculated in {elapsed:.2f} seconds")

    # Calculate total distance
    total_distance = 0
    for i in range(len(route)):
        p1 = points[route[i]]
        p2 = points[route[(i + 1) % len(route)]]
        total_distance += np.linalg.norm(p2 - p1)

    return route, total_distance


def load_points_from_csv(csv_path: str) -> np.ndarray:
    """Load seedling coordinates from forest planner CSV output."""
    points = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append([float(row['x_meters']), float(row['y_meters'])])
    return np.array(points)


def load_points_from_plan_image(plan_path: str, resolution: float = 0.2) -> np.ndarray:
    """Load seedling coordinates from forest plan image."""
    import cv2
    plan = cv2.imread(plan_path, 0)  # Grayscale
    if plan is None:
        raise FileNotFoundError(f"Could not read plan image: {plan_path}")

    # Find non-zero pixels (seedling locations)
    pixel_indices = np.asarray(np.where(plan > 0)).T  # (row, col)

    # Convert to (x, y) in meters
    # x = col * resolution, y = row * resolution
    points = np.zeros((len(pixel_indices), 2))
    points[:, 0] = pixel_indices[:, 1] * resolution  # x from column
    points[:, 1] = pixel_indices[:, 0] * resolution  # y from row

    return points


def generate_random_points(n_points: int, area_size: float = 100.0) -> np.ndarray:
    """Generate random test points."""
    rng = np.random.default_rng()
    return rng.uniform(0, area_size, size=(n_points, 2))


def visualize_route(
    points: np.ndarray,
    route: np.ndarray,
    total_distance: float,
    start_idx: int = 0,
    save_path: str = None,
    show_plot: bool = True
):
    """
    Visualize the route through all points.

    Args:
        points: Nx2 array of coordinates
        route: Array of indices defining visit order
        total_distance: Total route distance
        start_idx: Index of starting point in route
        save_path: Path to save figure (optional)
        show_plot: Whether to display the plot
    """
    # Reorder points according to route, starting from start_idx
    route_rolled = np.roll(route, -start_idx)
    ordered_points = points[route_rolled]

    # Close the loop for visualization
    ordered_points_closed = np.vstack([ordered_points, ordered_points[0]])

    fig, ax = plt.subplots(figsize=(12, 12))

    # Plot route line
    ax.plot(ordered_points_closed[:, 0], ordered_points_closed[:, 1],
            'b-', linewidth=1, alpha=0.7, label='Route')

    # Plot all points
    ax.scatter(points[:, 0], points[:, 1],
               c='green', s=30, zorder=5, label='Planting locations')

    # Highlight start point
    start_point = ordered_points[0]
    ax.scatter(start_point[0], start_point[1],
               c='red', s=200, marker='*', zorder=10, label='Start')

    # Add direction arrows along route (every N points)
    n_arrows = min(20, len(ordered_points) // 5)
    if n_arrows > 0:
        arrow_indices = np.linspace(0, len(ordered_points) - 2, n_arrows, dtype=int)
        for idx in arrow_indices:
            p1 = ordered_points[idx]
            p2 = ordered_points[idx + 1]
            direction = p2 - p1
            ax.annotate('', xy=p2, xytext=p1,
                        arrowprops=dict(arrowstyle='->', color='blue', lw=1.5))

    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(f'Route Planning Result\n{len(points)} locations, Total distance: {total_distance:.1f}m',
                 fontsize=14)
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    ax.invert_yaxis()  # Match image convention: 0,0 at top-left, y increases downward
    ax.grid(True, alpha=0.3)

    # Add coordinate ticks
    ax.tick_params(axis='both', which='major', labelsize=10)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Route visualization saved to: {save_path}")

    if show_plot:
        plt.show()

    return fig


def save_route_to_csv(points: np.ndarray, route: np.ndarray, csv_path: str):
    """Save the ordered route to a CSV file."""
    with open(csv_path, 'w', newline='') as f:
        f.write("visit_order,point_id,x_meters,y_meters\n")
        for visit_order, point_idx in enumerate(route):
            x, y = points[point_idx]
            f.write(f"{visit_order},{point_idx},{x:.3f},{y:.3f}\n")
    print(f"Route saved to: {csv_path}")


def main():
    script_dir = Path(__file__).parent

    # Try to load points from forest planner output
    forest_planner_dir = script_dir.parent.parent / "forest_planning" / "forest_planning" / "output"
    csv_path = forest_planner_dir / "forest_plan_coordinates.csv"

    points = None

    if csv_path.exists():
        print(f"Loading points from forest planner output: {csv_path}")
        points = load_points_from_csv(str(csv_path))
        print(f"Loaded {len(points)} planting locations")
    else:
        print(f"Forest planner output not found at: {csv_path}")
        print("Generating random test points instead...")
        points = generate_random_points(n_points=100, area_size=50.0)
        print(f"Generated {len(points)} random points in 50x50m area")

    # Add a "robot start position" at origin or center
    robot_start = np.array([[points[:, 0].mean(), points[:, 1].mean()]])
    points_with_start = np.vstack([robot_start, points])
    print(f"Added robot start position at ({robot_start[0, 0]:.1f}, {robot_start[0, 1]:.1f})")

    # Calculate route
    use_fast_tsp = FAST_TSP_AVAILABLE
    route, total_distance = calculate_route(points_with_start, use_fast_tsp=use_fast_tsp)

    # Roll route so robot start (index 0) is first
    robot_position_in_route = np.where(route == 0)[0][0]
    route = np.roll(route, -robot_position_in_route)

    print(f"\nRoute Statistics:")
    print(f"  Total points: {len(points_with_start)}")
    print(f"  Total distance: {total_distance:.1f} meters")
    print(f"  Average leg: {total_distance / len(route):.1f} meters")

    # Create output directory
    output_dir = script_dir / "output"
    output_dir.mkdir(exist_ok=True)

    # Save route visualization
    fig = visualize_route(
        points_with_start,
        route,
        total_distance,
        start_idx=0,
        save_path=str(output_dir / "route_visualization.png"),
        show_plot=True
    )

    # Save route to CSV
    save_route_to_csv(points_with_start, route, str(output_dir / "route_order.csv"))

    print(f"\nOutputs saved to: {output_dir}")


if __name__ == "__main__":
    main()
