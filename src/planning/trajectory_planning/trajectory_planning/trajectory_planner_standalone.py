"""
Standalone Trajectory Planner - No ROS Required

Extracts the core Dynamic Window Approach (DWA) trajectory planning logic
from the ROS node so it can be visualized on Windows without ROS dependencies.

Usage:
    python trajectory_planner_standalone.py

Requirements:
    pip install numpy scipy matplotlib scikit-image
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from scipy.spatial.distance import pdist
from skimage.draw import disk


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

class Candidate:
    def __init__(self, speed: float, omega: float, trajectory: np.ndarray, cost: float = -1):
        self.speed = speed
        self.omega = omega
        self.trajectory = trajectory
        self.cost = cost


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------

def generate_trajectory(v: float, omega: float, time_horizon: float = 5.0, dt: float = 0.5) -> np.ndarray:
    """Simulate a constant (v, omega) arc for time_horizon seconds."""
    pose = np.zeros(4)   # [x, y, yaw, t]
    traj = []
    for t in np.arange(0, time_horizon, dt):
        traj.append(pose.copy())
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += omega * dt
        pose[3] = t + dt
    return np.asarray(traj)


def generate_two_phase_trajectory(
    v: float, omega1: float, omega2: float,
    time_horizon: float = 5.0, dt: float = 0.5
) -> np.ndarray:
    """Two-phase trajectory: first half at omega1, second half at omega2."""
    pose = np.zeros(4)
    traj = []

    # Phase 1
    for t in np.arange(0, time_horizon, dt):
        traj.append(pose.copy())
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += omega1 * dt
        pose[3] = t + dt

    # Phase 2
    for t in np.arange(0, time_horizon, dt):
        traj.append(pose.copy())
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += omega2 * dt
        pose[3] = t + dt

    return np.asarray(traj)


def get_candidate_mask(
    trajectory: np.ndarray,
    grid_shape: tuple = (100, 100),
    resolution: float = 0.2,
    origin_offset: tuple = (8.0, 10.0),   # metres: (x_offset, y_offset)
    collision_radius: float = 1.0
) -> np.ndarray:
    """
    Rasterise a trajectory onto a grid and return a boolean occupancy mask.

    Args:
        trajectory:       Nx4 array [x, y, yaw, t] in robot-local (base_link) coords
        grid_shape:       (height, width) in pixels
        resolution:       metres per pixel
        origin_offset:    (ox, oy) shift so that the robot sits at pixel (ox/res, oy/res)
        collision_radius: inflation radius in metres
    """
    radius_px = collision_radius / resolution
    grid_coords = trajectory[:, :2].copy()
    grid_coords[:, 0] += origin_offset[0]   # x → col
    grid_coords[:, 1] += origin_offset[1]   # y → row
    grid_coords /= resolution               # metres → pixels

    mask = np.zeros(grid_shape, dtype=bool)
    for px, py in grid_coords:
        # disk(center, radius, shape) — note skimage uses (row, col)
        rr, cc = disk((py, px), radius_px, shape=grid_shape)
        mask[rr, cc] = True
    return mask


def build_candidates(top_speed: float = 1.5) -> list[Candidate]:
    """
    Mirror the three-speed-level candidate set from the ROS planner.
    Returns a flat list of Candidate objects.
    """
    speed_levels = [
        (top_speed,         np.linspace(-0.1, 0.1, 3)),
        (top_speed * 0.667, np.linspace(-0.2, 0.2, 5)),
        (top_speed * 0.333, np.linspace(-0.3, 0.3, 7)),
    ]

    candidates = []
    for v, omegas in speed_levels:
        for omega1 in omegas:
            for omega2 in omegas:
                traj = generate_two_phase_trajectory(v, omega1, omega2)
                mask = get_candidate_mask(traj)
                c = Candidate(v, omega1, traj)
                c.mask = mask
                candidates.append(c)
    return candidates


# ---------------------------------------------------------------------------
# Synthetic cost map
# ---------------------------------------------------------------------------

def make_cost_map(
    shape: tuple = (100, 100),
    goal_px: tuple = (50, 75),      # (row, col) of goal — ahead of robot at col=40, row=50
    obstacles: list = None
) -> np.ndarray:
    """
    Create a synthetic cost map:
      - Low cost near goal (seedling to navigate toward)
      - High cost (100) at obstacle positions
      - Background gradient so closer-to-goal cells cost less

    Grid conventions (100×100, resolution=0.2 m/px):
      Robot is at pixel (row=50, col=40), world (x=0, y=0).
      col = (x + 8) / 0.2,  row = (y + 10) / 0.2
    """
    H, W = shape
    # Distance-to-goal gradient (0 at goal, up to 40 further away)
    rows, cols = np.mgrid[0:H, 0:W]
    dist = np.sqrt((rows - goal_px[0]) ** 2 + (cols - goal_px[1]) ** 2)
    cost_map = (dist / dist.max() * 40).astype(np.float32)

    # Obstacles — placed off the direct forward path (row≈50, col>40)
    if obstacles is None:
        obstacles = [
            (35, 65, 5),   # upper region (ahead-left):  x=5m, y=-3m
            (65, 68, 5),   # lower region (ahead-right):  x=5.6m, y=3m
            (42, 80, 4),   # right-forward:               x=8m, y=-1.6m
        ]
    for r, c, rad in obstacles:
        rr, cc = disk((r, c), rad, shape=shape)
        cost_map[rr, cc] = 100.0   # impassable

    return cost_map


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def score_candidates(candidates: list[Candidate], cost_map: np.ndarray):
    """
    Assign a cost to each candidate.
    Any candidate whose mask overlaps a cell ≥ 100 is marked infeasible (cost = inf).
    """
    for c in candidates:
        max_cost = np.max(cost_map[c.mask])
        if max_cost >= 100:
            c.cost = np.inf
        else:
            c.cost = float(np.sum(cost_map[c.mask]))
            # Prefer higher speeds (reward fast candidates)
            if c.speed > 0.9:
                c.cost *= 0.5


def best_candidate(candidates: list[Candidate]) -> Candidate | None:
    feasible = [c for c in candidates if np.isfinite(c.cost)]
    if not feasible:
        return None
    return min(feasible, key=lambda c: c.cost)


# ---------------------------------------------------------------------------
# Visualisation helpers
# ---------------------------------------------------------------------------

GRID_EXTENT = [-8, 12, -10, 10]   # [xmin, xmax, ymin, ymax] in metres


def traj_world_coords(traj: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (xs, ys) of trajectory in robot-local world coords (metres)."""
    return traj[:, 0], traj[:, 1]


def plot_all_panels(
    cost_map: np.ndarray,
    candidates: list[Candidate],
    best: Candidate | None,
    goal_px: tuple
):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Standalone Trajectory Planner — DWA Visualisation", fontsize=15, fontweight="bold")

    gs = fig.add_gridspec(2, 3, hspace=0.38, wspace=0.3)

    # ------------------------------------------------------------------
    # Panel 1 — cost map
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    im = ax1.imshow(cost_map, origin="upper", extent=GRID_EXTENT, cmap="RdYlGn_r", vmin=0, vmax=100)
    ax1.scatter(0, 0, c="blue", s=120, zorder=10, label="Robot")
    # Goal in world coords: col→x, row→y with our origin offset
    resolution = 0.2
    ox, oy = 8.0, 10.0
    goal_world_x = goal_px[1] * resolution - ox
    goal_world_y = goal_px[0] * resolution - oy
    ax1.scatter(goal_world_x, goal_world_y, c="lime", marker="*", s=300, zorder=10, label="Goal seedling")
    plt.colorbar(im, ax=ax1, label="Cost")
    ax1.set_title("Synthetic Cost Map")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.legend(fontsize=8)

    # ------------------------------------------------------------------
    # Panel 2 — all candidate trajectories coloured by feasibility
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.imshow(cost_map, origin="upper", extent=GRID_EXTENT, cmap="Greys", alpha=0.4)

    feasible_costs = [c.cost for c in candidates if np.isfinite(c.cost)]
    norm = Normalize(vmin=min(feasible_costs) if feasible_costs else 0,
                     vmax=max(feasible_costs) if feasible_costs else 1)
    cmap = plt.cm.plasma

    for c in candidates:
        xs, ys = traj_world_coords(c.trajectory)
        if not np.isfinite(c.cost):
            ax2.plot(xs, ys, color="red", alpha=0.08, linewidth=0.6)
        else:
            color = cmap(norm(c.cost))
            ax2.plot(xs, ys, color=color, alpha=0.35, linewidth=0.8)

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    plt.colorbar(sm, ax=ax2, label="Feasible cost")

    ax2.scatter(0, 0, c="blue", s=100, zorder=10)
    infeasible_patch = mpatches.Patch(color="red", alpha=0.4, label="Infeasible (obstacle)")
    feasible_patch = mpatches.Patch(color="purple", alpha=0.6, label="Feasible (lower=better)")
    ax2.legend(handles=[infeasible_patch, feasible_patch], fontsize=7)
    ax2.set_title(f"All Candidates ({len(candidates)} total)")
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_xlim(GRID_EXTENT[:2]); ax2.set_ylim(GRID_EXTENT[2:])

    # ------------------------------------------------------------------
    # Panel 3 — best trajectory
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.imshow(cost_map, origin="upper", extent=GRID_EXTENT, cmap="RdYlGn_r", alpha=0.5, vmin=0, vmax=100)

    if best is not None:
        xs, ys = traj_world_coords(best.trajectory)
        ax3.plot(xs, ys, color="cyan", linewidth=2.5, zorder=8, label="Best trajectory")
        ax3.scatter(xs, ys, color="cyan", s=40, zorder=9)
        ax3.scatter(xs[-1], ys[-1], color="magenta", s=200, marker="^", zorder=10, label="Endpoint")
        ax3.set_title(
            f"Best Trajectory\nv={best.speed:.2f} m/s  ω={best.omega:.2f} rad/s  cost={best.cost:.1f}"
        )
    else:
        ax3.set_title("No feasible trajectory found!")

    ax3.scatter(0, 0, c="blue", s=120, zorder=10, label="Robot")
    ax3.scatter(goal_world_x, goal_world_y, c="lime", marker="*", s=300, zorder=10, label="Goal")
    ax3.legend(fontsize=8)
    ax3.set_xlabel("x (m)"); ax3.set_ylabel("y (m)")
    ax3.set_xlim(GRID_EXTENT[:2]); ax3.set_ylim(GRID_EXTENT[2:])

    # ------------------------------------------------------------------
    # Panel 4 — speed vs omega scatter of feasible candidates
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0])
    feasible = [c for c in candidates if np.isfinite(c.cost)]
    infeasible = [c for c in candidates if not np.isfinite(c.cost)]

    if feasible:
        fv = [c.speed for c in feasible]
        fo = [c.omega for c in feasible]
        fc = [c.cost for c in feasible]
        sc = ax4.scatter(fo, fv, c=fc, cmap="plasma_r", s=60, zorder=5, label="Feasible")
        plt.colorbar(sc, ax=ax4, label="Cost")

    if infeasible:
        iv = [c.speed for c in infeasible]
        io = [c.omega for c in infeasible]
        ax4.scatter(io, iv, c="red", marker="x", s=40, alpha=0.4, label="Infeasible")

    if best is not None:
        ax4.scatter(best.omega, best.speed, c="cyan", marker="*", s=300, zorder=10, label="Best")

    ax4.set_xlabel("Angular velocity ω (rad/s)")
    ax4.set_ylabel("Linear speed v (m/s)")
    ax4.set_title("Candidate Space")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 5 — candidate mask overlay for best trajectory
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1])
    if best is not None:
        overlay = cost_map.copy()
        display = np.stack([
            np.clip(overlay / 100, 0, 1),
            np.clip(overlay / 100, 0, 1),
            np.clip(overlay / 100, 0, 1),
        ], axis=-1)
        display[best.mask] = [0, 0.8, 1.0]   # cyan for swept area
        ax5.imshow(display, origin="upper", extent=GRID_EXTENT)
        ax5.set_title("Best Candidate Footprint (cyan)")
    else:
        ax5.set_title("No best candidate")
    ax5.set_xlabel("x (m)"); ax5.set_ylabel("y (m)")

    # ------------------------------------------------------------------
    # Panel 6 — cost histogram
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2])
    if feasible_costs:
        ax6.hist(feasible_costs, bins=30, color="steelblue", edgecolor="white", alpha=0.8)
        if best is not None:
            ax6.axvline(best.cost, color="cyan", linewidth=2, label=f"Best: {best.cost:.1f}")
        ax6.legend()
    ax6.set_xlabel("Cost")
    ax6.set_ylabel("Number of candidates")
    ax6.set_title("Cost Distribution (feasible)")
    ax6.grid(True, alpha=0.3)

    plt.savefig("trajectory_planner_output.png", dpi=150, bbox_inches="tight")
    print("Saved: trajectory_planner_output.png")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=== Steward Standalone Trajectory Planner ===")

    # 1. Build cost map
    goal_px = (50, 75)   # (row, col) → x=7m, y=0 (straight ahead)
    cost_map = make_cost_map(shape=(100, 100), goal_px=goal_px)
    print(f"Cost map: {cost_map.shape}, max={cost_map.max():.0f}")

    # 2. Generate candidates
    candidates = build_candidates(top_speed=1.5)
    print(f"Generated {len(candidates)} candidate trajectories")

    # 3. Score
    score_candidates(candidates, cost_map)
    feasible = [c for c in candidates if np.isfinite(c.cost)]
    print(f"Feasible: {len(feasible)} / {len(candidates)}")

    # 4. Pick best
    best = best_candidate(candidates)
    if best:
        print(f"Best: v={best.speed:.2f} m/s, ω={best.omega:.2f} rad/s, cost={best.cost:.1f}")
    else:
        print("No feasible trajectory found — all paths blocked!")

    # 5. Visualise
    plot_all_panels(cost_map, candidates, best, goal_px)


if __name__ == "__main__":
    main()
