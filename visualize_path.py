import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from astar import AStarPlanner

print("=" * 45)
print("  A* Path Planning Visualizer")
print("=" * 45)

# Create planner
planner = AStarPlanner(grid_size=30, cell_size=2.0)

# Add some obstacles (x, y, z, radius)
obstacles = [
    (10,  0, -5, 3),
    (20,  5, -5, 4),
    (15, -5, -8, 3),
    (25,  0, -6, 3),
    ( 5,  8, -5, 2),
    (18, -8, -5, 3),
]

print("\nAdding obstacles...")
for ox, oy, oz, r in obstacles:
    planner.add_obstacle(ox, oy, oz, r)
    print(f"  Obstacle at ({ox}, {oy}, {oz}) "
          f"radius={r}m")

# Define start and goal
start = (0, 0, -5)      # Start position (meters)
goal  = (40, 0, -10)    # Goal position  (meters)

print(f"\nStart: {start}")
print(f"Goal:  {goal}")

# Find path
path = planner.find_path(start, goal)

if path:
    print("\nWaypoints:")
    for i, wp in enumerate(path):
        print(f"  WP{i+1}: x={wp[0]:.1f} "
              f"y={wp[1]:.1f} z={wp[2]:.1f}")

    # ── 3D Visualization ──────────────────────
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles
    for ox, oy, oz, r in obstacles:
        ax.scatter(ox, oy, oz,
                  c='red', s=500,
                  marker='X', alpha=0.7,
                  label='Obstacle' if ox==10 else "")

    # Plot path
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    path_z = [p[2] for p in path]

    ax.plot(path_x, path_y, path_z,
            'b-', linewidth=2,
            label='A* Path')

    # Plot waypoints
    ax.scatter(path_x, path_y, path_z,
               c='blue', s=30, alpha=0.5)

    # Plot start and goal
    ax.scatter(*start, c='green', s=200,
               marker='^', label='Start', zorder=5)
    ax.scatter(*goal, c='gold', s=200,
               marker='*', label='Goal', zorder=5)

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('A* Drone Path Planning — 3D View')
    ax.legend()

    plt.tight_layout()
    plt.savefig(
        'D:\\DroneProjects\\path-planning\\'
        'path_visualization.png')
    plt.show()
    print("\nPath visualization saved!")
else:
    print("Could not find a path!")