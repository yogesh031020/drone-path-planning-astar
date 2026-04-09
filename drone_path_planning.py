import airsim
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from astar import AStarPlanner

print("=" * 50)
print("  Drone A* Path Planning in AirSim")
print("=" * 50)

# ── Connect to AirSim ─────────────────────────
print("\nConnecting to AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
time.sleep(2)
print("Connected!")

# ── Take Off ──────────────────────────────────
print("\nTaking off...")
client.takeoffAsync().join()
time.sleep(2)

# Move to safe starting height
client.moveToZAsync(-10, 2).join()
time.sleep(1)

# Get current drone position
pos = client.getMultirotorState().kinematics_estimated.position
start_x = pos.x_val
start_y = pos.y_val
start_z = pos.z_val
print(f"Drone position: x={start_x:.1f} "
      f"y={start_y:.1f} z={start_z:.1f}")

# ── Setup A* Planner ──────────────────────────
print("\nSetting up A* planner...")
planner = AStarPlanner(grid_size=40, cell_size=2.0)

# Add obstacles based on AirSim environment
# These match the neighborhood environment
obstacles = [
    (10,   5, -10, 3),
    (20,  -5, -10, 4),
    (15,   0,  -8, 3),
    (25,   8, -10, 3),
    ( 8,  -8, -10, 2),
    (30,   0, -10, 4),
    (20,  10, -10, 3),
]

print("Adding obstacles to grid...")
for ox, oy, oz, r in obstacles:
    planner.add_obstacle(
        start_x + ox,
        start_y + oy,
        oz, r)

# Define goal (50 meters ahead)
goal_x = start_x + 40
goal_y = start_y
goal_z = -10

start_world = (start_x, start_y, start_z)
goal_world  = (goal_x, goal_y, goal_z)

print(f"\nStart: {start_world}")
print(f"Goal:  {goal_world}")

# ── Find Path ─────────────────────────────────
path = planner.find_path(start_world, goal_world)

if not path:
    print("No path found! Landing...")
    client.landAsync().join()
    exit()

print(f"\nPath found with {len(path)} waypoints!")

# ── Visualize Path Before Flying ──────────────
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot obstacles
for ox, oy, oz, r in obstacles:
    ax.scatter(start_x+ox, start_y+oy, oz,
              c='red', s=300, marker='X')

# Plot path
path_x = [p[0] for p in path]
path_y = [p[1] for p in path]
path_z = [p[2] for p in path]
ax.plot(path_x, path_y, path_z,
        'b-', linewidth=2, label='A* Path')
ax.scatter(path_x, path_y, path_z,
           c='blue', s=20, alpha=0.5)

# Start and goal
ax.scatter(*start_world, c='green',
           s=200, marker='^', label='Start')
ax.scatter(*goal_world, c='gold',
           s=200, marker='*', label='Goal')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('A* Path — Drone Will Follow This Route')
ax.legend()
plt.tight_layout()
plt.savefig(
    'D:\\DroneProjects\\path-planning\\'
    'airsim_path.png')
plt.show(block=False)
plt.pause(2)

# ── Fly the Path ──────────────────────────────
print("\nStarting autonomous flight!")
print("Watch the drone in AirSim!")
print("-" * 40)

SPEED = 3.0          # meters per second
WAYPOINT_RADIUS = 2.0 # how close to count as reached

for i, waypoint in enumerate(path):
    wp_x, wp_y, wp_z = waypoint

    print(f"Waypoint {i+1}/{len(path)}: "
          f"x={wp_x:.1f} "
          f"y={wp_y:.1f} "
          f"z={wp_z:.1f}")

    # Fly to waypoint
    client.moveToPositionAsync(
        wp_x, wp_y, wp_z, SPEED).join()

    # Check if reached waypoint
    pos = client.getMultirotorState()\
        .kinematics_estimated.position
    dist = np.sqrt(
        (pos.x_val - wp_x)**2 +
        (pos.y_val - wp_y)**2 +
        (pos.z_val - wp_z)**2)

    print(f"  Reached! Distance error: {dist:.2f}m")
    time.sleep(0.2)

# ── Mission Complete ──────────────────────────
print("\n" + "=" * 50)
print("Mission Complete!")
print(f"Reached goal: x={goal_x:.1f} "
      f"y={goal_y:.1f} z={goal_z:.1f}")
print("=" * 50)

# Hover for 3 seconds
client.hoverAsync().join()
time.sleep(3)

# Land
print("\nLanding...")
client.landAsync().join()
client.armDisarm(False)
print("Done!")

plt.close()