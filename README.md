

# Drone Path Planning — A* Algorithm in AirSim

3D A* path planning for autonomous UAV navigation, implemented in Python and executed live in Microsoft AirSim. The drone computes an optimal collision-free path through a grid world and flies it — no human input after launch.

![Simulation Demo](simulation_demo.gif)

## Why A* over simpler approaches

Waypoint-following is easy but breaks the moment an obstacle is in the way. A* gives you optimal paths with obstacle avoidance built in. I implemented it in 3D with 26-neighbor connectivity (all directions including diagonals) so the drone can plan routes that go around and over obstacles, not just around them.

## Architecture

Three separate scripts, each with a clear job:

```
astar.py              — pure A* implementation, no AirSim dependency
drone_path_planning.py — connects A* to AirSim, flies the path
visualize_path.py      — matplotlib visualization of planned path
```

Keeping `astar.py` standalone meant I could unit-test the planner without launching AirSim every time.

## How the planner works

```
Define 3D grid → mark obstacles → set start + goal
        ↓
A* search with Euclidean heuristic (26-neighbor)
        ↓
Waypoint list → AirSim moveToPositionAsync()
```

Each grid cell maps to a real-world coordinate in AirSim's NED frame. The planner runs once at launch, then the drone executes the path.

## Setup

```bash
pip install airsim numpy matplotlib
# Launch AirSim Neighborhood environment
python drone_path_planning.py
```

The script takes off, computes the path, flies it, and lands. Path visualization saved to `airsim_path.png`.

## Results

| Metric | Value |
|---|---|
| Grid size tested | 20×20×10 |
| Avg planning time | ~12ms |
| Path optimality | Confirmed optimal vs brute-force on small grids |
| AirSim execution | Smooth — no oscillation at waypoints |

## What I learned

The first version re-planned every 2 seconds to handle dynamic obstacles — but replanning mid-flight caused jerky velocity transitions. Switched to plan-once with a static obstacle map, then handle dynamic obstacles reactively at execution time. Much smoother flight.

Diagonal movement in 3D adds 18 extra neighbors per node — worth it because paths are ~23% shorter on average versus 6-neighbor (axis-aligned only).

## Relation to warehouse-drone-v2

This AirSim version was the simulation prototype. The A* planner from `astar.py` was adapted and ported to C++ for the ESP32 in the [warehouse-drone-v2](https://github.com/yogesh031020/warehouse-drone-v2) project.

## Status

Complete. Future work: real-time replanning with incoming LiDAR data.
