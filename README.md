# ✈️ Drone Path Planning — A* Algorithm in AirSim

[![Status](https://img.shields.io/badge/Status-Complete-brightgreen?style=flat-square)](https://github.com/yogesh031020/drone-path-planning-astar)
[![Algorithm](https://img.shields.io/badge/Algorithm-A*%203D%2026--Neighbor-blue?style=flat-square)](https://github.com/yogesh031020/drone-path-planning-astar)
[![Simulator](https://img.shields.io/badge/Simulator-AirSim-orange?style=flat-square)](https://github.com/Microsoft/AirSim)
[![Language](https://img.shields.io/badge/Language-Python-yellow?style=flat-square)](https://www.python.org/)

3D A* path planning for autonomous UAV navigation, implemented in Python and executed live in Microsoft AirSim. The drone computes an optimal collision-free path through a grid world and flies it — no human input after launch.

---

## 🎬 Simulation Demo

![Simulation Demo](simulation_demo.gif)

> A* computes the optimal 3D path through the grid, then the drone executes the waypoints smoothly in AirSim — no oscillation at waypoints, ~12ms average planning time.

---

## Why A* Over Simpler Approaches

Waypoint-following is easy but breaks the moment an obstacle is in the way. A* gives you optimal paths with obstacle avoidance built in. Implemented in 3D with 26-neighbor connectivity (all directions including diagonals) so the drone can plan routes that go around *and* over obstacles — not just around them.

---

## Architecture

Three scripts, each with a clear job:

```
astar.py               -- pure A* implementation, no AirSim dependency
drone_path_planning.py -- connects A* to AirSim, flies the path
visualize_path.py      -- matplotlib 3D visualization of planned path
```

Keeping `astar.py` standalone means you can unit-test the planner without launching AirSim every time.

---

## How the Planner Works

```
Define 3D grid --> mark obstacles --> set start + goal
        |
A* search with Euclidean heuristic (26-neighbor)
        |
Waypoint list --> AirSim moveToPositionAsync()
```

Each grid cell maps to a real-world coordinate in AirSim's NED frame. The planner runs once at launch, then the drone executes the path.

---

## 🛠️ How to Run

### 1. Set Up AirSim
1. Download a pre-compiled AirSim binary — **AirSimNH (Neighborhood)** — from the [AirSim releases page](https://github.com/Microsoft/AirSim/releases).
2. Unzip and launch the executable (`AirSimNH.exe` on Windows, `./AirSimNH.sh` on Linux).
3. When prompted **"Would you like to use car instead of quadcopter?"** click **No**.

### 2. Clone & Install Dependencies
```bash
git clone https://github.com/yogesh031020/drone-path-planning-astar.git
cd drone-path-planning-astar
pip install airsim numpy matplotlib
```

### 3. Run the Path Planning Mission
```bash
python drone_path_planning.py
```

The script connects to AirSim, arms motors, takes off, runs the A* planner, flies the computed waypoints, and lands autonomously. The final trajectory is saved to `airsim_path.png` in the repo root.

### 4. Visualize the Path Without AirSim
To view the computed 3D path in a matplotlib wireframe without launching the simulator:
```bash
python visualize_path.py
```

---

## Results

| Metric | Value |
|---|---|
| Grid size tested | 20×20×10 |
| Avg planning time | ~12ms |
| Path optimality | Confirmed optimal vs brute-force on small grids |
| AirSim execution | Smooth — no oscillation at waypoints |

---

## What I Learned

- The first version re-planned every 2 seconds to handle dynamic obstacles — mid-flight replanning caused jerky velocity transitions. Switched to plan-once with a static obstacle map, handling dynamic obstacles reactively at execution time. Much smoother flight.
- Diagonal movement in 3D adds 18 extra neighbors per node — worth it because paths are ~23% shorter on average versus 6-neighbor (axis-aligned only).

---

## Relation to warehouse-drone-v2

This AirSim version was the simulation prototype. The A* planner from `astar.py` was adapted and ported to C++ for the ESP32 in the [warehouse-drone-v2](https://github.com/yogesh031020/warehouse-drone-v2) project.

---

## Status

Complete. Future work: real-time replanning with incoming LiDAR data.
