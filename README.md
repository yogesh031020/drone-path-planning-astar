# ?? Autonomous 3D Path Planning (A* Algorithm)
**Aeronautical Engineering & GNC Validation in AirSim**

![Autonomous Navigation Demo](simulation_demo.gif)

### ?? Project Overview
This project implements a high-performance **3D A* Path Planning algorithm** designed for autonomous UAV navigation in complex environments. It bridges the gap between theoretical flight stability and real-time autonomous pathfinding.

### ?? Technical Specifications
*   **Algorithm:** 3D A* with 26-neighbor connectivity.
*   **Physics Engine:** Microsoft AirSim (Photorealistic Unreal Engine 4).
*   **Heuristic:** Euclidean distance for optimal 3D path calculation.
*   **Focus:** GNC (Guidance, Navigation, and Control) logic for quadrotors.

### ?? How to Run
1. Ensure **AirSim** is running in the 'Neighborhood' environment.
2. Run python drone_path_planning.py.
3. The drone will compute the optimal 3D path and execute it automatically.

### ?? Future Work
*   Integration with **Dynamic Obstacle Avoidance**.
*   Optimization for **Real-time Trajectory Smoothing**.
