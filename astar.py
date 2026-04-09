import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, grid_size=50, cell_size=2.0):
        """
        grid_size : number of cells in each direction
        cell_size : real world meters per cell
        """
        self.grid_size = grid_size
        self.cell_size = cell_size

        # 3D grid: 0=free, 1=obstacle
        self.grid = np.zeros((
            grid_size,
            grid_size,
            grid_size
        ), dtype=np.uint8)

        print(f"Grid created: {grid_size}x"
              f"{grid_size}x{grid_size}")
        print(f"Cell size: {cell_size}m")
        print(f"Total space: "
              f"{grid_size * cell_size}m x "
              f"{grid_size * cell_size}m x "
              f"{grid_size * cell_size}m")

    def world_to_grid(self, x, y, z):
        """Convert real world coords to grid coords"""
        gx = int(x / self.cell_size + self.grid_size // 2)
        gy = int(y / self.cell_size + self.grid_size // 2)
        gz = int(z / self.cell_size + self.grid_size // 2)
        # Clamp to grid bounds
        gx = max(0, min(gx, self.grid_size - 1))
        gy = max(0, min(gy, self.grid_size - 1))
        gz = max(0, min(gz, self.grid_size - 1))
        return gx, gy, gz

    def grid_to_world(self, gx, gy, gz):
        """Convert grid coords back to real world"""
        x = (gx - self.grid_size // 2) * self.cell_size
        y = (gy - self.grid_size // 2) * self.cell_size
        z = (gz - self.grid_size // 2) * self.cell_size
        return x, y, z

    def add_obstacle(self, x, y, z, radius=2.0):
        """Mark an obstacle in the grid"""
        r = int(radius / self.cell_size) + 1
        gx, gy, gz = self.world_to_grid(x, y, z)
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                for dz in range(-r, r+1):
                    nx = gx + dx
                    ny = gy + dy
                    nz = gz + dz
                    if (0 <= nx < self.grid_size and
                        0 <= ny < self.grid_size and
                        0 <= nz < self.grid_size):
                        self.grid[nx][ny][nz] = 1

    def heuristic(self, a, b):
        """3D Euclidean distance heuristic"""
        return np.sqrt(
            (a[0]-b[0])**2 +
            (a[1]-b[1])**2 +
            (a[2]-b[2])**2
        )

    def get_neighbors(self, node):
        """Get all 26 neighbors in 3D grid"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nx = node[0] + dx
                    ny = node[1] + dy
                    nz = node[2] + dz
                    if (0 <= nx < self.grid_size and
                        0 <= ny < self.grid_size and
                        0 <= nz < self.grid_size and
                        self.grid[nx][ny][nz] == 0):
                        # Diagonal moves cost more
                        cost = np.sqrt(
                            dx**2 + dy**2 + dz**2)
                        neighbors.append(
                            ((nx, ny, nz), cost))
        return neighbors

    def find_path(self, start_world, goal_world):
        """
        Find shortest path from start to goal
        Returns list of world coordinate waypoints
        """
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)

        print(f"\nFinding path...")
        print(f"Start grid: {start}")
        print(f"Goal  grid: {goal}")

        # Check if start or goal is obstacle
        if self.grid[start] == 1:
            print("WARNING: Start position is obstacle!")
            self.grid[start] = 0
        if self.grid[goal] == 1:
            print("WARNING: Goal position is obstacle!")
            self.grid[goal] = 0

        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            # Reached goal!
            if current == goal:
                path = []
                while current in came_from:
                    world = self.grid_to_world(*current)
                    path.append(world)
                    current = came_from[current]
                path.append(self.grid_to_world(*start))
                path.reverse()
                print(f"Path found! "
                      f"{len(path)} waypoints")
                return path

            for neighbor, cost in \
                    self.get_neighbors(current):
                tentative_g = (g_score[current] + cost)

                if (neighbor not in g_score or
                        tentative_g < g_score[neighbor]):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = (
                        tentative_g +
                        self.heuristic(neighbor, goal))
                    heapq.heappush(
                        open_set,
                        (f_score[neighbor], neighbor))

        print("No path found!")
        return None