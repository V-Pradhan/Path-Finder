from collections import deque
import heapq
import math

# -------------------------------
# Helper: Define grid environment
# -------------------------------
grid = [
    [0, 0, 1, 0],
    [1, 0, 1, 0],
    [0, 0, 0, 0],
    [0, 1, 1, 0]
]

start = (0, 0)
goal = (3, 3)

# Valid directional moves (Right, Down, Left, Up)
directions = [(0,1), (1,0), (0,-1), (-1,0)]

rows, cols = len(grid), len(grid[0])

# -------------------------------
# 1. BFS Implementation
# -------------------------------
def bfs(grid, start, goal):
    queue = deque([(start, [start])])
    visited = set([start])

    while queue:
        (r, c), path = queue.popleft()
        if (r, c) == goal:
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0 and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append(((nr, nc), path + [(nr, nc)]))
    return None

# -------------------------------
# 2. DFS Implementation
# -------------------------------
def dfs(grid, start, goal):
    stack = [(start, [start])]
    visited = set([start])

    while stack:
        (r, c), path = stack.pop()
        if (r, c) == goal:
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0 and (nr, nc) not in visited:
                visited.add((nr, nc))
                stack.append(((nr, nc), path + [(nr, nc)]))
    return None

# -------------------------------
# 3. A* Algorithm Implementation
# -------------------------------
def heuristic(a, b):
    # Manhattan distance heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start, [start]))
    g_cost = {start: 0}
    visited = set()

    while open_set:
        _, current, path = heapq.heappop(open_set)
        if current == goal:
            return path

        if current in visited:
            continue
        visited.add(current)

        for dr, dc in directions:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                new_cost = g_cost[current] + 1
                if neighbor not in g_cost or new_cost < g_cost[neighbor]:
                    g_cost[neighbor] = new_cost
                    f_cost = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_cost, neighbor, path + [neighbor]))
    return None

# -------------------------------
# 4. AO* (AND-OR Search Algorithm)
# -------------------------------
class AOStar:
    def __init__(self, graph, heuristic):
        self.graph = graph
        self.heuristic = heuristic
        self.solution = {}

    def get_minimum_cost_child_nodes(self, node):
        min_cost = math.inf
        min_cost_child_nodes = []
        for child_nodes in self.graph[node]:
            cost = sum(self.heuristic[child] for child in child_nodes)
            if cost < min_cost:
                min_cost = cost
                min_cost_child_nodes = child_nodes
        return min_cost, min_cost_child_nodes

    def ao_star(self, node):
        if node in self.solution:
            return self.solution[node]
        min_cost, min_children = self.get_minimum_cost_child_nodes(node)
        self.solution[node] = min_children
        for child in min_children:
            self.ao_star(child)
        return self.solution

# Example graph for AO* demonstration
graph = {
    'A': [['B', 'C'], ['D']],
    'B': [['E', 'F']],
    'C': [['G']],
    'D': [[]],
    'E': [[]],
    'F': [[]],
    'G': [[]]
}
heuristics = {'A': 5, 'B': 2, 'C': 4, 'D': 7, 'E': 3, 'F': 5, 'G': 1}

ao = AOStar(graph, heuristics)

# -------------------------------
# Execute Algorithms
# -------------------------------
print("\nBFS Path:", bfs(grid, start, goal))
print("DFS Path:", dfs(grid, start, goal))
print("A* Path:", astar(grid, start, goal))
print("\nAO* Solution Graph:")
print(ao.ao_star('A'))


import matplotlib.pyplot as plt
import numpy as np

# The grid and solutions
grid = [
    [0, 0, 1, 0],
    [1, 0, 1, 0],
    [0, 0, 0, 0],
    [0, 1, 1, 0]
]
start = (0, 0)
goal = (3, 3)
bfs_path = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 2), (2, 3), (3, 3)]
dfs_path = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 2), (2, 3), (3, 3)]
astar_path = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 2), (2, 3), (3, 3)]

def plot_grid_path(grid, path, start, goal, title):
    grid_np = np.array(grid)
    plt.figure(figsize=(4,4))
    plt.imshow(grid_np, cmap='Greys', origin='upper')
    if path:
        px, py = zip(*path)
        plt.plot(py, px, marker='o', color='red', linewidth=2, markersize=8, label='Path')
    plt.scatter([start[1]], [start[0]], color='green', s=120, label='Start')
    plt.scatter([goal[1]], [goal[0]], color='blue', s=120, label='Goal')
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.title(title)
    plt.legend()
    plt.grid(False)
    plt.show()

# Visualize all three paths
plot_grid_path(grid, bfs_path, start, goal, 'BFS Path')
plot_grid_path(grid, dfs_path, start, goal, 'DFS Path')
plot_grid_path(grid, astar_path, start, goal, 'A* Path')
