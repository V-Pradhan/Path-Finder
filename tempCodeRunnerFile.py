import matplotlib.pyplot as plt
import numpy as np

# The grid environment (1 = obstacle, 0 = free cell)
grid = [
    [0, 0, 1, 0],
    [1, 0, 1, 0],
    [0, 0, 0, 0],
    [0, 1, 1, 0]
]

def get_user_node(prompt):
    while True:
        try:
            r = int(input(f"{prompt} row (0-{len(grid)-1}): "))
            c = int(input(f"{prompt} col (0-{len(grid[0])-1}): "))
            if (0 <= r < len(grid)) and (0 <= c < len(grid[0])) and grid[r][c] == 0:
                return (r, c)
            else:
                print("Invalid cell (outside grid or obstacle). Try again.")
        except Exception as e:
            print("Invalid input, enter integers only.")

# Get start and goal from user
start = get_user_node("Enter START")
goal = get_user_node("Enter GOAL")

def bfs(grid, start, goal):
    from collections import deque
    rows, cols = len(grid), len(grid[0])
    directions = [(0,1), (1,0), (0,-1), (-1,0)]
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

def dfs(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    directions = [(0,1), (1,0), (0,-1), (-1,0)]
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

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    import heapq
    rows, cols = len(grid), len(grid[0])
    directions = [(0,1), (1,0), (0,-1), (-1,0)]
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

# Visualize all three algorithms
plot_grid_path(grid, bfs(grid, start, goal), start, goal, 'BFS Path')
plot_grid_path(grid, dfs(grid, start, goal), start, goal, 'DFS Path')
plot_grid_path(grid, astar(grid, start, goal), start, goal, 'A* Path')
