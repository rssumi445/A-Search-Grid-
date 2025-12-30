import heapq

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    """
    A* search on a 2D grid.
    grid: 2D list (0 = free, 1 = blocked)
    start, goal: (row, col)
    returns: path list from start to goal, or None
    """
    rows, cols = len(grid), len(grid[0])

    open_heap = []
    heapq.heappush(open_heap, (0, start))  # (f_score, node)

    came_from = {}
    g_score = {start: 0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        r, c = current
        neighbors = [(r+1, c), (r-1, c), (r, c+1), (r, c-1)]

        for nr, nc in neighbors:
            # check bounds and obstacles
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                neighbor = (nr, nc)
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

    return None

# Example usage
if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
    ]

    start = (0, 0)
    goal = (4, 4)

    path = astar(grid, start, goal)
    print("Path:", path)
