import heapq
import numpy as np

FREE = 0  # 0 = free, anything else = blocked

def get_neighbors(x, y, grid):
    """Get 4-connected neighbors (left, right, up, down)."""
    h, w = grid.shape  # h = rows (y), w = cols (x)

    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        # Check bounds
        if 0 <= nx < w and 0 <= ny < h:
            # grid indexed as [row, col] = [y, x]
            if grid[ny, nx] == FREE:
                yield nx, ny

def dijkstra(grid, start, goal):
    """
    Dijkstra on a 2D grid.

    - grid[y, x] == 0 => free
    - grid[y, x] != 0 => blocked
    - start, goal are (x, y) in grid coordinates.
    """
    sx, sy = start  # (x, y)
    gx, gy = goal

    h, w = grid.shape  # (rows, cols) = (y, x)

    # Distance and parent maps
    dist = np.full((h, w), np.inf)
    parent = np.full((h, w, 2), -1, dtype=int)

    # Priority queue of (cost, x, y)
    pq = []
    dist[sy, sx] = 0.0
    heapq.heappush(pq, (0.0, sx, sy))

    while pq:
        curr_cost, x, y = heapq.heappop(pq)

        # Early exit if we've reached the goal
        if (x, y) == (gx, gy):
            break

        # Skip stale entries
        if curr_cost > dist[y, x]:
            continue

        # Relax neighbors
        for nx, ny in get_neighbors(x, y, grid):
            new_cost = curr_cost + 1.0

            if new_cost < dist[ny, nx]:
                dist[ny, nx] = new_cost
                parent[ny, nx] = [x, y]  # store parent as (x, y)
                heapq.heappush(pq, (new_cost, nx, ny))

    # If goal still has infinite distance, no path
    if not np.isfinite(dist[gy, gx]):
        return []  # No path found

    # Reconstruct path from goal back to start
    path = []
    x, y = gx, gy
    while True:
        path.append((x, y))
        if (x, y) == (sx, sy):
            break
        px, py = parent[y, x]  # read parent from [row, col]
        if px < 0:
            # Safety guard: parent not set -> no valid path
            return []
        x, y = px, py

    path.reverse()
    return path
