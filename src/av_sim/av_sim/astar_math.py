"""Pure A* path-planning algorithm — no ROS dependencies."""
import heapq
import math


def astar(start, goal, obstacles: set, grid_w: int, grid_h: int):
    """8-directional A* on an integer grid.

    Returns an ordered list of (gx, gy) cells from *start* to *goal*, or
    None if no path exists.  *obstacles* is a set of blocked (gx, gy) cells.
    """
    if goal in obstacles:
        return None

    if start == goal:
        return [start]

    MOVES = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
    ]

    def h(n):
        return math.hypot(goal[0] - n[0], goal[1] - n[1])

    open_heap = [(h(start), start)]
    came_from = {}
    g = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return list(reversed(path))

        for dx, dy, cost in MOVES:
            nb = (current[0] + dx, current[1] + dy)
            if not (0 <= nb[0] < grid_w and 0 <= nb[1] < grid_h):
                continue
            if nb in obstacles:
                continue
            ng = g[current] + cost
            if ng < g.get(nb, float('inf')):
                came_from[nb] = current
                g[nb] = ng
                heapq.heappush(open_heap, (ng + h(nb), nb))

    return None


def inflate_obstacles(obstacles: set, grid_w: int, grid_h: int,
                      radius: int = 1) -> set:
    """Expand each obstacle cell by *radius* grid cells in all 8 directions.

    Used to maintain clearance equal to the robot circumscribed radius before
    running A*.  Returns a new set; the input is not mutated.
    """
    inflated: set = set()
    for gx, gy in obstacles:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < grid_w and 0 <= ny < grid_h:
                    inflated.add((nx, ny))
    return inflated


def prune_path(path) -> list:
    """Remove collinear intermediate waypoints from *path*.

    Walks the cell list and drops any point whose removal leaves the direction
    unchanged (cross-product == 0).  Always keeps start and end.
    """
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    for i in range(1, len(path) - 1):
        prev = result[-1]
        curr = path[i]
        nxt = path[i + 1]
        cross = ((curr[0] - prev[0]) * (nxt[1] - prev[1]) -
                 (curr[1] - prev[1]) * (nxt[0] - prev[0]))
        if cross != 0:
            result.append(curr)
    result.append(path[-1])
    return result
