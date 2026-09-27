# -*- coding: utf-8 -*-
"""
Navigation and Path Planning Module.
Provides wall checking functions and BFS shortest path algorithm
for autonomous exploration of the maze.
"""

from collections import deque
from config import DIRECTIONS, SIM_H_WALLS, SIM_V_WALLS, WALL_THRESHOLD_MM, adjacent, in_bounds
import state


def is_wall_distance(dist):
    """
    Check if ToF reading corresponds to a real foam wall on cell border (25mm - 380mm).
    Accepts full physical reflection range down to 25mm so close walls are never ignored.
    """
    return 30 <= dist <= WALL_THRESHOLD_MM  # Slightly higher lower bound to ignore noise spikes


def is_wall_between(cell, direction):
    """Check if a detected foam wall exists on the border of cell in the given direction."""
    x, y = cell
    if direction == "NORTH":
        return (x, y) in state.detected_h_walls
    elif direction == "SOUTH":
        return (x, y - 1) in state.detected_h_walls
    elif direction == "EAST":
        return (x, y) in state.detected_v_walls
    elif direction == "WEST":
        return (x - 1, y) in state.detected_v_walls
    return True


def sim_has_wall_between(cell, direction):
    """Simulated check for foam walls on borders (used during --sim mode)."""
    x, y = cell
    if direction == "NORTH":
        return (x, y) in SIM_H_WALLS
    elif direction == "SOUTH":
        return (x, y - 1) in SIM_H_WALLS
    elif direction == "EAST":
        return (x, y) in SIM_V_WALLS
    elif direction == "WEST":
        return (x - 1, y) in SIM_V_WALLS
    return True


def find_path_to_nearest_unvisited(start, visited):
    """
    BFS through open passages (where no foam walls exist) to find the shortest
    path to the nearest reachable unvisited cell.
    """
    queue = deque([start])
    parent = {start: None}

    while queue:
        curr = queue.popleft()

        # Goal: A cell that has been discovered (open) but not yet visited
        if curr not in visited and curr in state.discovered_cells:
            path = []
            c = curr
            while c is not None:
                path.append(c)
                c = parent[c]
            return list(reversed(path))

        for d in DIRECTIONS:
            if is_wall_between(curr, d):
                continue  # Blocked by foam wall

            nxt = adjacent(curr, d)
            if in_bounds(nxt) and nxt not in parent:
                parent[nxt] = curr
                queue.append(nxt)

    return None
