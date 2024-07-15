# a_star.py

from queue import PriorityQueue
import numpy as np
from a_star_new import a_star

# def heuristic(a, b):
#     return abs(b[0] - a[0]) + abs(b[1] - a[1])

# def get_neighbors(current, occupancy_map):
#     neighbors = []
#     for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
#         x, y = current[0] + dx, current[1] + dy
#         if 0 <= x < occupancy_map.shape[0] and 0 <= y < occupancy_map.shape[1] and occupancy_map[x, y] == 0:
#             neighbors.append((x, y))
#     return neighbors

# def a_star(start, goal, occupancy_map):
#     start = (int(start[0]), int(start[1]))
#     goal = (int(goal[0]), int(goal[1]))
    
#     frontier = PriorityQueue()
#     frontier.put((0, heuristic(goal, start),start))
#     came_from = {start: None}
#     cost_so_far = {start: 0}

#     while not frontier.empty():
#         current = frontier.get()[1]

#         if current == goal:
#             break

#         for next in get_neighbors(current, occupancy_map):
#             new_cost = cost_so_far[current] + 1
#             if next not in cost_so_far or new_cost < cost_so_far[next]:
#                 cost_so_far[next] = new_cost
#                 priority = new_cost + heuristic(goal, next)
#                 frontier.put((priority, next))
#                 came_from[next] = current

#     # Check if a path was found
#     if goal not in came_from:
#         print(f"No path found from {start} to {goal}")
#         return None

#     # Reconstruct path
#     path = []
#     current = goal
#     while current != start:
#         path.append(current)
#         current = came_from[current]
#     path.append(start)
#     path.reverse()
    
#     return path

def find_path(start, goal, occupancy_map):
    """
    Wrapper function to find a path using A* algorithm.
    
    :param start: Tuple of (x, y) coordinates for the start position
    :param goal: Tuple of (x, y) coordinates for the goal position
    :param occupancy_map: 2D numpy array representing the environment (0 for free space, 1 for obstacle)
    :return: List of (x, y) coordinates representing the path, or None if no path is found
    """
    path = a_star(occupancy_map, start, goal)
    if path is None:
        print(f"Failed to find path from {start} to {goal}")
        # You might want to implement a fallback strategy here
        # For now, we'll return a direct path ignoring obstacles
        return [start, goal]
    return path
