import math

def distance_grid(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def move_one_step(current, target, grid):
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    step_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    step_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    new_pos = (current[0] + step_x, current[1] + step_y)
    if 0 <= new_pos[0] < grid.shape[0] and 0 <= new_pos[1] < grid.shape[1] and grid[new_pos] == 0:
        return new_pos
    return current

def dwa_step(current, path, grid):
    if current not in path:
        target = path[0]
        return move_one_step(current, target, grid)

    idx = path.index(current)
    if idx == len(path)-1:
        return current

    for next_pos in path[idx+1: idx+6]:
        if distance_grid(current, next_pos) <= 1.5 and grid[next_pos] == 0:
            return next_pos

    return path[idx+1]
