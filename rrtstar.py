import math
import random

class Node:
    def __init__(self, point):
        self.point = point
        self.parent = None
        self.cost = 0

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def is_collision(grid, p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    n = 1 + dx + dy
    x_inc = 1 if x2 > x1 else -1
    y_inc = 1 if y2 > y1 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if grid[x,y] == 1:
            return True
        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
    return False

def get_nearest_node(tree, point):
    min_dist = float('inf')
    nearest = None
    for node in tree:
        d = dist(node.point, point)
        if d < min_dist:
            min_dist = d
            nearest = node
    return nearest

def rrt_star(grid, start, goal, max_iter=500, step_size=3, goal_sample_rate=0.1, search_radius=5):
    start_node = Node(start)
    goal_node = Node(goal)
    tree = [start_node]

    for _ in range(max_iter):
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = (random.randint(0, grid.shape[0]-1), random.randint(0, grid.shape[1]-1))

        nearest_node = get_nearest_node(tree, sample)
        direction = (sample[0] - nearest_node.point[0], sample[1] - nearest_node.point[1])
        length = math.hypot(*direction)
        if length == 0:
            continue
        direction = (direction[0]/length, direction[1]/length)

        new_point = (int(round(nearest_node.point[0] + step_size*direction[0])),
                     int(round(nearest_node.point[1] + step_size*direction[1])))

        if not (0 <= new_point[0] < grid.shape[0] and 0 <= new_point[1] < grid.shape[1]):
            continue
        if grid[new_point] == 1:
            continue
        if is_collision(grid, nearest_node.point, new_point):
            continue

        new_node = Node(new_point)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + dist(nearest_node.point, new_point)

        near_nodes = [node for node in tree if dist(node.point, new_point) <= search_radius]
        min_cost = new_node.cost
        min_parent = nearest_node
        for near_node in near_nodes:
            if not is_collision(grid, near_node.point, new_point):
                cost_through_near = near_node.cost + dist(near_node.point, new_point)
                if cost_through_near < min_cost:
                    min_cost = cost_through_near
                    min_parent = near_node

        new_node.parent = min_parent
        new_node.cost = min_cost
        tree.append(new_node)

        if dist(new_point, goal) <= step_size:
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + dist(new_point, goal)
            tree.append(goal_node)
            path = []
            current = goal_node
            while current:
                path.append(current.point)
                current = current.parent
            path.reverse()
            return path

    return None
