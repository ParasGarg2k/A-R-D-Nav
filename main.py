import numpy as np
import matplotlib.pyplot as plt
from astar import a_star
from rrtstar import rrt_star
from dwa import dwa_step
from utils import smooth_path

WIDTH, HEIGHT = 20, 20
grid = np.zeros((HEIGHT, WIDTH), dtype=int)

for i in range(5, 15):
    grid[10][i] = 1

start = (0, 0)
goal = (19, 19)

DYNAMIC_OBS_SCHEDULE = {
    2:  [(0, 3), (1, 3)],
    4:  [(0, 6), (1, 7), (2, 8)],
    6:  [(5, 2), (6, 2)],
    8:  [(7, 7), (8, 8)],
    10: [(0, 12), (1, 12)],
    12: [(15, 5), (16, 5)],
    14: [(12, 12), (13, 13)],
    16: [(18, 10), (18, 11)],
    18: [(10, 18), (11, 18)],
    30: [(8, 12), (9, 13)],
    38: [(14, 8), (15, 9)],
    40: [(4, 14), (5, 14)],
    42: [(11, 6), (11, 7)],
    44: [(9, 3), (10, 3)],
    46: [(3, 13), (3, 14)],
    48: [(16, 2), (17, 2)],
    50: [(8, 5), (9, 5)],
}


# Moving obstacles
MOVING_OBSTACLES = [
    {'pos': (9, 6), 'dir': (1, 1)},
    {'pos': (10, 10), 'dir': (1, 0)},
    {'pos': (15, 10), 'dir': (1, 0)},
    {'pos': (19, 13), 'dir': (1, 1)},
]

def simulate(grid, start, goal, initial_path):
    plt.ion()
    robot_path = []
    pos = start
    path = smooth_path(initial_path, HEIGHT, WIDTH)
    step = 0
    replanning_needed = False

    fig, ax = plt.subplots()

    def onclick(event):
        nonlocal replanning_needed
        if event.inaxes == ax:
            x = int(round(event.xdata))
            y = int(round(event.ydata))
            if 0 <= y < HEIGHT and 0 <= x < WIDTH:
                if grid[y, x] == 0 and (y, x) != pos and (y, x) != goal:
                    grid[y, x] = 1
                    print(f"User added dynamic obstacle at {(y, x)}")
                    replanning_needed = True

    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    while pos != goal:
        # Update moving obstacles
        for mob in MOVING_OBSTACLES:
            y, x = mob['pos']
            dy, dx = mob['dir']
            if 0 <= y < HEIGHT and 0 <= x < WIDTH:
                grid[y][x] = 0  # Clear old position

            new_y, new_x = y + dy, x + dx
            if 0 <= new_y < HEIGHT and 0 <= new_x < WIDTH:
                mob['pos'] = (new_y, new_x)
            else:
                mob['dir'] = (-dy, -dx)
                mob['pos'] = (y - dy, x - dx)

            grid[mob['pos']] = 1

        # Add scheduled dynamic obstacles
        if step in DYNAMIC_OBS_SCHEDULE:
            for obs in DYNAMIC_OBS_SCHEDULE[step]:
                grid[obs] = 1
            print(f"Dynamic obstacles appeared at step {step} at {DYNAMIC_OBS_SCHEDULE[step]}")

        if replanning_needed or (pos in path and any(grid[p] == 1 for p in path[path.index(pos)+1:path.index(pos)+6])):
            print(f"Replanning due to dynamic obstacle at step {step}...")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                print("A* failed, trying RRT*...")
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found, stopping.")
                    break
            path = smooth_path(new_path, HEIGHT, WIDTH)
            replanning_needed = False

        next_pos = dwa_step(pos, path, grid)

        if grid[next_pos] == 1:
            print("Collision detected at next_pos! Replanning again.")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found after collision. Stopping.")
                    break
            path = smooth_path(new_path, HEIGHT, WIDTH)
            next_pos = dwa_step(pos, path, grid)

        if next_pos == pos:
            print("Robot stuck, trying to replan once more.")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found, stopping.")
                    break
            path = smooth_path(new_path, HEIGHT, WIDTH)
            next_pos = dwa_step(pos, path, grid)
            if next_pos == pos:
                print("Still stuck after replanning. Stopping.")
                break

        print(f"Step {step}: Moving from {pos} to {next_pos}")

        pos = next_pos
        robot_path.append(pos)

        ax.clear()
        ax.imshow(grid, cmap='gray_r')
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='Planned Path')
        rx = [p[1] for p in robot_path]
        ry = [p[0] for p in robot_path]
        ax.plot(rx, ry, 'go', label='Robot Path')
        ax.plot(start[1], start[0], 'bo', markersize=10, label='Start')
        ax.plot(goal[1], goal[0], 'ro', markersize=10, label='Goal')
        ax.set_title(f"Step {step}")
        ax.legend(loc='upper left')
        plt.pause(0.25)

        step += 1

    plt.ioff()
    fig.canvas.mpl_disconnect(cid)
    return robot_path

if __name__ == "__main__":
    initial_path = a_star(grid, start, goal)
    if initial_path is None:
        print("Initial A* path not found, trying RRT*...")
        initial_path = rrt_star(grid, start, goal)
        if initial_path is None:
            print("No initial path found.")
            exit()

    final_path = simulate(grid, start, goal, initial_path)
    print("Simulation finished, final path length:", len(final_path))
    plt.show()
