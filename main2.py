import numpy as np
import matplotlib.pyplot as plt

from astar import a_star
from rrtstar import rrt_star
from dwa import dwa_step
from utils import smooth_path

# Grid world setup
WIDTH, HEIGHT = 20, 20
grid = np.zeros((HEIGHT, WIDTH), dtype=int)

# Add static obstacles (a wall)
for i in range(5, 15):
    grid[10][i] = 1

start = (0, 0)
goal = (19, 19)

DYNAMIC_OBS_SCHEDULE = {
    2:  [(0,3), (1,3), (2,3), (3,3), (4,3), (5,3)],
    4:  [(0,6), (1,7), (2,8), (3,9), (4,10), (5,11), (6,12)],
    6:  [(5,2), (6,2), (7,2), (8,2), (9,2), (10,2), (11,2)],
    8:  [(7,7), (8,8), (9,9), (10,10), (11,11), (12,12)],
    10: [(0,12), (1,12), (2,12), (3,12), (4,12), (5,12), (6,12)],
    12: [(15,5), (16,5), (17,5), (18,5), (19,5), (14,5), (13,5)],
    14: [(12,12), (13,13), (14,14), (15,15), (16,16), (17,17), (18,18)],
    16: [(18,10), (18,11), (18,12), (18,13), (18,14), (18,15), (18,16)],
    18: [(10,18), (11,18), (12,18), (13,18), (14,18), (15,18), (16,18)],
    20: [(5,15), (6,16), (7,17), (8,18), (9,19), (10,19), (11,19)],
    22: [(15,0), (15,1), (15,2), (15,3), (15,4), (15,5), (15,6)],
    24: [(0,15), (1,16), (2,17), (3,18), (4,19), (5,19), (6,19)],
    28: [(12,0), (12,1), (12,2), (12,3), (12,4), (12,5), (12,6)],
    30: [(8,12), (9,13), (10,14), (11,15), (12,16), (13,17), (14,18)],
    34: [(6,8), (6,9), (7,8), (7,9), (8,9), (8,10), (9,9)],
    36: [(1,1), (1,2), (2,1), (2,2), (3,1), (3,2), (4,1)],
    38: [(14,8), (15,9), (16,10), (17,11), (18,12), (19,13)],
    40: [(4,14), (5,14), (6,14), (7,14), (8,14), (9,14)],
    42: [(11,6), (11,7), (12,7), (13,7), (13,8), (14,7), (15,7)],
    44: [(9,3), (10,3), (11,3), (12,3), (13,3), (14,3)],
    46: [(3,13), (3,14), (4,13), (5,13), (6,13), (7,13)],
    48: [(16,2), (17,2), (18,2), (19,2), (19,1), (18,1)],
    50: [(8,5), (9,5), (10,5), (11,5), (12,5), (13,5)],
}

def simulate(grid, start, goal, initial_path):
    plt.ion()
    robot_path = []
    pos = start
    path = path = smooth_path(initial_path, HEIGHT, WIDTH)
    step = 0
    replanning_needed = False

    # Keep track of dynamic obstacles currently active, so we can clear them each step
    active_dynamic_obstacles = set()

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
        # Clear previous dynamic obstacles (only those from schedule)
        # for obs in active_dynamic_obstacles:
        #     if grid[obs] == 1:
        #         grid[obs] = 0
        # active_dynamic_obstacles.clear()

        # Add scheduled dynamic obstacles for this step
        if step in DYNAMIC_OBS_SCHEDULE:
            for obs in DYNAMIC_OBS_SCHEDULE[step]:
                grid[obs] = 1
                active_dynamic_obstacles.add(obs)
            print(f"Dynamic obstacles appeared at step {step} at {DYNAMIC_OBS_SCHEDULE[step]}")

        # Check if replanning needed
        # If any upcoming position in next 5 steps on path is blocked, replan
        if replanning_needed or (
            pos in path and
            any(grid[p] == 1 for p in path[path.index(pos)+1:path.index(pos)+6])
        ):
            print(f"Replanning due to dynamic obstacle at step {step}...")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                print("A* failed, trying RRT*...")
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found, stopping.")
                    break
            path = path = smooth_path(new_path, HEIGHT, WIDTH)
            print(f"New path length: {len(path)}")
            replanning_needed = False

        next_pos = next_pos = dwa_step(pos, path, grid)

        # Collision check at next_pos
        if grid[next_pos] == 1:
            print("Collision detected at next_pos! Replanning again.")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found after collision. Stopping.")
                    break
            path = smooth_path(new_path)
            next_pos = next_pos = dwa_step(pos, path, grid)

        # Check if stuck (no move)
        if next_pos == pos:
            print("Robot stuck, no movement possible. Trying to replan once more.")
            new_path = a_star(grid, pos, goal)
            if new_path is None:
                new_path = rrt_star(grid, pos, goal)
                if new_path is None:
                    print("No path found, stopping.")
                    break
            path = smooth_path(new_path)
            next_pos = next_pos = dwa_step(pos, path, grid)

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
        ax.set_xlim(-0.5, WIDTH - 0.5)
        ax.set_ylim(HEIGHT - 0.5, -0.5)
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
