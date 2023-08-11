#!/usr/bin/env python3
""" Script for animating a coverage planning run.

Author: Charlie Street
Owner: Charlie Street
"""

from matplotlib import animation
import matplotlib.pyplot as plt
import csv
import os

FPS = 25
TS_LEN = 10


def read_visited(in_file):
    """Read in the visited CSV file.

    Args:
        in_file: The CSV file

    Returns:
        visited: A list of (x,y) coordinates
    """

    with open(in_file) as csv_file:
        reader = csv.reader(csv_file, delimiter=",")
        return [(int(row[0]), int(row[1])) for row in reader]


def read_map(in_file):
    """Read in the map CSV file.

    Args:
        in_file: The CSV file

    Returns:
        map_dynamics: A list of dictionaries from (x,y) to occupied
    """

    map_dynamics = []
    with open(in_file) as csv_file:
        reader = csv.reader(csv_file, delimiter=",")
        for row in reader:
            map_at_ts = {}
            for i in range(1, len(row) - 1, 3):
                map_at_ts[(int(row[i]), int(row[i + 1]))] = int(row[i + 2])
            map_dynamics.append(map_at_ts)

    return map_dynamics


def draw_grid(x_len, y_len, ax):
    """Draw the environment grid.

    Args:
        x_len: X dimension of the grid
        y_len: Y dimension of the grid
        ax: Axis

    Returns:
        grid: Dict from (x,y) to square object
    """

    grid = {}
    for x in range(x_len):
        for y in range(y_len):
            grid[(x, y)] = plt.Rectangle(
                (x, (y_len - y) - 1),
                1,
                1,
                facecolor="white",
                edgecolor="black",
                zorder=10,
                linewidth=2,
            )
            ax.add_patch(grid[(x, y)])
    return grid


def animate(visited, map_dynamics, grid, robot, time_label, frame):
    """Generate a given frame in the animation.

    Args:
        visited: The visited locations of the robot
        map_dynamics: The evolution of the map
        grid: The grid drawing ((x,y)->square)
        robot: The robot drawing
        time_label: The label for the current timestep
        frame: The current frame number
    """
    # Current timestep
    ts = frame // TS_LEN

    # Where is the robot moving between?
    prev_loc = visited[ts]
    if ts < len(visited) - 1:
        next_loc = visited[ts + 1]
    else:
        next_loc = visited[ts]

    # Have robot move smoothly between grid cells
    fraction = (frame % TS_LEN) / TS_LEN
    current_x = prev_loc[0] + fraction * (next_loc[0] - prev_loc[0])
    current_y = prev_loc[1] + fraction * (next_loc[1] - prev_loc[1])
    y_len = max([k[1] for k in grid.keys()]) + 1
    robot.center = robot_grid_to_plot_pos(current_x, current_y, y_len)

    # Set time step label
    time_label.set_text("Time: {}".format(ts))

    # Set grid cells
    for cell in map_dynamics[ts]:
        if cell in visited[: ts + 1]:
            if map_dynamics[ts][cell] == 1:
                grid[cell].set_facecolor("#007500")
            else:
                grid[cell].set_facecolor("lime")
        else:
            if map_dynamics[ts][cell] == 1:
                grid[cell].set_facecolor("black")
            else:
                grid[cell].set_facecolor("white")


def robot_grid_to_plot_pos(x, y, y_len):
    """Convert an (x,y) grid position to the robot's position on the plot.

    Args:
        x: The robot's x position
        y: The robot's y position
        y_len: Y dimension on the map

    Returns:
        plot_pos: The (x,y) position on the plot
    """
    return (x + 0.5, y_len - (y + 0.5))


def generate_robot(start_loc, y_len, ax):
    """Generate the robot circle on the plot.

    Args:
        start_loc: The robot's initial (x,y) position.
        y_len: Y dimension of the grid map
        ax: The axis

    Returns:
        robot: The robot circle object
    """
    circle_loc = robot_grid_to_plot_pos(start_loc[0], start_loc[1], y_len)
    robot = plt.Circle(circle_loc, 0.45, color="blue", zorder=11, linewidth=0)
    ax.add_patch(robot)
    return robot


def generate_time(x_len, y_len, ax):
    """Generate the timestep label.

    Args:
        x_len: X dimension of the map
        ax: The axis
    """
    return ax.annotate(
        "Time: 0",
        (x_len / 2, y_len + 0.7),
        color="black",
        ha="center",
        va="center",
        zorder=9,
        size=24,
    )


def run_animation(x_len, y_len, visited_file, map_file, output_path=None):
    """Run the animation of the coverage run.

    Args:
        x_len: The x dimension of the map
        y_len: The y dimension of the map
        visited_file: The visited CSV file
        map_file: The map CSV file
        output_path: The location to store the video
    """

    # Read in from files
    visited = read_visited(visited_file)
    map_dynamics = read_map(map_file)

    # Set up axis
    fig, ax = plt.subplots()
    ax.set_aspect("equal", adjustable="box")

    # Draw the grid
    grid = draw_grid(x_len, y_len, ax)

    # Draw the robot
    robot = generate_robot(visited[0], y_len, ax)

    # Draw timestep
    time_label = generate_time(x_len, y_len, ax)

    # Now set up the animation
    frames = TS_LEN * len(visited)
    interval = (1.0 / FPS) * 1000

    anim = animation.FuncAnimation(
        fig,
        lambda i: animate(visited, map_dynamics, grid, robot, time_label, i),
        frames=int(frames),
        interval=interval,
        blit=False,
    )

    plt.xlim([0, 10])
    plt.ylim([0, 11])
    plt.axis("off")

    if output_path is not None:
        anim.save(
            output_path,
            writer="ffmpeg",
            fps=FPS,
            extra_args=["-vcodec", "libx264"],
            bitrate=-1,
            dpi=500,
        )

    plt.show()


if __name__ == "__main__":
    x_len = 10
    y_len = 10

    visited_file = os.path.join(
        "../../data/results/randomCoverageRobotExampleVisited.csv"
    )

    map_file = os.path.join("../../data/results/randomCoverageRobotExampleMap.csv")

    run_animation(
        x_len,
        y_len,
        visited_file,
        map_file,
        "../../data/videos/randomCoverageRobotRunVideo.mp4",
    )
