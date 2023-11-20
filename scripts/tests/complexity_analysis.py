#!/usr/bin/env python3
""" Example script to try and figure out coverage POMDP complexity.

Author: Charlie Street
Owner: Charlie Street
"""


def num_paths(init_cell, t):
    """Recursive function which prints the set of possible paths at t.

    Args:
        init_cell: The robot's initial position
        t: The time step we want to compute for

    Return:
        paths: A list of possible paths
    """

    if t == 0:
        return [[init_cell]]
    else:
        prev_paths = num_paths(init_cell, t - 1)

        paths = []
        for path in prev_paths:
            paths.append(path + [(path[-1][0], path[-1][1] - 1)])
            paths.append(path + [(path[-1][0], path[-1][1] + 1)])
            paths.append(path + [(path[-1][0] - 1, path[-1][1])])
            paths.append(path + [(path[-1][0] + 1, path[-1][1])])
            paths.append(path + [(path[-1])])
        return paths


def current_cells(init_cell, t):
    """Computes the cells the robot could be at at time t.

    Args:
        init_cell: The robot's initial position
        t: The time step we want to compute for

    Returns:
        possible_cells: Which cells could the robot be at?
    """

    paths = num_paths(init_cell, t)

    possible_cells = set([])
    for path in paths:
        possible_cells.add(path[-1])

    return possible_cells


def current_covered(init_cell, t):
    """Computes the possible covered sets at time t.

    Args:
        init_cell: The robot's initial position
        t: The time step we want to compute for

    Returns:
        possible_covered: Which covered sets could the robot achieve?
    """

    paths = num_paths(init_cell, t)
    print(len(paths))

    possible_covered = set([])
    for path in paths:
        possible_covered.add(frozenset(path))

    return possible_covered


if __name__ == "__main__":
    # for i in range(10):
    #    print("Num Cells at t={}: {}".format(i, len(current_cells((3, 4), i))))

    # for i in range(11):
    #    covered = list(current_covered((3, 4), i))
    #    print("Num Covered Sets at t={}: {}".format(i, len(covered)))
    #    for j in range(1, i + 2):
    #        print(
    #            "Len {}: {}".format(
    #                j, len(list(filter(lambda x: len(x) == j, covered)))
    #            )
    #        )

    test = list(current_covered((3, 4), 3))
    test.sort(key=lambda x: len(x))
    for s in test:
        print(s)
