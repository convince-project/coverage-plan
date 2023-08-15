#!/usr/bin/env python3
""" Plot the results to the multi episode test.

Hopefully this will demonstrate that robot performance is improving as it learns.
As we have a small map, the biggest improvement might be a decrease in variance.

Author: Charlie Street
Owner: Charlie Street
"""

import matplotlib.pyplot as plt
import numpy as np
import csv

import matplotlib

plt.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams.update({"font.size": 36})


def plot_results(results_file):
    """Plot the results file with std around it.

    Args:
        results_file: The csv file with all the results
    """
    runs = []
    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            runs.append([float(x) for x in row])

    # Format into each episode's results over the multiple repeats
    runs_formatted = []
    for i in range(200):
        day_res = []
        for run in runs:
            day_res.append(run[i])

        runs_formatted.append(day_res)

    # x axis of our plot
    episodes = list(range(1, 201))

    # main line of our plot
    means = [np.mean(r) for r in runs_formatted]
    plt.plot(episodes, means, linewidth=3.0)

    # Show the std as well
    std = [np.std(r) for r in runs_formatted]
    mean_plus = [means[i] + std[i] for i in range(len(means))]
    mean_minus = [means[i] - std[i] for i in range(len(means))]
    plt.fill_between(episodes, mean_plus, mean_minus, alpha=0.5)

    plt.xlabel("Episode")
    plt.ylabel("Proportion Covered")
    plt.xlim((1, 201))
    plt.show()


if __name__ == "__main__":
    plot_results(
        "/home/charlie/work/coverage-plan/data/results/" + "MultiEpisodeTestResults.csv"
    )
