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
        results: The csv file with all the results
    """
    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            results = [float(x) for x in row[:-1]]

    # Format into each episode's results over the multiple repeats
    # runs_formatted = []
    # for i in range(200):
    #    day_res = []
    #    for run in runs:
    #        day_res.append(run[i])
    #    runs_formatted.append(day_res)

    # x axis of our plot
    episodes = list(range(1, 301))

    # main line of our plot
    # means = [np.mean(r) for r in runs_formatted]
    plt.plot(episodes, results, linewidth=3.0)

    # Show the std as well
    # std = [np.std(r) for r in runs_formatted]
    # mean_plus = [means[i] + std[i] for i in range(len(means))]
    # mean_minus = [means[i] - std[i] for i in range(len(means))]
    # plt.fill_between(episodes, mean_plus, mean_minus, alpha=0.5)

    plt.xlabel("Episode")
    plt.ylabel("Proportion Covered")
    plt.xlim((1, 301))
    plt.show()


def plot_imac_errors(results_file):
    """Plot the imac errors, i.e. the gap between our estimate and the ground truth.

    Args:
        results: The csv file with all the results
    """
    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            results = [float(x) for x in row[:-1]]

    # x axis of our plot
    episodes = list(range(0, 301))

    # main line of our plot
    plt.plot(episodes, results, linewidth=3.0)

    plt.xlabel("Episode")
    plt.ylabel("Absolute Error")
    plt.xlim((0, 300))
    plt.show()


if __name__ == "__main__":
    plot_imac_errors(
        "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
        + "lifelong_test/posterior_sample_imac_errors.csv"
    )
