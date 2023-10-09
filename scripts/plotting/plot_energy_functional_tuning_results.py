#!/usr/bin/env python3
""" Plot the results of the energy functional tuning experiments.

Author: Charlie Street
Owner: Charlie Street
"""

from scipy.stats import mannwhitneyu
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import csv
import os

plt.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams.update({"font.size": 40})


def collect_results(results_files):
    """Read in the results from the files.

    Args:
        results_files: wall points flag to file path

    Returns:
        results: wall points flag to env to results list
    """
    results = {}

    for wall_points_flag in results_files:
        results_for_val = {}

        with open(results_files[wall_points_flag], "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:  # Row is env, results
                res = list(map(lambda x: float(x), row[1:-1]))
                results_for_val[row[0]] = []
                for i in range(len(res)):
                    if i % 2 == 1:
                        results_for_val[row[0]].append(res[i])

        results[wall_points_flag] = results_for_val

    return results


def set_box_colors(bp):
    plt.setp(bp["boxes"][0], color="tab:blue", linewidth=8.0)
    plt.setp(bp["caps"][0], color="tab:blue", linewidth=8.0)
    plt.setp(bp["caps"][1], color="tab:blue", linewidth=8.0)
    plt.setp(bp["whiskers"][0], color="tab:blue", linewidth=8.0)
    plt.setp(bp["whiskers"][1], color="tab:blue", linewidth=8.0)
    plt.setp(bp["fliers"][0], color="tab:blue")
    plt.setp(bp["medians"][0], color="tab:blue", linewidth=8.0)

    plt.setp(bp["boxes"][1], color="tab:red", linewidth=8.0)
    plt.setp(bp["caps"][2], color="tab:red", linewidth=8.0)
    plt.setp(bp["caps"][3], color="tab:red", linewidth=8.0)
    plt.setp(bp["whiskers"][2], color="tab:red", linewidth=8.0)
    plt.setp(bp["whiskers"][3], color="tab:red", linewidth=8.0)
    plt.setp(bp["medians"][1], color="tab:red", linewidth=8.0)


def plot_results(results, env):
    """Plot the results for a given environment.

    Args:
        results: wall_points flag to env to results list
        env: The environment name to plot
    """
    print("Plotting results")

    results_list = []
    wall_points_vals = [False, True]
    for wall_points_flag in wall_points_vals:
        results_list.append(results[wall_points_flag][env])

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2],
        widths=0.6,
    )
    set_box_colors(box)

    plt.tick_params(
        axis="x",  # changes apply to the x-axis
        which="both",  # both major and minor ticks are affected
        bottom=True,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=True,  # labels along the bottom edge are offcd
        labelsize=28,
    )
    plt.ylabel("Proportion Covered")

    plt.xticks([1, 2], ["No Wall Points Term", "With Wall Points Term"])

    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_combined_results(results):
    """Plot the results combined across all environments.

    Args:
        results: wall_points_val to env to results list
    """
    print("Plotting results")

    results_list = []
    wall_points_vals = [False, True]
    for wall_points_flag in wall_points_vals:
        combined_results = []
        for env in results[wall_points_flag]:
            combined_results += results[wall_points_flag][env]
        results_list.append(combined_results)

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2],
        widths=0.6,
    )
    set_box_colors(box)

    plt.tick_params(
        axis="x",  # changes apply to the x-axis
        which="both",  # both major and minor ticks are affected
        bottom=True,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=True,  # labels along the bottom edge are offcd
        labelsize=28,
    )
    plt.ylabel("Proportion Covered")
    plt.xticks([1, 2], ["No Wall Points Term", "With Wall Points Term"])
    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_statistics(results):
    """Plot the means and medians over the combined results.

    Args:
        results: num_scenarios to env to results list
    """
    for wall_points_flag in [False, True]:
        results_for_params = []
        for env in results[wall_points_flag]:
            results_for_params += results[wall_points_flag][env]
        print(
            "WALL POINTS TERM: {}, MEAN: {}, MEDIAN: {}".format(
                wall_points_flag,
                np.mean(results_for_params),
                np.median(results_for_params),
            )
        )


def plot_stat_sig(results):
    """Plot statistical significance results compared to best mean.

    TODO: Get best mean, assuming wall points term for now
    Uses mannwhitneyu test, I think this is correct.
    TODO: Check its mannwhitneyu, not paired samples t-test

    Args:
        results: wall_points_flag to env to results list
    """
    combined = {}
    wall_points_vals = [False, True]
    for wall_points_flag in wall_points_vals:
        results_for_params = []
        for env in results[wall_points_flag]:
            results_for_params += results[wall_points_flag][env]
            combined[wall_points_flag] = results_for_params

    print("BEST = True")

    for wall_points_flag in wall_points_vals:
        p = mannwhitneyu(
            combined[True],
            combined[wall_points_flag],
            alternative="greater",
        )[1]
        print(
            "True > {}: p = {}, stat sig better = {}".format(
                wall_points_flag, p, p < 0.05
            )
        )


if __name__ == "__main__":
    results_dir = os.path.abspath("../../data/results/energy_functional_tuning")

    results_files = {}
    results_files[False] = os.path.join(
        results_dir, "energy_functional_no_wall_points_term_results.csv"
    )
    results_files[True] = os.path.join(
        results_dir, "energy_functional_with_wall_points_term_results.csv"
    )

    results = collect_results(results_files)

    plot_results(results, "four_light")
    plot_combined_results(results)
    # plot_statistics(results)
    # plot_stat_sig(results)
