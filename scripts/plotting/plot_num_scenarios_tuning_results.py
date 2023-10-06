#!/usr/bin/env python3
""" Plot the results of the num_scenarios tuning experiment.

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
        results_files: num_scenarios to file path

    Returns:
        results: num_scenarios to env to results list
    """
    results = {}

    for num_scenarios in results_files:
        results_for_scenarios = {}

        with open(results_files[num_scenarios], "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:  # Row is env, results
                res = list(map(lambda x: float(x), row[1:-1]))
                results_for_scenarios[row[0]] = []
                for i in range(len(res)):
                    if i % 2 == 1:
                        results_for_scenarios[row[0]].append(res[i])

        results[num_scenarios] = results_for_scenarios

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
        results: num_scenarios to env to results list
        env: The environment name to plot
    """
    print("Plotting results")

    results_list = []
    scenarios = ["100", "500"]
    for num_scenarios in scenarios:
        results_list.append(results[num_scenarios][env])

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

    plt.xticks([1, 2], scenarios)

    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_combined_results(results):
    """Plot the results combined across all environments.

    Args:
        results: num_scenarios to env to results list
    """
    print("Plotting results")

    results_list = []
    scenarios = ["100", "500"]
    for num_scenarios in scenarios:
        combined_results = []
        for env in results[num_scenarios]:
            combined_results += results[num_scenarios][env]
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
    plt.xticks([1, 2], scenarios)
    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_statistics(results):
    """Plot the means and medians over the combined results.

    Args:
        results: num_scenarios to env to results list
    """
    for num_scenarios in ["100", "500"]:
        results_for_params = []
        for env in results[num_scenarios]:
            results_for_params += results[num_scenarios][env]
        print(
            "NUM_SCENARIOS: {}, MEAN: {}, MEDIAN: {}".format(
                num_scenarios,
                np.mean(results_for_params),
                np.median(results_for_params),
            )
        )


def plot_stat_sig(results):
    """Plot statistical significance results compared to best mean.

    TODO: Get best mean, assuming POMDP for now
    Uses mannwhitneyu test, I think this is correct.
    TODO: Check its mannwhitneyu, not paired samples t-test

    Args:
        results: num_scenarios to env to results list
    """
    combined = {}
    scenarios = ["100", "500"]
    for num_scenarios in scenarios:
        results_for_params = []
        for env in results[num_scenarios]:
            results_for_params += results[num_scenarios][env]
            combined[num_scenarios] = results_for_params

    print("BEST = 500")

    for num_scenarios in scenarios:
        p = mannwhitneyu(
            combined["500"],
            combined[num_scenarios],
            alternative="greater",
        )[1]
        print(
            "500 > {}: p = {}, stat sig better = {}".format(num_scenarios, p, p < 0.05)
        )


if __name__ == "__main__":
    results_dir = os.path.abspath("../../data/results/num_scenarios_tuning")

    results_files = {}
    results_files["100"] = os.path.join(results_dir, "DESPOT_100_scenarios_results.csv")
    results_files["500"] = os.path.join(results_dir, "DESPOT_500_scenarios_results.csv")

    results = collect_results(results_files)

    plot_results(results, "four_light")
    # plot_combined_results(results)
    # plot_statistics(results)
    # plot_stat_sig(results)
