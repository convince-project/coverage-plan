#!/usr/bin/env python3
""" Plot the results of the parameter tuning experiments.

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
        results_files: (bound, pruning_constant) to file path

    Returns:
        results: (bound, pruning_constant) to env to results list
    """
    results = {}

    for pair in results_files:
        results_for_method = {}

        with open(results_files[pair], "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:  # Row is env, results
                results_for_method[row[0]] = list(map(lambda x: float(x), row[1:-1]))

        results[pair] = results_for_method

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

    plt.setp(bp["boxes"][2], color="tab:green", linewidth=8.0)
    plt.setp(bp["caps"][4], color="tab:green", linewidth=8.0)
    plt.setp(bp["caps"][5], color="tab:green", linewidth=8.0)
    plt.setp(bp["whiskers"][4], color="tab:green", linewidth=8.0)
    plt.setp(bp["whiskers"][5], color="tab:green", linewidth=8.0)
    plt.setp(bp["medians"][2], color="tab:green", linewidth=8.0)

    plt.setp(bp["boxes"][3], color="tab:gray", linewidth=8.0)
    plt.setp(bp["caps"][6], color="tab:gray", linewidth=8.0)
    plt.setp(bp["caps"][7], color="tab:gray", linewidth=8.0)
    plt.setp(bp["whiskers"][6], color="tab:gray", linewidth=8.0)
    plt.setp(bp["whiskers"][7], color="tab:gray", linewidth=8.0)
    plt.setp(bp["medians"][3], color="tab:gray", linewidth=8.0)

    plt.setp(bp["boxes"][4], color="tab:purple", linewidth=8.0)
    plt.setp(bp["caps"][8], color="tab:purple", linewidth=8.0)
    plt.setp(bp["caps"][9], color="tab:purple", linewidth=8.0)
    plt.setp(bp["whiskers"][8], color="tab:purple", linewidth=8.0)
    plt.setp(bp["whiskers"][9], color="tab:purple", linewidth=8.0)
    plt.setp(bp["medians"][4], color="tab:purple", linewidth=8.0)

    plt.setp(bp["boxes"][5], color="tab:orange", linewidth=8.0)
    plt.setp(bp["caps"][10], color="tab:orange", linewidth=8.0)
    plt.setp(bp["caps"][11], color="tab:orange", linewidth=8.0)
    plt.setp(bp["whiskers"][10], color="tab:orange", linewidth=8.0)
    plt.setp(bp["whiskers"][11], color="tab:orange", linewidth=8.0)
    plt.setp(bp["medians"][5], color="tab:orange", linewidth=8.0)

    plt.setp(bp["boxes"][6], color="tab:brown", linewidth=8.0)
    plt.setp(bp["caps"][12], color="tab:brown", linewidth=8.0)
    plt.setp(bp["caps"][13], color="tab:brown", linewidth=8.0)
    plt.setp(bp["whiskers"][12], color="tab:brown", linewidth=8.0)
    plt.setp(bp["whiskers"][13], color="tab:brown", linewidth=8.0)
    plt.setp(bp["medians"][6], color="tab:brown", linewidth=8.0)

    plt.setp(bp["boxes"][7], color="tab:olive", linewidth=8.0)
    plt.setp(bp["caps"][14], color="tab:olive", linewidth=8.0)
    plt.setp(bp["caps"][15], color="tab:olive", linewidth=8.0)
    plt.setp(bp["whiskers"][14], color="tab:olive", linewidth=8.0)
    plt.setp(bp["whiskers"][15], color="tab:olive", linewidth=8.0)
    plt.setp(bp["medians"][7], color="tab:olive", linewidth=8.0)

    plt.setp(bp["boxes"][8], color="tab:cyan", linewidth=8.0)
    plt.setp(bp["caps"][16], color="tab:cyan", linewidth=8.0)
    plt.setp(bp["caps"][17], color="tab:cyan", linewidth=8.0)
    plt.setp(bp["whiskers"][16], color="tab:cyan", linewidth=8.0)
    plt.setp(bp["whiskers"][17], color="tab:cyan", linewidth=8.0)
    plt.setp(bp["medians"][8], color="tab:cyan", linewidth=8.0)

    plt.setp(bp["boxes"][9], color="tab:pink", linewidth=8.0)
    plt.setp(bp["caps"][18], color="tab:pink", linewidth=8.0)
    plt.setp(bp["caps"][19], color="tab:pink", linewidth=8.0)
    plt.setp(bp["whiskers"][18], color="tab:pink", linewidth=8.0)
    plt.setp(bp["whiskers"][19], color="tab:pink", linewidth=8.0)
    plt.setp(bp["medians"][9], color="tab:pink", linewidth=8.0)


def plot_results(results, env):
    """Plot the results for a given environment.

    Args:
        results: (bound, pruning_constant) to env to results list
        env: The environment name to plot
    """
    print("Plotting results")

    results_list = []
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            results_list.append(results[(bound, pruning_constant)][env])

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        widths=0.6,
    )
    set_box_colors(box)

    plt.tick_params(
        axis="x",  # changes apply to the x-axis
        which="both",  # both major and minor ticks are affected
        bottom=True,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=True,  # labels along the bottom edge are offcd
        labelsize=12,
    )
    plt.ylabel("Proportion Covered")

    parameters = []
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            parameters.append("{}, {}".format(bound, pruning_constant))

    plt.xticks([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], parameters)

    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_combined_results(results):
    """Plot the results combined across all environments.

    Args:
        results: (bound, pruning_constant) to env to results list
    """
    print("Plotting results")

    results_list = []
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            combined_results = []
            for env in results[(bound, pruning_constant)]:
                combined_results += results[(bound, pruning_constant)][env]
            results_list.append(combined_results)

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        widths=0.6,
    )
    set_box_colors(box)

    plt.tick_params(
        axis="x",  # changes apply to the x-axis
        which="both",  # both major and minor ticks are affected
        bottom=True,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=True,  # labels along the bottom edge are offcd
        labelsize=12,
    )
    plt.ylabel("Proportion Covered")

    parameters = []
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            parameters.append("{}, {}".format(bound, pruning_constant))

    plt.xticks([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], parameters)

    plt.xlabel("Parameters", fontsize=40)
    plt.show()


def plot_statistics(results):
    """Plot the means and medians over the combined results.

    Args:
        results: (bound, pruning_constant) to env to results list
    """
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            results_for_params = []
            for env in results[(bound, pruning_constant)]:
                results_for_params += results[(bound, pruning_constant)][env]
            print(
                "BOUND: {}, PRUNING CONSTANT: {}, MEAN: {}, MEDIAN: {}".format(
                    bound,
                    pruning_constant,
                    np.mean(results_for_params),
                    np.median(results_for_params),
                )
            )


def plot_stat_sig(results):
    """Plot statistical significance results compared to best mean.

    Best mean is DEFAULT, 0.1.
    Uses mannwhitneyu test, I think this is correct.
    TODO: Check its mannwhitneyu, not paired samples t-test

    Args:
        results: (bound, pruning_constant) to env to results list
    """
    combined = {}
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            results_for_params = []
            for env in results[(bound, pruning_constant)]:
                results_for_params += results[(bound, pruning_constant)][env]
                combined[(bound, pruning_constant)] = results_for_params

    print("BEST = (DEFAULT, 0.1)")

    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            p = mannwhitneyu(
                combined[("DEFAULT", 0.1)],
                combined[(bound, pruning_constant)],
                alternative="greater",
            )[1]
            print(
                "(DEFAULT, 0.1) > ({}, {}): p = {}, stat sig better = {}".format(
                    bound, pruning_constant, p, p < 0.05
                )
            )


if __name__ == "__main__":
    results_dir = os.path.abspath("../../data/results/prelim_exps")

    results_files = {}

    # Collect all the results files
    for bound in ["TRIVIAL", "DEFAULT"]:
        for pruning_constant in [0, 0.01, 0.1, 1, 10]:
            results_files[(bound, pruning_constant)] = os.path.join(
                results_dir,
                "DESPOT_pruning_{}_bound_{}_results.csv".format(
                    pruning_constant, bound
                ),
            )

    results = collect_results(results_files)

    # plot_results(results, "five_heavy")
    # plot_combined_results(results)
    plot_statistics(results)
    plot_stat_sig(results)
