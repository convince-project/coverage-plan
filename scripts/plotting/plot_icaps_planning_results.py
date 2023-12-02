#!/usr/bin/env python3
""" Plot the ICAPS planning results.

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
        results_files: method to file path

    Returns:
        results: method to env to results list
    """
    results = {}

    for pair in results_files:
        results_for_method = {}

        with open(results_files[pair], "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:  # Row is env, results
                results_for_method[row[0]] = list(map(lambda x: float(x), row[1:-1]))
                assert len(results_for_method[row[0]]) == 40

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

    plt.setp(bp["boxes"][3], color="tab:purple", linewidth=8.0)
    plt.setp(bp["caps"][6], color="tab:purple", linewidth=8.0)
    plt.setp(bp["caps"][7], color="tab:purple", linewidth=8.0)
    plt.setp(bp["whiskers"][6], color="tab:purple", linewidth=8.0)
    plt.setp(bp["whiskers"][7], color="tab:purple", linewidth=8.0)
    plt.setp(bp["medians"][3], color="tab:purple", linewidth=8.0)

    plt.setp(bp["boxes"][4], color="tab:orange", linewidth=8.0)
    plt.setp(bp["caps"][8], color="tab:orange", linewidth=8.0)
    plt.setp(bp["caps"][9], color="tab:orange", linewidth=8.0)
    plt.setp(bp["whiskers"][8], color="tab:orange", linewidth=8.0)
    plt.setp(bp["whiskers"][9], color="tab:orange", linewidth=8.0)
    plt.setp(bp["medians"][4], color="tab:orange", linewidth=8.0)

    plt.setp(bp["boxes"][5], color="tab:olive", linewidth=8.0)
    plt.setp(bp["caps"][10], color="tab:olive", linewidth=8.0)
    plt.setp(bp["caps"][11], color="tab:olive", linewidth=8.0)
    plt.setp(bp["whiskers"][10], color="tab:olive", linewidth=8.0)
    plt.setp(bp["whiskers"][11], color="tab:olive", linewidth=8.0)
    plt.setp(bp["medians"][5], color="tab:olive", linewidth=8.0)


def plot_results(results, env):
    """Plot the results for a given environment.

    Args:
        results: method to env to results list
        env: The environment name to plot
    """
    print("Plotting results")

    results_list = []
    methods = [
        "POMDP",
        "GREEDY",
        "RANDOM",
        "ENERGY",
        "BOUSTROPHEDON",
        "BOUSTROPHEDON_OFFLINE",
    ]
    for method in methods:
        results_list.append(results[method][env])

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2, 3, 4, 5, 6],
        widths=0.6,
    )
    set_box_colors(box)

    plt.tick_params(
        axis="x",  # changes apply to the x-axis
        which="both",  # both major and minor ticks are affected
        bottom=True,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=True,  # labels along the bottom edge are offcd
        labelsize=22,
    )
    plt.ylabel("Proportion of Cells Covered")

    plt.xticks(
        [1, 2, 3, 4, 5, 6],
        [
            "POMDP",
            "Greedy",
            "Random",
            "Energy\nFunctional",
            "Online\nBoustrophedon",
            "Offline\nBoustrophedon",
        ],
    )

    # plt.xlabel("Method", fontsize=40)
    plt.show()


def plot_statistics(results, env_to_select=None):
    """Plot the means and medians over the combined results.

    Args:
        results: method to env to results list
        env_to_select: The environment to plot for. When None, combine all envs together
    """
    for method in [
        "POMDP",
        "GREEDY",
        "RANDOM",
        "ENERGY",
        "BOUSTROPHEDON",
        "BOUSTROPHEDON_OFFLINE",
    ]:
        results_for_params = []
        if env_to_select is None:
            for env in results[method]:
                results_for_params += results[method][env]
        else:
            results_for_params += results[method][env_to_select]
        print(
            "METHOD: {}, MEAN: {}, MEDIAN: {}".format(
                method,
                np.mean(results_for_params),
                np.median(results_for_params),
            )
        )


def plot_stat_sig(results, env_to_select=None):
    """Plot statistical significance results compared to best mean.

    TODO: Get best mean, assuming POMDP for now
    Uses mannwhitneyu test, I think this is correct.
    TODO: Check its mannwhitneyu, not paired samples t-test

    Args:
        results: method to env to results list
        env_to_select: The environment to plot for. When None, combine all envs together
    """
    combined = {}
    methods = [
        "POMDP",
        "GREEDY",
        "RANDOM",
        "ENERGY",
        "BOUSTROPHEDON",
        "BOUSTROPHEDON_OFFLINE",
    ]
    for method in methods:
        results_for_params = []
        if env_to_select is None:
            for env in results[method]:
                results_for_params += results[method][env]
        else:
            results_for_params = results[method][env_to_select]
        combined[method] = results_for_params

    print("BEST = POMDP")

    for method in methods:
        p = mannwhitneyu(
            combined["POMDP"],
            combined[method],
            alternative="greater",
        )[1]
        print("POMDP > {}: p = {}, stat sig better = {}".format(method, p, p < 0.05))


if __name__ == "__main__":
    results_dir = os.path.abspath("../../data/results/icaps_exps/planning")

    results_files = {}
    results_files["POMDP"] = os.path.join(results_dir, "POMDP_results.csv")
    results_files["GREEDY"] = os.path.join(results_dir, "GREEDY_results.csv")
    results_files["RANDOM"] = os.path.join(results_dir, "RANDOM_results.csv")
    results_files["ENERGY"] = os.path.join(results_dir, "ENERGY_FUNCTIONAL_results.csv")
    results_files["BOUSTROPHEDON"] = os.path.join(
        results_dir, "BOUSTROPHEDON_results.csv"
    )
    results_files["BOUSTROPHEDON_OFFLINE"] = os.path.join(
        results_dir, "BOUSTROPHEDON_OFFLINE_results.csv"
    )

    results = collect_results(results_files)

    plot_results(results, "eight_very_heavy")
    plot_statistics(results, "eight_very_heavy")
    plot_stat_sig(results, "eight_very_heavy")
