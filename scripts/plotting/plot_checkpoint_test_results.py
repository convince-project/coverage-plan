#!/usr/bin/env python3
""" Plot the results of the checkpoint test.

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


def collect_results(results_file):
    """Read in the results from file.

    Args:
        results_file: File with results

    Returns:
        results: imac model to results list
    """
    results = {}

    with open(results_file, "r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        for row in csv_reader:  # Row is model, results
            results[row[0]] = list(map(lambda x: float(x), row[1:-1]))

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

    plt.setp(bp["boxes"][6], color="tab:pink", linewidth=8.0)
    plt.setp(bp["caps"][12], color="tab:pink", linewidth=8.0)
    plt.setp(bp["caps"][13], color="tab:pink", linewidth=8.0)
    plt.setp(bp["whiskers"][12], color="tab:pink", linewidth=8.0)
    plt.setp(bp["whiskers"][13], color="tab:pink", linewidth=8.0)
    plt.setp(bp["medians"][6], color="tab:pink", linewidth=8.0)

    plt.setp(bp["boxes"][7], color="tab:brown", linewidth=8.0)
    plt.setp(bp["caps"][14], color="tab:brown", linewidth=8.0)
    plt.setp(bp["caps"][15], color="tab:brown", linewidth=8.0)
    plt.setp(bp["whiskers"][14], color="tab:brown", linewidth=8.0)
    plt.setp(bp["whiskers"][15], color="tab:brown", linewidth=8.0)
    plt.setp(bp["medians"][7], color="tab:brown", linewidth=8.0)


def plot_results(results, estimate):
    """Plot the results.

    Args:
        results: model to results list
        estimate: The estimate type
    """
    print("Plotting results")

    results_list = []
    models = [
        estimate + "_episode_0",
        estimate + "_episode_1",
        estimate + "_episode_5",
        estimate + "_episode_10",
        estimate + "_episode_50",
        estimate + "_episode_100",
        estimate + "_episode_150",
        "ground_truth",
    ]
    for model in models:
        results_list.append(results[model])

    box = plt.boxplot(
        results_list,
        whis=[0, 100],
        positions=[1, 2, 3, 4, 5, 6, 7, 8],
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

    plt.xticks([1, 2, 3, 4, 5, 6, 7, 8], [0, 1, 5, 10, 50, 100, 150, "GT"])

    plt.xlabel("Model", fontsize=40)
    plt.show()


def plot_statistics(results, estimate):
    """Plot the means and medians over the combined results.

    Args:
        results: model to results list
        estimate: The estimate type
    """
    for model in [
        estimate + "_episode_0",
        estimate + "_episode_1",
        estimate + "_episode_5",
        estimate + "_episode_10",
        estimate + "_episode_50",
        estimate + "_episode_100",
        estimate + "_episode_150",
        "ground_truth",
    ]:
        print(
            "METHOD: {}, MEAN: {}, MEDIAN: {}".format(
                model,
                np.mean(results[model]),
                np.median(results[model]),
            )
        )


def plot_stat_sig(results, estimate):
    """Plot statistical significance results compared to best mean.

    Compare against GT

    Args:
        results: model to results list
        estimate: The estimate type
    """
    models = [
        estimate + "_episode_0",
        estimate + "_episode_1",
        estimate + "_episode_5",
        estimate + "_episode_10",
        estimate + "_episode_50",
        estimate + "_episode_100",
        estimate + "_episode_150",
        "ground_truth",
    ]

    print("BEST = ground_truth")

    for model in models:
        p = mannwhitneyu(
            results["ground_truth"],
            results[model],
            alternative="greater",
        )[1]
        print(
            "ground_truth > {}: p = {}, stat sig better = {}".format(model, p, p < 0.05)
        )


if __name__ == "__main__":
    results_file = os.path.abspath(
        "../../data/results/prelim_exps/checkpoint_test/seven_very_heavy_results.csv"
    )

    results = collect_results(results_file)

    plot_results(results, "maximum_likelihood")
    plot_statistics(results, "maximum_likelihood")
    plot_stat_sig(results, "maximum_likelihood")
