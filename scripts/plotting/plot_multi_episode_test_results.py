#!/usr/bin/env python3
""" Plot the results to the multi episode test.

Hopefully this will demonstrate that robot performance is improving as it learns.
As we have a small map, the biggest improvement might be a decrease in variance.

Author: Charlie Street
Owner: Charlie Street
"""

from scipy.stats import mannwhitneyu
import matplotlib.pyplot as plt
import numpy as np
import csv

import matplotlib

plt.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams.update({"font.size": 36})


def read_results(results_file):
    """Read in results file.

    Args:
        results_file: The csv file with all the results

    Return:
        results: The results
    """
    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            results = [float(x) for x in row[:-1]]

    return results


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
    # plot_imac_errors(
    #    "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
    #    + "lifelong_test/posterior_sample_imac_errors.csv"
    # )

    # plot_results(
    #    "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
    #    + "lifelong_test/posterior_sample_results.csv"
    # )

    # Get a rough idea of the means and medians
    results_ps = read_results(
        "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
        + "lifelong_test/posterior_sample_results.csv"
    )

    results_mle = read_results(
        "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
        + "lifelong_test/maximum_likelihood_results.csv"
    )

    results_gt = read_results(
        "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
        + "lifelong_test/ground_truth_results.csv"
    )

    print(
        "POSTERIOR SAMPLE - MEAN: {}; MEDIAN: {}".format(
            np.mean(results_ps), np.median(results_ps)
        )
    )

    print(
        "MAXIMUM LIKELIHOOD - MEAN: {}; MEDIAN: {}".format(
            np.mean(results_mle), np.median(results_mle)
        )
    )

    print(
        "GROUND TRUTH - MEAN: {}; MEDIAN: {}".format(
            np.mean(results_gt), np.median(results_gt)
        )
    )

    print("BEST MEAN - GROUND TRUTH")
    print(
        "GROUND TRUTH > POSTERIOR SAMPLE: p={}".format(
            mannwhitneyu(
                results_gt,
                results_ps,
                alternative="greater",
            )[1]
        )
    )
    print(
        "GROUND TRUTH > MAXIMUM LIKELIHOOD: p={}".format(
            mannwhitneyu(
                results_gt,
                results_mle,
                alternative="greater",
            )[1]
        )
    )

    print(
        "MAXIMUM LIKELIHOOD > POSTERIOR SAMPLE: p={}".format(
            mannwhitneyu(
                results_mle,
                results_ps,
                alternative="greater",
            )[1]
        )
    )
