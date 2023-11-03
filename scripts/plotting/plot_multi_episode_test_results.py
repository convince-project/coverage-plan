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
import os

import matplotlib

plt.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams.update({"font.size": 36})


def read_results(results_file):
    """Read in results file.

    Args:
        results_file: The csv file with all the results

    Return:
        means: The means
        std: The standard deviations
    """
    processed = [[]] * 300

    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            results = [float(x) for x in row[:-1]]
            for i in range(len(processed)):
                processed[i].append(results[i])

    for i in range(len(processed)):
        assert len(processed[i]) == 40

    means = [np.mean(r) for r in processed]
    std = [np.std(r) for r in processed]

    return means, std


def plot_coverage_results(gt_file, ps_file, mle_file):
    """Plot the results with std around it.

    Args:
        gt_file: Ground truth results
        ps_file: Posterior sampling results
        mle_file: Maximum likelihood results
    """

    gt_mean, gt_std = read_results(gt_file)
    ps_mean, ps_std = read_results(ps_file)
    mle_mean, mle_std = read_results(mle_file)

    # x axis of our plot
    episodes = list(range(1, 301))

    # plot the means
    (gt_line,) = plt.plot(episodes, gt_mean, linewidth=3.0, color="red")
    (ps_line,) = plt.plot(episodes, ps_mean, linewidth=3.0, color="green")
    (mle_line,) = plt.plot(episodes, mle_mean, linewidth=3.0, color="blue")

    # Show the stds
    gt_mean_plus = [gt_mean[i] + gt_std[i] for i in range(len(gt_mean))]
    gt_mean_minus = [gt_mean[i] - gt_std[i] for i in range(len(gt_mean))]
    plt.fill_between(episodes, gt_mean_plus, gt_mean_minus, alpha=0.5, color="red")

    ps_mean_plus = [ps_mean[i] + ps_std[i] for i in range(len(ps_mean))]
    ps_mean_minus = [ps_mean[i] - ps_std[i] for i in range(len(ps_mean))]
    plt.fill_between(episodes, ps_mean_plus, ps_mean_minus, alpha=0.5, color="green")

    mle_mean_plus = [mle_mean[i] + mle_std[i] for i in range(len(mle_mean))]
    mle_mean_minus = [mle_mean[i] - mle_std[i] for i in range(len(mle_mean))]
    plt.fill_between(episodes, mle_mean_plus, mle_mean_minus, alpha=0.5, color="blue")

    plt.legend(
        (gt_line, ps_line, mle_line),
        ("Ground Truth", "Posterior Sampling", "Maximum Likelihood Estimate"),
        prop={"size": 32},
    )

    plt.xlabel("Episode")
    plt.ylabel("Proportion Covered")
    plt.xlim((1, 301))
    plt.show()


def plot_imac_errors(ps_file, mle_file):
    """Plot the imac errors, i.e. the gap between our estimate and the ground truth.

    Args:
        ps_file: Posterior sampling results
        mle_file: Maximum likelihood results
    """
    ps_mean, ps_std = read_results(ps_file)
    mle_mean, mle_std = read_results(mle_file)

    # x axis of our plot
    episodes = list(range(0, 301))

    # Plot the means
    (ps_line,) = plt.plot(episodes, ps_mean, linewidth=3.0, color="red")
    (mle_line,) = plt.plot(episodes, mle_mean, linewidth=3.0, color="blue")

    # Show the stds
    ps_mean_plus = [ps_mean[i] + ps_std[i] for i in range(len(ps_mean))]
    ps_mean_minus = [ps_mean[i] - ps_std[i] for i in range(len(ps_mean))]
    plt.fill_between(episodes, ps_mean_plus, ps_mean_minus, alpha=0.5, color="red")

    mle_mean_plus = [mle_mean[i] + mle_std[i] for i in range(len(mle_mean))]
    mle_mean_minus = [mle_mean[i] - mle_std[i] for i in range(len(mle_mean))]
    plt.fill_between(episodes, mle_mean_plus, mle_mean_minus, alpha=0.5, color="blue")

    plt.legend(
        (ps_line, mle_line),
        ("Posterior Sampling", "Maximum Likelihood Estimate"),
        prop={"size": 32},
    )

    plt.xlabel("Episode")
    plt.ylabel("Absolute iMac Error")
    plt.xlim((0, 300))
    plt.show()


if __name__ == "__main__":
    results_dir = (
        "/home/charlie/work/coverage-plan/data/results/prelim_exps/"
        + "lifelong_test/ten_very_heavy_greedy"
    )

    gt_file = os.path.join(results_dir, "ground_truth_results.csv")
    ps_file = os.path.join(results_dir, "posterior_sample_results.csv")
    mle_file = os.path.join(results_dir, "maximum_likelihood_results.csv")

    plot_coverage_results(gt_file, ps_file, mle_file)
    plot_imac_errors(ps_file, mle_file)
