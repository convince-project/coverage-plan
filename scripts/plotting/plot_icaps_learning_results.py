#!/usr/bin/env python3
""" Plot the results for the ICAPS learning experiment.

Author: Charlie Street
Owner: Charlie Street
"""

import matplotlib.pyplot as plt
import numpy as np
import csv
import os

import matplotlib

plt.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams.update({"font.size": 34})


def read_results(results_file, is_imac_error=False):
    """Read in results file.

    Args:
        results_file: The csv file with all the results#
        is_imac_error: If True, 301 results, if False 300

    Return:
        means: The means
        std: The standard deviations
    """
    results = []

    num_results = 300 if not is_imac_error else 301

    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            repeat = [float(x) for x in row[:-1]]
            results.append(repeat)

    assert len(results) == 40
    for i in range(len(results)):
        assert len(results[i]) == num_results

    runs_formatted = []
    for i in range(num_results):
        day_res = []
        for repeat in results:
            day_res.append(repeat[i])
        runs_formatted.append(day_res)

    for i in range(len(runs_formatted)):
        assert len(runs_formatted[i]) == 40

    means = [np.mean(r) for r in runs_formatted]
    std = [np.std(r) for r in runs_formatted]

    return means, std


def plot_imac_errors(pm_file, ps_file):
    """Plot the imac errors, i.e. the gap between our estimate and the ground truth.

    Args:
        pm_file: Posterior mean results
        ps_file: Posterior sampling results
    """
    pm_mean, pm_std = read_results(pm_file, True)
    ps_mean, ps_std = read_results(ps_file, True)

    # x axis of our plot
    episodes = list(range(0, 301))

    # Plot the means
    (ps_line,) = plt.plot(episodes, ps_mean, linewidth=3.0, color="red")
    (pm_line,) = plt.plot(episodes, pm_mean, linewidth=3.0, color="blue")

    # Show the stds
    ps_mean_plus = [ps_mean[i] + ps_std[i] for i in range(len(ps_mean))]
    ps_mean_minus = [ps_mean[i] - ps_std[i] for i in range(len(ps_mean))]
    plt.fill_between(episodes, ps_mean_plus, ps_mean_minus, alpha=0.2, color="red")

    pm_mean_plus = [pm_mean[i] + pm_std[i] for i in range(len(pm_mean))]
    pm_mean_minus = [pm_mean[i] - pm_std[i] for i in range(len(pm_mean))]
    plt.fill_between(episodes, pm_mean_plus, pm_mean_minus, alpha=0.2, color="blue")

    plt.legend(
        (pm_line, ps_line),
        ("Posterior Mean", "Posterior Sampling"),
        prop={"size": 34},
    )

    plt.xlabel("Episode")
    plt.ylabel("iMac Learning Error")
    plt.xlim((0, 200))
    plt.show()


if __name__ == "__main__":
    results_dir = "/home/charlie/work/coverage-plan/data/results/icaps_exps/learning"
    pm_imac = os.path.join(results_dir, "posterior_mean_imac_errors.csv")
    ps_imac = os.path.join(results_dir, "posterior_sample_imac_errors.csv")
    plot_imac_errors(pm_imac, ps_imac)
