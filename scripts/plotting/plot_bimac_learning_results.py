#!/usr/bin/env python3
""" Plot the results to the BIMac learning experiment. 

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
    mle_runs = []
    with open(results_file, "r") as csv_in:
        csv_reader = csv.reader(csv_in, delimiter=",")

        for row in csv_reader:
            if row[0] == "mle":
                mle_runs.append([float(x) for x in row[1:]])

    mle_formatted = []
    for i in range(401):
        day_res = []
        for run in mle_runs:
            day_res.append(run[i])

        mle_formatted.append(day_res)

    means = [np.mean(r) for r in mle_formatted]
    x = list(range(401))
    plt.plot(x, means)
    plt.xlabel("Episode")
    plt.ylabel("Error Between MLE Estimate and Ground Truth IMac")
    plt.show()


if __name__ == "__main__":
    plot_results(
        "/home/charlie/work/coverage-plan/data/results/"
        + "BIMacLearningExperimentResults.csv"
    )
